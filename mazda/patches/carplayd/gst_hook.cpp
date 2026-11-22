// SPDX-License-Identifier: AGPL-3.0-or-later
//
// GStreamer 0.10 interpose for carplayd.
//
// CarPlay video path (libCarPlayPlatformScreen.so):
//   appsrc ! capsfilter ! vpudec ! mfw_v4lsink
//
// mfw_gst_v4lsink_show_frame is not exported, so we:
//   1. Interpose gst_element_factory_make — when the sink is created,
//      attach a buffer probe on its "sink" pad.
//   2. Optionally interpose gst_element_link — when carplay_ocr_force_linear
//      is set, insert mfw_ipucsc (+ I420 capsfilter) before the V4L sink so
//      the probe sees CPU-readable planar frames instead of tiled VPU output.
//
// Every GST/GLib symbol is resolved with dlsym(RTLD_NEXT) so we do not
// link against libgstreamer (which would defeat the interpose).

#include "gst_decls.h"
#include "ocr.h"

#define LOG_TAG "GST"
#include "log.h"
#include "common/config.h"
#include "common/preload.h"

#include <dlfcn.h>
#include <string.h>

#include <atomic>

namespace {

using FnFactoryMake = GstElement *(*)(const char *factory, const char *name);
using FnGetStaticPad = GstPad *(*)(GstElement *element, const char *name);
using FnAddBufferProbe = gulong (*)(GstPad *pad, GCallback handler,
                                    gpointer data);
using FnObjectUnref = void (*)(void *object);
using FnBufferGetCaps = GstCaps *(*)(GstBuffer *buffer);
using FnCapsGetStructure = GstStructure *(*)(const GstCaps *caps, guint index);
using FnStructGetInt = gboolean (*)(const GstStructure *structure,
                                    const char *fieldname, gint *value);
using FnStructGetFourcc = gboolean (*)(const GstStructure *structure,
                                       const char *fieldname, guint32 *value);
using FnElementLink = gboolean (*)(GstElement *src, GstElement *dest);
using FnBinAdd = gboolean (*)(GstBin *bin, GstElement *element);
using FnObjectGetParent = GstObject *(*)(GstObject *object);
using FnCapsFromString = GstCaps *(*)(const char *string);
using FnGObjectSet = void (*)(gpointer object, const char *first_property_name,
                              ...);

FnFactoryMake      real_factory_make      = nullptr;
FnGetStaticPad     real_get_static_pad    = nullptr;
FnAddBufferProbe   real_add_buffer_probe  = nullptr;
FnObjectUnref      real_object_unref      = nullptr;
FnBufferGetCaps    real_buffer_get_caps   = nullptr;
FnCapsGetStructure real_caps_get_structure = nullptr;
FnStructGetInt     real_struct_get_int    = nullptr;
FnStructGetFourcc  real_struct_get_fourcc = nullptr;
FnElementLink      real_element_link      = nullptr;
FnBinAdd           real_bin_add           = nullptr;
FnObjectGetParent  real_object_get_parent = nullptr;
FnCapsFromString   real_caps_from_string  = nullptr;
FnGObjectSet       real_g_object_set      = nullptr;

std::atomic<GstElement *> g_v4l_sink{nullptr};
std::atomic<bool>         g_probe_attached{false};
std::atomic<bool>         g_linear_inserted{false};

void *lib_handle(const char *soname)
{
    // Prefer an already-mapped copy (carplayd has linked GST by the time
    // factory_make runs). Fall back to a real open if the ctor races.
    void *h = dlopen(soname, RTLD_NOW | RTLD_NOLOAD);
    if (!h) h = dlopen(soname, RTLD_NOW);
    return h;
}

template <typename T>
T resolve_next(const char *name)
{
    // RTLD_NEXT skips this LD_PRELOAD image — required for symbols we
    // ourselves export (factory_make / element_link). A handle-based
    // dlsym would walk preloads first and recurse into our shim.
    void *p = dlsym(RTLD_NEXT, name);
    if (!p) {
        LOGE("resolve: %s not found via RTLD_NEXT", name);
    }
    return reinterpret_cast<T>(p);
}

template <typename T>
T resolve_lib(void *handle, const char *name)
{
    void *p = nullptr;
    if (handle) p = dlsym(handle, name);
    if (!p) p = dlsym(RTLD_NEXT, name);
    if (!p) {
        LOGE("resolve: %s not found", name);
    }
    return reinterpret_cast<T>(p);
}

void ensure_resolved()
{
    static std::atomic<bool> once{false};
    if (once.load(std::memory_order_acquire)) return;

    void *gst = lib_handle("libgstreamer-0.10.so.0");
    void *gobj = lib_handle("libgobject-2.0.so.0");

    real_factory_make       = resolve_next<FnFactoryMake>("gst_element_factory_make");
    real_element_link       = resolve_next<FnElementLink>("gst_element_link");
    real_get_static_pad     = resolve_lib<FnGetStaticPad>(gst, "gst_element_get_static_pad");
    real_add_buffer_probe   = resolve_lib<FnAddBufferProbe>(gst, "gst_pad_add_buffer_probe");
    real_object_unref       = resolve_lib<FnObjectUnref>(gst, "gst_object_unref");
    real_buffer_get_caps    = resolve_lib<FnBufferGetCaps>(gst, "gst_buffer_get_caps");
    real_caps_get_structure = resolve_lib<FnCapsGetStructure>(gst, "gst_caps_get_structure");
    real_struct_get_int     = resolve_lib<FnStructGetInt>(gst, "gst_structure_get_int");
    real_struct_get_fourcc  = resolve_lib<FnStructGetFourcc>(gst, "gst_structure_get_fourcc");
    real_bin_add            = resolve_lib<FnBinAdd>(gst, "gst_bin_add");
    real_object_get_parent  = resolve_lib<FnObjectGetParent>(gst, "gst_object_get_parent");
    real_caps_from_string   = resolve_lib<FnCapsFromString>(gst, "gst_caps_from_string");
    real_g_object_set       = resolve_lib<FnGObjectSet>(gobj, "g_object_set");

    once.store(true, std::memory_order_release);
}

splim_ocr::PixelFormat fourcc_to_fmt(guint32 fcc)
{
    // GStreamer fourccs are little-endian integer encodings of the ASCII tag.
    auto tag = [&](char a, char b, char c, char d) -> guint32 {
        return (static_cast<guint32>(a)) |
               (static_cast<guint32>(b) << 8) |
               (static_cast<guint32>(c) << 16) |
               (static_cast<guint32>(d) << 24);
    };
    if (fcc == tag('I', '4', '2', '0')) return splim_ocr::FMT_I420;
    if (fcc == tag('Y', 'V', '1', '2')) return splim_ocr::FMT_YV12;
    if (fcc == tag('N', 'V', '1', '2')) return splim_ocr::FMT_NV12;
    if (fcc == tag('Y', '8', '0', '0')) return splim_ocr::FMT_Y800;
    return splim_ocr::FMT_UNKNOWN;
}

gboolean carplayd_ocr_on_buffer(GstPad * /*pad*/, GstBuffer *buffer,
                                gpointer /*ud*/)
{
    if (!libpatch_config::carplay_ocr()) return TRUE;
    if (!buffer || !real_buffer_get_caps) return TRUE;

    auto *view = reinterpret_cast<GstBufferView *>(buffer);
    if (!view->data || view->size == 0) return TRUE;

    gint width = 0, height = 0;
    guint32 fourcc = 0;
    splim_ocr::PixelFormat fmt = splim_ocr::FMT_UNKNOWN;

    GstCaps *caps = real_buffer_get_caps(buffer);
    if (caps && real_caps_get_structure && real_struct_get_int) {
        GstStructure *st = real_caps_get_structure(caps, 0);
        if (st) {
            real_struct_get_int(st, "width", &width);
            real_struct_get_int(st, "height", &height);
            if (real_struct_get_fourcc)
                real_struct_get_fourcc(st, "format", &fourcc);
            fmt = fourcc_to_fmt(fourcc);
        }
        if (real_object_unref) real_object_unref(caps);
    }

    if (width <= 0 || height <= 0) return TRUE;

    splim_ocr::FrameView frame;
    frame.data   = view->data;
    frame.size   = view->size;
    frame.width  = width;
    frame.height = height;
    frame.fmt    = fmt;
    splim_ocr::process_frame(frame);
    return TRUE; // never drop
}

void attach_probe(GstElement *sink)
{
    if (!sink || g_probe_attached.load(std::memory_order_relaxed)) return;
    if (!real_get_static_pad || !real_add_buffer_probe) return;

    GstPad *pad = real_get_static_pad(sink, "sink");
    if (!pad) {
        LOGE("mfw_v4lsink: no static 'sink' pad yet");
        return;
    }
    real_add_buffer_probe(
        pad, reinterpret_cast<GCallback>(
                 reinterpret_cast<void *>(&carplayd_ocr_on_buffer)),
        nullptr);
    // Keep the pad ref — probe holds the lifetime for the element.
    g_probe_attached.store(true, std::memory_order_relaxed);
    LOGD("buffer probe attached to mfw_v4lsink sink pad");
}

// Insert mfw_ipucsc ! capsfilter(I420) between src and the V4L sink so the
// probe observes linear planar frames. Only used when
// carplay_ocr_force_linear=true (tiled VPU output otherwise).
gboolean insert_linear_path(GstElement *src, GstElement *sink)
{
    if (!real_factory_make || !real_bin_add || !real_element_link ||
        !real_object_get_parent || !real_caps_from_string || !real_g_object_set) {
        LOGE("force_linear: missing GST/GObject symbols");
        return FALSE;
    }

    GstObject *parent_obj = real_object_get_parent(reinterpret_cast<GstObject *>(sink));
    if (!parent_obj) {
        LOGE("force_linear: sink has no parent bin yet");
        return FALSE;
    }
    GstBin *bin = reinterpret_cast<GstBin *>(parent_obj);

    GstElement *csc = real_factory_make("mfw_ipucsc", "splim_ipucsc");
    GstElement *cf  = real_factory_make("capsfilter", "splim_i420");
    if (!csc || !cf) {
        LOGE("force_linear: factory_make mfw_ipucsc/capsfilter failed");
        if (real_object_unref) {
            if (csc) real_object_unref(csc);
            if (cf)  real_object_unref(cf);
            real_object_unref(parent_obj);
        }
        return FALSE;
    }

    GstCaps *caps = real_caps_from_string(
        "video/x-raw-yuv,format=(fourcc)I420");
    if (caps) {
        real_g_object_set(cf, "caps", caps, static_cast<void *>(0));
        if (real_object_unref) real_object_unref(caps);
    }

    if (!real_bin_add(bin, csc) || !real_bin_add(bin, cf)) {
        LOGE("force_linear: gst_bin_add failed");
        if (real_object_unref) real_object_unref(parent_obj);
        return FALSE;
    }

    if (!real_element_link(src, csc) ||
        !real_element_link(csc, cf) ||
        !real_element_link(cf, sink)) {
        LOGE("force_linear: link src->ipucsc->capsfilter->sink failed");
        if (real_object_unref) real_object_unref(parent_obj);
        return FALSE;
    }

    if (real_object_unref) real_object_unref(parent_obj);
    g_linear_inserted.store(true, std::memory_order_relaxed);
    LOGD("force_linear: inserted mfw_ipucsc ! I420 capsfilter before sink");
    return TRUE;
}

} // namespace

extern "C" {

PRELOAD_EXPORT
GstElement *gst_element_factory_make(const char *factory, const char *name)
{
    ensure_resolved();
    if (!real_factory_make) return nullptr;

    GstElement *el = real_factory_make(factory, name);
    if (!el) return el;

    if (factory && strcmp(factory, "mfw_v4lsink") == 0 &&
        libpatch_config::carplay_ocr()) {
        LOGD("factory_make mfw_v4lsink name=%s -> %p",
             name ? name : "(null)", static_cast<void *>(el));
        g_v4l_sink.store(el, std::memory_order_relaxed);
        attach_probe(el);
    }
    return el;
}

PRELOAD_EXPORT
gboolean gst_element_link(GstElement *src, GstElement *dest)
{
    ensure_resolved();
    if (!real_element_link) return FALSE;

    GstElement *sink = g_v4l_sink.load(std::memory_order_relaxed);
    if (libpatch_config::carplay_ocr() &&
        libpatch_config::carplay_ocr_force_linear() &&
        sink && dest == sink &&
        !g_linear_inserted.load(std::memory_order_relaxed)) {
        if (insert_linear_path(src, dest)) {
            return TRUE;
        }
        LOGW("force_linear failed — falling back to direct link");
    }
    return real_element_link(src, dest);
}

} // extern "C"
