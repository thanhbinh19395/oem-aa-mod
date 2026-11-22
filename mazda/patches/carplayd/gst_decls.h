// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Minimal GStreamer 0.10 declarations for the carplayd OCR preload.
//
// We deliberately do NOT link against libgstreamer: the shim is
// LD_PRELOAD'd into carplayd and resolves every GST symbol with dlsym
// so the interpose of gst_element_factory_make binds correctly. Only the
// bits we touch are declared here. GstBuffer field offsets were verified
// against the CMU's libgstreamer-0.10.so.0.29.0 (data@+0x10, size@+0x14,
// caps@+0x28).

#ifndef LIBPATCH_CARPLAYD_GST_DECLS_H
#define LIBPATCH_CARPLAYD_GST_DECLS_H

#include <stdint.h>

extern "C" {

typedef struct _GstElement GstElement;
typedef struct _GstPad     GstPad;
typedef struct _GstBuffer  GstBuffer;
typedef struct _GstCaps    GstCaps;
typedef struct _GstStructure GstStructure;
typedef struct _GstBin     GstBin;
typedef struct _GstObject  GstObject;
typedef struct _GObject    GObject;

typedef int       gboolean;
typedef void     *gpointer;
typedef uint32_t  guint;
typedef uint64_t  gulong;
typedef int32_t   gint;
typedef char      gchar;
typedef uint32_t  GQuark;
typedef uint32_t  guint32;

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif

// Layout fragment matching CMU libgstreamer-0.10.so.0.29.0.
struct GstBufferView {
    uint8_t  _pad0[0x10];
    uint8_t *data;     // +0x10
    uint32_t size;     // +0x14
    uint32_t maxsize;  // +0x18
    uint8_t  _pad1[0x0c];
    GstCaps *caps;     // +0x28
};

typedef gboolean (*GstPadBufferProbeFunc)(GstPad *pad, GstBuffer *buffer,
                                          gpointer user_data);

typedef void (*GCallback)(void);

} // extern "C"

#endif // LIBPATCH_CARPLAYD_GST_DECLS_H
