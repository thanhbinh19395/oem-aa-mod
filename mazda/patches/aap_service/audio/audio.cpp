// SPDX-License-Identifier: AGPL-3.0-or-later
// Lower only Android Auto playback's ALSA auto-start threshold. The shared
// dmix buffer geometry and every non-AA PCM handle remain untouched.

#define LOG_TAG "AUDIO"
#include "../log.h"
#include "audio.h"
#include "common/preload.h"

#include <alsa/asoundlib.h>
#include <cerrno>
#include <cstring>
#include <dlfcn.h>
#include <pthread.h>

namespace {

typedef int (*pcm_open_fn)(snd_pcm_t **, const char *, snd_pcm_stream_t, int);
typedef int (*pcm_close_fn)(snd_pcm_t *);
typedef int (*set_start_threshold_fn)(snd_pcm_t *, snd_pcm_sw_params_t *,
                                      snd_pcm_uframes_t);
typedef int (*get_params_fn)(snd_pcm_t *, snd_pcm_uframes_t *,
                             snd_pcm_uframes_t *);

pcm_open_fn            g_real_open = nullptr;
pcm_close_fn           g_real_close = nullptr;
set_start_threshold_fn g_real_set_start_threshold = nullptr;
get_params_fn          g_real_get_params = nullptr;
pthread_once_t         g_resolve_once = PTHREAD_ONCE_INIT;
bool                   g_enabled = false;

struct TrackedPcm {
    snd_pcm_t *pcm;
    const char *name;
};

TrackedPcm g_tracked[8] = {};
pthread_mutex_t g_tracked_mu = PTHREAD_MUTEX_INITIALIZER;

void resolve_real_functions()
{
    g_real_open = reinterpret_cast<pcm_open_fn>(dlsym(RTLD_NEXT, "snd_pcm_open"));
    g_real_close = reinterpret_cast<pcm_close_fn>(dlsym(RTLD_NEXT, "snd_pcm_close"));
    g_real_set_start_threshold = reinterpret_cast<set_start_threshold_fn>(
        dlsym(RTLD_NEXT, "snd_pcm_sw_params_set_start_threshold"));
    g_real_get_params = reinterpret_cast<get_params_fn>(
        dlsym(RTLD_NEXT, "snd_pcm_get_params"));
}


const char *canonical_aa_name(const char *name)
{
    if (!name) return nullptr;
    if (std::strcmp(name, "androidautoMainAudio") == 0)
        return "androidautoMainAudio";
    if (std::strcmp(name, "androidautoMainAudioVR") == 0)
        return "androidautoMainAudioVR";
    if (std::strcmp(name, "androidautoAltAudio") == 0)
        return "androidautoAltAudio";
    return nullptr;
}

void track_pcm(snd_pcm_t *pcm, const char *name)
{
    pthread_mutex_lock(&g_tracked_mu);
    for (size_t i = 0; i < sizeof(g_tracked) / sizeof(g_tracked[0]); ++i) {
        if (!g_tracked[i].pcm) {
            g_tracked[i].pcm = pcm;
            g_tracked[i].name = name;
            pthread_mutex_unlock(&g_tracked_mu);
            LOGD("tracking playback PCM %s handle=%p", name, (void *)pcm);
            return;
        }
    }
    pthread_mutex_unlock(&g_tracked_mu);
    LOGE("cannot track playback PCM %s handle=%p: table full", name, (void *)pcm);
}

const char *tracked_name(snd_pcm_t *pcm)
{
    const char *name = nullptr;
    pthread_mutex_lock(&g_tracked_mu);
    for (size_t i = 0; i < sizeof(g_tracked) / sizeof(g_tracked[0]); ++i) {
        if (g_tracked[i].pcm == pcm) {
            name = g_tracked[i].name;
            break;
        }
    }
    pthread_mutex_unlock(&g_tracked_mu);
    return name;
}

void untrack_pcm(snd_pcm_t *pcm)
{
    pthread_mutex_lock(&g_tracked_mu);
    for (size_t i = 0; i < sizeof(g_tracked) / sizeof(g_tracked[0]); ++i) {
        if (g_tracked[i].pcm == pcm) {
            g_tracked[i] = TrackedPcm();
            break;
        }
    }
    pthread_mutex_unlock(&g_tracked_mu);
}

} // namespace

namespace aap_service_audio {

void init()
{
    g_enabled = true;
    LOGD("AA playback start threshold override active (one negotiated period)");
}

} // namespace aap_service_audio

extern "C" PRELOAD_EXPORT
int snd_pcm_open(snd_pcm_t **pcm, const char *name,
                 snd_pcm_stream_t stream, int mode)
{
    pthread_once(&g_resolve_once, resolve_real_functions);
    if (!g_real_open) return -ENOSYS;

    int rc = g_real_open(pcm, name, stream, mode);
    const char *aa_name = canonical_aa_name(name);
    if (rc == 0 && g_enabled && pcm && *pcm &&
        stream == SND_PCM_STREAM_PLAYBACK && aa_name)
        track_pcm(*pcm, aa_name);
    return rc;
}

extern "C" PRELOAD_EXPORT
int snd_pcm_sw_params_set_start_threshold(snd_pcm_t *pcm,
                                          snd_pcm_sw_params_t *params,
                                          snd_pcm_uframes_t threshold)
{
    pthread_once(&g_resolve_once, resolve_real_functions);
    if (!g_real_set_start_threshold) return -ENOSYS;

    const char *name = g_enabled ? tracked_name(pcm) : nullptr;
    if (!name || !g_real_get_params)
        return g_real_set_start_threshold(pcm, params, threshold);

    snd_pcm_uframes_t buffer_size = 0;
    snd_pcm_uframes_t period_size = 0;
    int rc = g_real_get_params(pcm, &buffer_size, &period_size);
    if (rc < 0 || period_size == 0) {
        LOGW("%s handle=%p: get_params rc=%d; keeping threshold=%lu",
             name, (void *)pcm, rc, (unsigned long)threshold);
        return g_real_set_start_threshold(pcm, params, threshold);
    }

    const snd_pcm_uframes_t replacement =
        period_size < threshold ? period_size : threshold;
    LOGD("%s handle=%p: buffer=%lu period=%lu start_threshold %lu -> %lu",
         name, (void *)pcm, (unsigned long)buffer_size,
         (unsigned long)period_size, (unsigned long)threshold,
         (unsigned long)replacement);
    return g_real_set_start_threshold(pcm, params, replacement);
}

extern "C" PRELOAD_EXPORT
int snd_pcm_close(snd_pcm_t *pcm)
{
    pthread_once(&g_resolve_once, resolve_real_functions);
    if (!g_real_close) return -ENOSYS;
    if (g_enabled) untrack_pcm(pcm);
    return g_real_close(pcm);
}
