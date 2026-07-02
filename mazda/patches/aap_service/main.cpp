// SPDX-License-Identifier: AGPL-3.0-or-later
//
// libpatch-aap_service.so — LD_PRELOAD shim for the Mazda CMU `aap_service` PID
// (the Android Auto projection daemon). Surfaces Android Auto turn-by-turn
// guidance for the Mazda Active Driving Display (HUD).
//
// Android Auto only sends structured navigation — NavigationState (maneuver +
// lanes) and CurrentPosition (distance + ETA) — when the head unit advertises
// GAL protocol >= 1.6. The OEM advertises 1.5, at which the phone sends only the
// deprecated turn events. With `use_protocol_v1_6 = true` in libpatch.conf this
// shim:
//
//   1. Bumps the advertised GAL version 1.5 -> 1.6 by patching one instruction
//      in Controller::sendVersionRequest (mov r2,#5 -> mov r2,#6).
//   2. Vtable-hooks NavigationStatusEndpoint::routeMessage and swallows the 1.6
//      navigation message range (0x8001..0x8007). The OEM 1.5 endpoint has no
//      parser for those ids and returns STATUS_UNEXPECTED_MESSAGE (-253), which
//      MessageRouter turns into a {00 FF} frame the phone treats as a protocol
//      violation -> Android Auto tears down. Returning 0 ("handled") for the
//      whole range prevents that.
//
// This shim does NOT parse or render anything. aap_service and jciAAPA are
// separate processes; the entire 1.6 decoder + HUD renderer (protobuf decode,
// glyph map, street fold, vbs/svcnavi transport, lanes) lives in the blmjciaapa
// shim. This shim just RELAYS the raw nav frames it swallows — an AaNav16Hdr +
// the original frame bytes — over an abstract AF_UNIX datagram socket to
// blmjciaapa, which decodes and renders them. See common/aa_nav16_msg.h.
//
// With `use_protocol_v1_6 = false` (the default) the shim is inert: the version
// stays 1.5 and the hook is not installed, so the unit behaves stock.
//
// Addresses are for firmware 74.00.324A /usr/bin/aap_service, a non-PIE ELF
// (fixed virtual addresses, no load-base math). Every patch site is byte-verified
// before writing and aborts on mismatch — aap_service runs reset_board="yes", so
// a wrong write must fail safe (no patch).
//
//   Controller::sendVersionRequest               @ 0x00173dfc
//     minor-version insn `mov r2,#5` (e3a02005)  @ 0x00173eb4
//   _ZTV24NavigationStatusEndpoint               @ 0x001f7d10
//     routeMessage vtable slot[7]                @ 0x001f7d2c  (= 0x0017be10)
//   NavigationStatusEndpoint::routeMessage       @ 0x0017be10

#ifndef _GNU_SOURCE
#define _GNU_SOURCE          // dladdr (config.h), abstract sockets; precede includes
#endif

#define LOG_TAG "CORE"
#include "log.h"                    // LIBPATCH_NAME + shared LOG* macros
#include "common/preload_guard.h"   // preload_in_aap_service(), crash handler
#include "common/config.h"          // libpatch_config::{use_protocol_v1_6, load}
#include "common/aa_nav16_msg.h"    // AaNav16Hdr raw-frame envelope + socket name

#include <cstdint>
#include <cstddef>           // offsetof
#include <cstring>           // memset/memcpy/strncpy/strerror
#include <cerrno>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

// ---- pinned addresses (firmware 74.00.324A aap_service, non-PIE) -----------
constexpr uintptr_t kVersionInsnAddr  = 0x00173eb4;  // `mov r2,#5`
constexpr uint32_t  kVersionInsnOld   = 0xe3a02005;  // mov r2,#5 (minor=5)
constexpr uint32_t  kVersionInsnNew   = 0xe3a02006;  // mov r2,#6 (minor=6)

constexpr uintptr_t kVtableSlotAddr   = 0x001f7d2c;  // NavigationStatusEndpoint vtable slot[7]
constexpr uintptr_t kRouteMessageAddr = 0x0017be10;  // NavigationStatusEndpoint::routeMessage

// IoBuffer accessors routeMessage itself uses to read the inbound frame; reused
// to read the protobuf body of the 1.6 frames we decode.
constexpr uintptr_t kSharedPtrDeref = 0x0016f810;  // shared_ptr<IoBuffer>::operator-> -> IoBuffer*
constexpr uintptr_t kIoBufferRaw    = 0x0016f2f8;  // IoBuffer::raw()  -> uint8_t* (msgId at [0:2])
constexpr uintptr_t kIoBufferSize   = 0x0016f330;  // IoBuffer::size() -> int (total, incl 2-byte id)

// routeMessage(unsigned char chan, unsigned short msgId,
// const shared_ptr<IoBuffer>&) -> int. AAPCS: r0=this, r1=chan, r2=msgId,
// r3=&shared_ptr; narrow args arrive widened to 32 bits.
typedef int      (*route_fn)(void *self, uint32_t chan, uint32_t msgId, const void *iobuf_ref);
typedef void *   (*sp_deref_fn)(const void *sp);   // shared_ptr<IoBuffer>::operator->
typedef uint8_t *(*iob_raw_fn)(void *iob);         // IoBuffer::raw()
typedef int      (*iob_size_fn)(void *iob);        // IoBuffer::size()

// ---- page protection helper ----
bool set_prot(uintptr_t addr, size_t len, int prot)
{
    const uintptr_t ps   = (uintptr_t)sysconf(_SC_PAGESIZE);
    const uintptr_t page = addr & ~(ps - 1);
    const size_t    span = (addr + len) - page;
    return mprotect(reinterpret_cast<void *>(page), span, prot) == 0;
}

// Read an inbound frame's raw protobuf via the IoBuffer accessors routeMessage
// uses. Returns the buffer pointer (and its size via *size_out), or nullptr on
// any failure. Read-only.
const uint8_t *frame_bytes(const void *iobuf_ref, int *size_out)
{
    *size_out = 0;
    if (!iobuf_ref) return nullptr;
    void *iob = reinterpret_cast<sp_deref_fn>(kSharedPtrDeref)(iobuf_ref);
    if (!iob) return nullptr;
    const uint8_t *raw = reinterpret_cast<iob_raw_fn>(kIoBufferRaw)(iob);
    int size = reinterpret_cast<iob_size_fn>(kIoBufferSize)(iob);
    if (!raw || size <= 0) return nullptr;
    *size_out = size;
    return raw;
}

// ---- raw relay to the blmjciaapa HUD renderer ------------------------------
// We do NOT parse the AA frames here — blmjciaapa decodes + renders them. This
// shim just forwards the swallowed nav frames (an AaNav16Hdr + the raw frame
// bytes) over an abstract AF_UNIX datagram socket. Fail-safe: any error
// (including "receiver not bound yet") just drops the datagram; the next lands.
int g_tx_fd = -1;

void nav16_send_raw(const void *buf, size_t len)
{
    if (g_tx_fd < 0) {
        g_tx_fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (g_tx_fd < 0) { LOGE("nav16: socket() failed: %s", strerror(errno)); return; }
    }
    struct sockaddr_un addr;
    socklen_t addrlen;
    aa_nav16_fill_addr(addr, addrlen);

    ssize_t n = sendto(g_tx_fd, buf, len, MSG_DONTWAIT,
                       reinterpret_cast<struct sockaddr *>(&addr), addrlen);
    if (n < 0) {
        // ECONNREFUSED / ENOENT just means blmjciaapa's receiver isn't bound
        // yet — normal during bring-up; the next frame will land.
        LOGV("nav16: sendto dropped: %s", strerror(errno));
    }
}

// Wrap one inbound AA nav frame in an AaNav16Hdr and relay it, raw. Read-only on
// the IoBuffer; caps the payload at AA_NAV16_MAX_FRAME.
void relay_frame(const void *iobuf_ref)
{
    int size = 0;
    const uint8_t *raw = frame_bytes(iobuf_ref, &size);
    if (!raw || size < 2) {
        LOGV("nav16: relay skipped — no/short frame (size=%d)", size);
        return;
    }
    if (size > AA_NAV16_MAX_FRAME) size = AA_NAV16_MAX_FRAME;

    char buf[sizeof(AaNav16Hdr) + AA_NAV16_MAX_FRAME];
    AaNav16Hdr hdr;
    hdr.magic    = AA_NAV16_MAGIC;
    hdr.version  = AA_NAV16_VERSION;
    hdr.reserved = 0;
    hdr.len      = (uint16_t)size;
    std::memcpy(buf, &hdr, sizeof(hdr));
    std::memcpy(buf + sizeof(hdr), raw, (size_t)size);
    nav16_send_raw(buf, sizeof(hdr) + (size_t)size);
}

// Installed replacement for NavigationStatusEndpoint::routeMessage (we swapped
// the vtable slot; the real body is untouched, so calling it stays safe and
// non-recursive).
//
// The OEM endpoint returns -253 for any msgId outside {0x8003,0x8004,0x8005},
// which MessageRouter turns into a {00 FF} frame -> AA teardown. So we SWALLOW
// the whole GAL 1.6 nav range (0x8001..0x8007) by returning 0 ("handled"). This
// stays a dumb pipe: every swallowed frame is relayed RAW and unfiltered, so the
// receiver can start consuming more ids without ever re-touching this brick-PID
// shim. (The receiver currently acts on 0x8002/0x8003/0x8006/0x8007.) Messages
// outside the range are not ours — pass them to the real OEM routeMessage.
int hook_routeMessage(void *self, uint32_t chan, uint32_t msgId, const void *iobuf_ref)
{
    const uint32_t id = msgId & 0xffff;
    if (id >= AA_NAV16_MSG_SWALLOW_FIRST && id <= AA_NAV16_MSG_SWALLOW_LAST) {
        LOGV("nav16: swallow msgId=0x%04x chan=%u -> relay", id, chan);
        relay_frame(iobuf_ref);                     // catch -> forward raw -> return 0
        return 0;                                   // routeMessage's "handled OK" value
    }
    return reinterpret_cast<route_fn>(kRouteMessageAddr)(self, chan, msgId, iobuf_ref);
}

// Patch the advertised GAL version 1.5 -> 1.6 (mov r2,#5 -> #6). Byte-verified.
void bump_version()
{
    volatile uint32_t *p = reinterpret_cast<volatile uint32_t *>(kVersionInsnAddr);
    if (*p != kVersionInsnOld) {
        LOGE("version: ABORT — 0x%08x at 0x%08lx, expected 0x%08x (binary differs; not patching)",
             *p, (unsigned long)kVersionInsnAddr, kVersionInsnOld);
        return;
    }
    if (!set_prot(kVersionInsnAddr, 4, PROT_READ | PROT_WRITE)) {
        LOGE("version: ABORT — mprotect RW failed");
        return;
    }
    *p = kVersionInsnNew;
    set_prot(kVersionInsnAddr, 4, PROT_READ | PROT_EXEC);
    __builtin___clear_cache(reinterpret_cast<char *>(kVersionInsnAddr),
                            reinterpret_cast<char *>(kVersionInsnAddr + 4));
    LOGD("version: advertising GAL 1.6 (mov r2,#5 -> #6 @0x%08lx)",
         (unsigned long)kVersionInsnAddr);
}

// Install the nav routeMessage hook by swapping the vtable slot. Byte-verified.
// Returns true only if the hook was actually installed; false on byte-verify
// mismatch or mprotect failure (caller must NOT bump the advertised version on
// false — see aap_service_patch_init).
bool install_nav_hook()
{
    volatile uintptr_t *slot = reinterpret_cast<volatile uintptr_t *>(kVtableSlotAddr);
    if (*slot != kRouteMessageAddr) {
        LOGE("hook: ABORT — vtable slot7 @0x%08lx = 0x%08lx, expected 0x%08lx "
             "(binary differs; not hooking)",
             (unsigned long)kVtableSlotAddr, (unsigned long)*slot,
             (unsigned long)kRouteMessageAddr);
        return false;
    }
    if (!set_prot(kVtableSlotAddr, sizeof(uintptr_t), PROT_READ | PROT_WRITE)) {
        LOGE("hook: ABORT — mprotect RW failed on vtable");
        return false;
    }
    *slot = reinterpret_cast<uintptr_t>(&hook_routeMessage);
    LOGD("hook: nav routeMessage hook installed (slot7 @0x%08lx)",
         (unsigned long)kVtableSlotAddr);
    // Restore the vtable page to read-only (W^X hardening: it lives in
    // .data.rel.ro and only needed to be writable for the swap above).
    if (!set_prot(kVtableSlotAddr, sizeof(uintptr_t), PROT_READ))
        LOGW("hook: mprotect RO restore failed (hook IS installed; continuing)");
    return true;
}

__attribute__((constructor))
void aap_service_patch_init()
{
    // This LD_PRELOAD is inherited by every process aap_service forks/execs; our
    // patches dereference fixed aap_service addresses, so stay inert unless
    // argv[0] is aap_service itself.
    if (!preload_in_aap_service())
        return;

    libpatch_config::load(reinterpret_cast<const void *>(&aap_service_patch_init));

    // The whole 1.6 path is opt-in on this one key. When hud is off nothing
    // renders the relayed frames, but the hook still swallows the nav range so
    // the AA session stays healthy — so gating here on use_protocol_v1_6 alone
    // is safe.
    if (!libpatch_config::use_protocol_v1_6()) {
        LOGD("use_protocol_v1_6=false -> GAL 1.5 (stock); shim inert");
        return;
    }

    LOGD("GAL 1.6 nav path active (pid=%d)", (int)getpid());
    preload_install_fatal_handler();   // dump regs/maps on crash, then re-raise

    // Transactional hook-first: install the vtable hook BEFORE advertising 1.6.
    // If install_nav_hook() fails (binary mismatch or mprotect failure) the OEM
    // 1.5 endpoint stays live; advertising 1.6 in that split-brain state would
    // cause the phone to send 1.6 nav messages that the OEM endpoint returns -253
    // for -> MessageRouter emits {00 FF} -> Android Auto tears down. By only
    // bumping the version when the hook is confirmed installed, both operations
    // are kept consistent: either the hook is live and we advertise 1.6, or
    // neither fires and the unit speaks stock GAL 1.5.
    if (install_nav_hook()) {
        bump_version();
    } else {
        LOGE("GAL 1.6 path DISABLED — hook failed; staying stock GAL 1.5");
    }
}

} // namespace
