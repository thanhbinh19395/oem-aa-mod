// Self-resolving thunks for the OEM functions inside blmjciaapa.so.
// See blmjciaapa.h for the rationale and the anchor-and-offset model.
//
// Each public function is a thin trampoline that, on first call,
// computes its target address (load bias of blmjciaapa.so + the
// target's file offset), caches it, and forwards. The load bias comes
// from the one dynamically-exported anchor symbol, GetServiceInterfaces,
// via dlsym. No init step, no shared global state, no ordering rule.
// An unresolved target (blmjciaapa.so not mapped — i.e. not the
// {L_jciAAPA} PID) degrades to a benign failure value.

#define LOG_TAG "BLMJCIAAPA"
#include "../log.h"

#include "blmjciaapa.h"

#include <dlfcn.h>

namespace {

// File offsets within blmjciaapa.so (FW 74.00.324A NA), harvested
// with the cross-toolchain nm. Re-harvest after any OEM update.
namespace off {
//   GetServiceInterfaces (the only dynamically-exported symbol; anchor)
constexpr uintptr_t GetServiceInterfaces          = 0x00061768;
//   _ZN9SingletonI7AapProcE11GetInstanceEv
constexpr uintptr_t Singleton_AapProc_GetInstance = 0x00062670;
//   _ZN7AapProc15GetVideoManagerEv
constexpr uintptr_t AapProc_GetVideoManager       = 0x0007f664;
//   _ZN7AapProc10GetRaceAapEv
constexpr uintptr_t AapProc_GetRaceAap            = 0x0007f614;
//   _ZN12VideoManager16IsAAVideoInFocusEv
constexpr uintptr_t VideoManager_IsAAVideoInFocus = 0x000b3af4;
//   _ZN7RaceAap14SendTouchInputEP14AAP_TouchEvent
constexpr uintptr_t RaceAap_SendTouchInput        = 0x0008e3a8;
} // namespace off

// Load bias of the already-mapped blmjciaapa.so: address of the
// exported anchor minus its file offset. 0 until resolved (or if
// blmjciaapa.so isn't mapped). Retries until it succeeds — the BLM's
// AapProc singleton may not be up on the earliest calls, but the
// library itself is mapped for the whole {L_jciAAPA} PID lifetime, so
// once non-zero the bias never changes.
uintptr_t blm_base()
{
	static uintptr_t base = 0;
	if (base == 0) {
		void *handle = dlopen("/jci/aapa/blmjciaapa.so",
		                      RTLD_NOW | RTLD_NOLOAD);
		void *sym = handle ? dlsym(handle, "GetServiceInterfaces")
		                   : nullptr;
		if (sym) {
			base = reinterpret_cast<uintptr_t>(sym)
			       - off::GetServiceInterfaces;
		} else {
			// Log once — repeated misses (e.g. we're not the
			// {L_jciAAPA} PID, so blmjciaapa.so is never mapped)
			// would otherwise spam the per-frame touch path.
			static bool logged = false;
			if (!logged) {
				logged = true;
				LOGC("oem: blmjciaapa.so anchor GetServiceInterfaces "
				     "unresolved (not the L_jciAAPA PID?)");
			}
		}
	}
	return base;
}

// Resolve one offset-based target to an absolute address, or nullptr
// if the load bias isn't available yet.
void *blm_addr(uintptr_t target_offset)
{
	uintptr_t base = blm_base();
	return base ? reinterpret_cast<void *>(base + target_offset) : nullptr;
}

} // namespace

// Define a self-resolving thunk that forwards to the OEM target at
// off::<name>. `params` / `args` are parenthesised lists.
#define BLM_THUNK(ret, name, params, args, failval) \
	ret name params \
	{ \
		using Fn = ret (*) params; \
		static Fn fn = nullptr; \
		if (!fn) fn = reinterpret_cast<Fn>(blm_addr(off::name)); \
		return fn ? fn args : (failval); \
	}

BLM_THUNK(void *, Singleton_AapProc_GetInstance, (void), (), nullptr)
BLM_THUNK(void *, AapProc_GetVideoManager, (void *self), (self), nullptr)
BLM_THUNK(void *, AapProc_GetRaceAap, (void *self), (self), nullptr)
BLM_THUNK(int,    VideoManager_IsAAVideoInFocus, (void *self), (self), 0)
BLM_THUNK(int,    RaceAap_SendTouchInput,
          (void *self, AAP_TouchEvent *evt), (self, evt), -1)
