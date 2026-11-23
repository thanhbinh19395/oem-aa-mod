# Handover: VietMap CarPlay Turn-by-Turn (TBT) Navigation on Mazda HUD

**Date:** 2026-11-23  
**Scope:** Root cause analysis, Hopper / REA reverse engineering evidence, and C++ patch implementation for displaying VietMap Live CarPlay Turn-by-Turn (TBT) guidance on the factory Mazda Active Driving Display (CMU150 HUD).  
**Targets:** VietMap Live iOS IPA (~3.3.x), Mazda CMU150 FW **74.00.324A**, repo `oem-aa-mod` (`blmjcicarplay`).  
**Verification Tools:** Hopper Disassembler v5 via REA MCP server (`rea-workflow` / `hopper`), Apple iPhoneOS SDK (`CPManeuver.h`), host test suite (`test/nav_carplay_test.cpp`).

---

## 1. Problem Statement

When driving with **VietMap Live** over Apple CarPlay, navigation maneuvers (arrows, turn directions, roundabout exits, road names) failed to appear on the Mazda factory HUD.

Previously, `oem-aa-mod` supported TBT exclusively on **Apple Maps**, leading to the assumption that third-party CarPlay navigation apps (like VietMap) did not emit iAP2 guidance messages.

---

## 2. Reverse Engineering Findings (VietMap Live iOS App)

Inspection of `tmp/vietmap_ipa_extract/Payload/Runner.app` using **Hopper Disassembler v5** (via REA MCP server) proved that VietMap Live **does** actively initialize and drive Apple CarPlay navigation sessions:

### 2.1 CarPlay Session & Maneuver Call Chain
Binary: `Frameworks/vietmap_live_navigation_plugin.framework/vietmap_live_navigation_plugin` (Mach-O 64-bit arm64, decrypted `cryptid 0`).

```text
Flutter / Dart (App.framework)
    │
    ▼ Pigeon Channel: dev.flutter.pigeon.vietmap_live_navigation_plugin.GeneratedCarApi
    ├── startTrip
    │    └── CarPlayMethodChannel2.startTrip(...)
    │         └── VIETMAPCarPlayManager2.startTripSession(trip:) [0x2e6f8]
    │              └── [mapTemplate startNavigationSessionForTrip:trip] -> navigationSession
    │
    └── setCarManeuver / setCarManeuver2
         └── CarPlayMethodChannel2.setCarManeuverManeuver [0x42138 -> 0x49a78]
              ├── FLTCarPlayCarManeuver_Args.toCPManeuver(cache:) [0x41920]
              ├── FLTCarPlayCarManeuver_Args.toCPNextManeuver(cache:) [0x49b1c]
              └── VIETMAPCarPlayManager2.setManeuver(maneuver:nextManeuver:) [0x3098c]
                   └── [navigationSession setUpcomingManeuvers:upcomingArray]
```

#### Detailed Hopper Disassembly Evidence:
1. **Trip Session Startup (`0x2e6f8`)**:
   Decompilation confirms `VIETMAPCarPlayManager2.startTripSession(trip:)` invokes `-[CPMapTemplate startNavigationSessionForTrip:]`, latches the resulting `CPNavigationSession` into static variable `VIETMAPCarPlayManager2.navigationSession`, and manages trip state.

2. **Maneuver Bridge (`0x49a78`)**:
   In `CarPlayMethodChannel2.setCarManeuverManeuver`:
   ```c
   r0 = __C.FLTCarPlayCarManeuver_Args.toCPManeuver(r20, cache: r21);
   if (r0 != 0x0) {
       r19 = r0;
       r22 = *static VIETMAPCarPlayManager2.instance;
       r23 = __C.FLTCarPlayCarManeuver_Args.toCPNextManeuver(r20, cache: r21);
       VIETMAPCarPlayManager2.setManeuver(r22, maneuver: r19, nextManeuver: r23);
       ...
   }
   ```

3. **Maneuver Array Submission (`0x3098c`)**:
   In `VIETMAPCarPlayManager2.setManeuver`:
   Gathers `[maneuver, nextManeuver]` into a contiguous array and calls:
   ```objc
   r19 = *static VIETMAPCarPlayManager2.navigationSession;
   [r19 setUpcomingManeuvers:r21];
   ```

---

### 2.2 Root Cause 1: Apple `CPManeuver` API Evolution & VietMap Serialization

In Apple's public `CarPlay.framework` (`CPManeuver.h`):
- Prior to **iOS 17.4**, `CPManeuver` had **no public `maneuverType` property or setter**. Third parties could only set:
  - `instructionVariants` (`[NSString]`)
  - `symbolImage` (`UIImage`)
  - `junctionImage` (`UIImage`)
  - `initialTravelEstimates` (`CPTravelEstimates`)
  - `userInfo` (`id`)
- Apple only introduced `@property (nonatomic, assign) CPManeuverType maneuverType API_AVAILABLE(ios(17.4));` in iOS 17.4.
- However, VietMap Live (and standard Flutter CarPlay plugins) targets older iOS versions (iOS 12+) and does not populate `maneuverType`.

#### Hopper Disassembly of `toCPManeuver` (`0x41920`):
Symbol: `(extension in vietmap_live_navigation_plugin):__C.FLTCarPlayCarManeuver_Args.toCPManeuver(cache: [Swift.String : __C.UIImage]) -> __C.CPManeuver?`

```arm64
0x41978  bl  _objc_msgSend$init
0x41994  bl  _objc_msgSend$setInitialTravelEstimates:
0x419f4  bl  _objc_msgSend$setInstructionVariants:
0x41a88  bl  _objc_msgSend$setSymbolImage:
0x41b3c  bl  _objc_msgSend$setCardBackgroundColor:
0x41b68  bl  _objc_msgSend$setUserInfo:
```

VietMap populates instructions (`instructionVariants`) and visual symbol images, but completely omits `maneuverType`.

---

### 2.3 Root Cause 2: iAP2 `0x8059` Packet Serialization on iOS

When iOS serializes `CPManeuver` objects into the Apple Accessory Protocol (iAP2) `0x8059` message sent over USB/Wireless CarPlay to the CMU:
- Offset `+0x24` (`maneuverDesc`): contains the instruction string (e.g. `"Rẽ trái vào Nguyễn Huệ"`).
- Offset `+0x124`: `maneuverDesc` string length (`uint32_t`).
- Offset `+0x128` (`maneuverType`): **defaults to `0` (`CPManeuverTypeNoTurn`)** because the field was left unpopulated by the 3rd-party app.
- Offset `+0x12c`: `afterManeuverRoadName` (often blank in VietMap).
- Offset `+0x33c`: `driveSide` (0 = RHT Vietnam, 1 = LHT).

---

### 2.4 Root Cause 3: Bug in `oem-aa-mod/mazda/patches/blmjcicarplay/nav.cpp`

In `nav.cpp`, the original parser assumed Apple Maps conventions where `mtype` is always populated with standard enum values:
```cpp
// Previous flawed logic:
bool is_context_type(uint32_t m) {
    return m == 0 || m == 5 || m == 11 || m == 18; // 0 was treated as context
}
...
uint32_t mtype = read_u32(b, n, kManeuverTypeOff);
if (idx == 0 || is_context_type(mtype)) return; // Dropped ALL VietMap maneuvers!
```
Because `mtype == 0`, `is_context_type(0)` evaluated to `true`, causing **every maneuver from VietMap Live to be dropped instantly**.

---

### 2.5 Root Cause 4: Guidance Active Maneuver List Missing in VietMap

In Apple Maps, iAP2 guidance message `0x8058` carries the active imminent maneuver index at `+0x458` (`kManListOff`) and list count at `+0x656` (`kManListCntOff`) from TLV `0x000D`.

VietMap Live updates travel estimates via `[navigationSession updateTravelEstimates:forManeuver:]`, which emits `0x8058` distance countdown ticks (`kGuidanceDistOff = 0x34c`), but leaves `maneuverList` count = 0. Without an explicit head pointer, the original code never called `emit_selection()`.

---

## 3. Solution Implementation

The fix was applied in [`mazda/patches/blmjcicarplay/nav.cpp`](../mazda/patches/blmjcicarplay/nav.cpp):

### 3.1 Text Normalization (`to_ascii_lower`)
Converts UTF-8 Vietnamese strings into lowercase ASCII, safely stripping combining marks and diacritics:
- Precomposed Latin (`0x1EA0..0x1EF9`) mapped by block ranges (`A/E/I/O/U/Y`).
- Latin Extended (`ă, â, đ, ê, ô, ơ, ư`) mapped to base ASCII.
- Example: `"Rẽ trái vào Nguyễn Huệ"` → `"re trai vao nguyen hue"`.

### 3.2 Natural Language Maneuver Classification (`classify_from_text`)
When `mtype == 0`, maneuvers are classified by matching keywords in `maneuverDesc`:

| Maneuver Category | Keyword Patterns (Vietnamese & English) | Resulting `Maneuver` Event & Side |
|-------------------|-----------------------------------------|-----------------------------------|
| **U-Turn** | `quay dau`, `vong nguoc lai`, `u turn`, `uturn` | `EV_UTURN` (Side: Left / Right per driveSide) |
| **Roundabout** | `vong xuyen`, `vong xoay`, `bung binh`, `roundabout` | `EV_ROUNDABOUT`, extracts exit number `1..9` |
| **Sharp Turn** | `re gat`, `re ngoat`, `sharp` | `EV_SHARP` (Side: Left if `trai`/`left`, else Right) |
| **Slight Turn** | `chech`, `hoi re`, `slight`, `bear`, `nhap lan` | `EV_SLIGHT` (Side: Left if `trai`/`left`, else Right) |
| **Normal Turn** | `re`, `queo`, `turn`, `sang trai`, `sang phai` | `EV_TURN` (Side: Left if `trai`/`left`, else Right) |
| **Fork / Keep** | `giu ben`, `keep`, `fork` | `EV_FORK` (Side: Left if `trai`/`left`, else Right) |
| **Destination** | `den noi`, `diem den`, `dich den`, `ban da den`, `arrive` | `EV_DEST`, `SIDE_NONE` |
| **Straight** | `di thang`, `chay thang`, `tiep tuc`, `straight`, `continue` | `EV_STRAIGHT`, `SIDE_NONE` |

If the text contains no valid maneuver keywords (e.g. heading text `"Đi về hướng Đông"`), `classify_from_text()` returns `false` and the entry is safely discarded as context.

### 3.3 Bootstrap Selection for 3rd-Party Guidance (`emit_selection(0)`)
To resolve Root Cause 4, when VietMap streams maneuvers but leaves `maneuverList` count = 0:
```cpp
if (mtype == 0) {
    // VietMap / 3rd-party: upcomingManeuvers[0] is always the imminent turn.
    // Guidance active-maneuverList is omitted by iOS, so invalidate any stale Apple Maps
    // head pointer and emit index 0 immediately.
    g_last_head_valid = false;
    g_last_head = 0;
    emit_selection(0);
} else if (g_last_head_valid) {
    int sel = select_by_head(g_last_head);
    if (sel >= 0) emit_selection(sel);
} else if (g_buf_count > 0) {
    // Bootstrap immediately on the first displayable turn
    LOGD("nav MAN bootstrap sel=0 (no guidance head yet, buf=%d)", g_buf_count);
    emit_selection(0);
}
```
And in `0x8058` guidance ticks:
```cpp
} else if (g_buf_count > 0 && (!g_last_head_valid || g_buf[0].mtype == 0)) {
    // Keep painting the buffered turn so the HUD does not stay blank
    g_last_head_valid = false;
    LOGD("nav GUIDE bootstrap sel=0 (maneuverList cnt=0, buf=%d)", g_buf_count);
    emit_selection(0);
}
```
This also prevents cross-app state leakage: if Apple Maps was used earlier in the same drive, its cached `g_last_head_valid` is cleanly cleared upon receiving VietMap maneuvers.

### 3.4 Road Name Fallback to Instruction Text
VietMap frequently does not populate `afterManeuverRoadName` (+0x12c). In `emit_selection()`:
```cpp
const char *road = im.road[0] ? im.road : im.desc;
```
If road name is empty, it falls back to the instruction description (e.g. `"Rẽ trái vào Nguyễn Huệ"`) so the HUD text strip remains informative instead of blank.

### 3.5 Zero Regression for Apple Maps
- When `mtype != 0`: existing enum-based classification `classify(mtype, junction, jAngle, driveSide)` remains unchanged.
- Context filtering `is_context_type(mtype)` preserves skipping of `5` (`FollowRoad`), `11` (`StartRoute`), and `18` (`StartRouteUTurn`).
- Authoritative Apple Maps `maneuverList[0]` head selection immediately overrides bootstrap selection as soon as a valid head is received.

### 3.6 Distance Countdown (`0x8058`)
Distance range check was updated to `dist >= 0 && dist <= 100000` so that `0m` arrival is preserved and properly forwarded to `hud_on_distance()`.

---

## 4. Verification & Automated Testing

A dedicated host unit test was created at [`test/nav_carplay_test.cpp`](../test/nav_carplay_test.cpp) and integrated into [`test/Makefile`](../test/Makefile):

```bash
make -C test nav_carplay_test && test/build/nav_carplay_test
```

### Verified Test Results:
```text
=== Test 1: VietMap Turn Left (mtype=0, text='Rẽ trái vào Nguyễn Huệ') ===
[libpatch-blmjcicarplay][CORE][D] nav MAN idx=0 type=0 vi=0 jt=0 jAng=0 ev=4 side=1 ang=0 num=0 desc="Rẽ trái vào Nguyễn Huệ" road=""
[libpatch-blmjcicarplay][CORE][D] nav MAN bootstrap sel=0 (no guidance head yet, buf=1)
[libpatch-blmjcicarplay][CORE][D] nav PICK sel=0/1 idx=0 type=0 vi=0 ev=4 side=1 ang=0 num=0 road="Rẽ trái vào Nguyễn Huệ"
[libpatch-blmjcicarplay][CORE][D] nav HUD: event=4 side=1 angle=0 number=0 road="Rẽ trái vào Nguyễn Huệ"
  ok:   event is EV_TURN = 4
  ok:   side is SIDE_LEFT = 1
  ok:   road fallback to desc = "Rẽ trái vào Nguyễn Huệ"

=== Test 2: VietMap Turn Right with road name ===
[libpatch-blmjcicarplay][CORE][D] nav MAN idx=0 type=0 vi=0 jt=0 jAng=0 ev=4 side=2 ang=0 num=0 desc="Quẹo phải" road="Đường Lê Lợi"
[libpatch-blmjcicarplay][CORE][D] nav MAN bootstrap sel=0 (no guidance head yet, buf=1)
[libpatch-blmjcicarplay][CORE][D] nav PICK sel=0/1 idx=0 type=0 vi=0 ev=4 side=2 ang=0 num=0 road="Đường Lê Lợi"
[libpatch-blmjcicarplay][CORE][D] nav HUD: event=4 side=2 angle=0 number=0 road="Đường Lê Lợi"
  ok:   event is EV_TURN = 4
  ok:   side is SIDE_RIGHT = 2
  ok:   road name preferred over desc = "Đường Lê Lợi"

=== Test 3: VietMap U-turn ===
[libpatch-blmjcicarplay][CORE][D] nav MAN idx=0 type=0 vi=0 jt=0 jAng=0 ev=6 side=1 ang=0 num=0 desc="Quay đầu tại giao lộ" road=""
[libpatch-blmjcicarplay][CORE][D] nav MAN bootstrap sel=0 (no guidance head yet, buf=1)
[libpatch-blmjcicarplay][CORE][D] nav PICK sel=0/1 idx=0 type=0 vi=0 ev=6 side=1 ang=0 num=0 road="Quay đầu tại giao lộ"
[libpatch-blmjcicarplay][CORE][D] nav HUD: event=6 side=1 angle=0 number=0 road="Quay đầu tại giao lộ"
  ok:   event is EV_UTURN = 6
  ok:   side is SIDE_LEFT (RHT) = 1

=== Test 4: VietMap Roundabout (bùng binh lối ra thứ 2) ===
[libpatch-blmjcicarplay][CORE][D] nav MAN idx=0 type=0 vi=0 jt=0 jAng=0 ev=13 side=2 ang=180 num=2 desc="Tại bùng binh, đi theo lối ra thứ 2" road=""
[libpatch-blmjcicarplay][CORE][D] nav MAN bootstrap sel=0 (no guidance head yet, buf=1)
[libpatch-blmjcicarplay][CORE][D] nav PICK sel=0/1 idx=0 type=0 vi=0 ev=13 side=2 ang=180 num=2 road="Tại bùng binh, đi theo lối ra thứ 2"
[libpatch-blmjcicarplay][CORE][D] nav HUD: event=13 side=2 angle=180 number=2 road="Tại bùng binh, đi theo lối ra thứ 2"
  ok:   event is EV_ROUNDABOUT = 13
  ok:   exit number is 2 = 2
  ok:   angle is 180 = 180

=== Test 5: VietMap Heading/Context ignored (mtype=0, text='Đi về hướng Đông') ===
  ok:   heading context is ignored / not emitted = 0

=== Test 6: Apple Maps regression check (mtype=1 at idx=1) ===
[libpatch-blmjcicarplay][CORE][D] nav MAN idx=1 type=1 vi=0 jt=0 jAng=0 ev=4 side=1 ang=0 num=0 desc="Rẽ trái" road="Võ Văn Kiệt"
[libpatch-blmjcicarplay][CORE][D] nav MAN bootstrap sel=0 (no guidance head yet, buf=1)
[libpatch-blmjcicarplay][CORE][D] nav PICK sel=0/1 idx=1 type=1 vi=0 ev=4 side=1 ang=0 num=0 road="Võ Văn Kiệt"
[libpatch-blmjcicarplay][CORE][D] nav HUD: event=4 side=1 angle=0 number=0 road="Võ Văn Kiệt"
[libpatch-blmjcicarplay][CORE][D] nav PICK sel=0/1 idx=1 type=1 vi=0 ev=4 side=1 ang=0 num=0 road="Võ Văn Kiệt"
  ok:   event is EV_TURN = 4
  ok:   side is SIDE_LEFT = 1
  ok:   road name is Võ Văn Kiệt = "Võ Văn Kiệt"
  ok:   distance is 350m = 350

=== Test 7: VietMap Distance tick in guidance ===
  ok:   guidance distance updated without head = 150

=== Test 8: Switch from Apple Maps to VietMap without reset ===
[libpatch-blmjcicarplay][CORE][D] nav MAN idx=0 type=0 vi=0 jt=0 jAng=0 ev=6 side=1 ang=0 num=0 desc="Quay đầu" road=""
[libpatch-blmjcicarplay][CORE][D] nav PICK sel=0/1 idx=0 type=0 vi=0 ev=6 side=1 ang=0 num=0 road="Quay đầu"
[libpatch-blmjcicarplay][CORE][D] nav HUD: event=6 side=1 angle=0 number=0 road="Quay đầu"
  ok:   event updated to EV_UTURN = 6

ALL TESTS PASSED
```

---

## 5. Related Files & Artifacts

| File | Purpose |
|------|---------|
| [`mazda/patches/blmjcicarplay/nav.cpp`](../mazda/patches/blmjcicarplay/nav.cpp) | iAP2 Maneuver & Guidance decoder with VietMap NLP classifier |
| [`test/nav_carplay_test.cpp`](../test/nav_carplay_test.cpp) | Host self-test for CarPlay / VietMap iAP2 decoder |
| [`knowledge/speed-limit-hud-handover.md`](speed-limit-hud-handover.md) | ESP32 Speed Limit (SPLIM) architecture |
| [`knowledge/vietmap-esp32-hud-connect.md`](vietmap-esp32-hud-connect.md) | BLE GATT `FFF0` protocol specification |
| `tmp/vietmap_ipa_extract/Payload/Runner.app` | Extracted & decrypted VietMap Live iOS application bundle |
