// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Host self-test for CarPlay / VietMap iAP2 nav decoder (nav.cpp).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

std::FILE *g_logf = nullptr;

static int g_fail = 0;

static uint32_t g_last_status = 0;
static char     g_last_turn_road[256] = {0};
static uint32_t g_last_turn_side = 0;
static uint32_t g_last_turn_event = 0;
static int32_t  g_last_turn_angle = 0;
static int32_t  g_last_turn_number = 0;
static int32_t  g_last_dist_m = -1;
static uint32_t g_last_dist_unit = 0;

void hud_on_status(uint32_t status)
{
    g_last_status = status;
}

void hud_on_next_turn(const char *road_name,
                      uint32_t    turn_side,
                      uint32_t    turn_event,
                      int32_t     turn_angle,
                      int32_t     turn_number)
{
    if (road_name) {
        strncpy(g_last_turn_road, road_name, sizeof(g_last_turn_road) - 1);
        g_last_turn_road[sizeof(g_last_turn_road) - 1] = '\0';
    } else {
        g_last_turn_road[0] = '\0';
    }
    g_last_turn_side = turn_side;
    g_last_turn_event = turn_event;
    g_last_turn_angle = turn_angle;
    g_last_turn_number = turn_number;
}

void hud_on_distance(int32_t  distance_meters,
                     int32_t  /*time_until_seconds*/,
                     int32_t  /*display_distance*/,
                     uint32_t display_distance_unit)
{
    g_last_dist_m = distance_meters;
    g_last_dist_unit = display_distance_unit;
}

// Declarations from nav.cpp
extern "C" void nav_on_devmgr_msg(const void *msgp, long n);
extern "C" void nav_request_reset(void);

// Helpers to build 0x8059 and 0x8058 packets
static std::vector<uint8_t> make_maneuver(uint32_t idx, uint32_t mtype,
                                          const char *text, const char *road,
                                          uint32_t junction = 0, int16_t jAngle = 0,
                                          uint32_t driveSide = 0)
{
    std::vector<uint8_t> b(840, 0);
    *reinterpret_cast<uint32_t *>(b.data()) = 0x8059;
    *reinterpret_cast<uint32_t *>(b.data() + 0x20) = (idx << 16);
    *reinterpret_cast<uint32_t *>(b.data() + 0x128) = mtype;
    *reinterpret_cast<uint32_t *>(b.data() + 0x33c) = driveSide;
    *reinterpret_cast<uint32_t *>(b.data() + 0x340) = junction;
    *reinterpret_cast<int16_t *>(b.data() + 0x346) = jAngle;
    if (text) strncpy(reinterpret_cast<char *>(b.data() + 0x24), text, 255);
    if (road) strncpy(reinterpret_cast<char *>(b.data() + 0x12c), road, 255);
    return b;
}

static std::vector<uint8_t> make_guidance(int32_t dist_m, uint16_t head = 0, uint16_t count = 0)
{
    std::vector<uint8_t> b(1628, 0);
    *reinterpret_cast<uint32_t *>(b.data()) = 0x8058;
    *reinterpret_cast<uint32_t *>(b.data() + 0x34c) = (uint32_t)dist_m;
    *reinterpret_cast<uint16_t *>(b.data() + 0x458) = head;
    *reinterpret_cast<uint16_t *>(b.data() + 0x656) = count;
    return b;
}

#define CHECK(cond, msg) do {                                            \
        if (!(cond)) { printf("  FAIL: %s\n", (msg)); g_fail = 1; }      \
        else         { printf("  ok:   %s\n", (msg)); }                  \
    } while (0)

#define CHECK_EQ_U(got, exp, msg) do {                                   \
        unsigned long _g = (unsigned long)(got), _e = (unsigned long)(exp); \
        if (_g != _e) { printf("  FAIL: %s (got %lu, want %lu)\n",       \
                               (msg), _g, _e); g_fail = 1; }             \
        else          { printf("  ok:   %s = %lu\n", (msg), _g); }       \
    } while (0)

#define CHECK_EQ_S(got, exp, msg) do {                                   \
        if (strcmp((got), (exp)) != 0) {                                 \
            printf("  FAIL: %s (got \"%s\", want \"%s\")\n",             \
                   (msg), (got), (exp)); g_fail = 1; }                   \
        else { printf("  ok:   %s = \"%s\"\n", (msg), (got)); }          \
    } while (0)

int main()
{
    nav_request_reset();

    printf("=== Test 1: VietMap Turn Left (mtype=0, text='Rẽ trái vào Nguyễn Huệ') ===\n");
    {
        auto m0 = make_maneuver(0, 0, "Rẽ trái vào Nguyễn Huệ", "");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 4, "event is EV_TURN");
        CHECK_EQ_U(g_last_turn_side, 1, "side is SIDE_LEFT");
        CHECK_EQ_S(g_last_turn_road, "Rẽ trái vào Nguyễn Huệ", "road fallback to desc");
    }

    printf("\n=== Test 2: VietMap Turn Right with road name ===\n");
    {
        nav_request_reset();
        auto m0 = make_maneuver(0, 0, "Quẹo phải", "Đường Lê Lợi");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 4, "event is EV_TURN");
        CHECK_EQ_U(g_last_turn_side, 2, "side is SIDE_RIGHT");
        CHECK_EQ_S(g_last_turn_road, "Đường Lê Lợi", "road name preferred over desc");
    }

    printf("\n=== Test 3: VietMap U-turn ===\n");
    {
        nav_request_reset();
        auto m0 = make_maneuver(0, 0, "Quay đầu tại giao lộ", "");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 6, "event is EV_UTURN");
        CHECK_EQ_U(g_last_turn_side, 1, "side is SIDE_LEFT (RHT)");
    }

    printf("\n=== Test 4: VietMap Roundabout (bùng binh lối ra thứ 2) ===\n");
    {
        nav_request_reset();
        auto m0 = make_maneuver(0, 0, "Tại bùng binh, đi theo lối ra thứ 2", "");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 13, "event is EV_ROUNDABOUT");
        CHECK_EQ_U(g_last_turn_number, 2, "exit number is 2");
        CHECK_EQ_U(g_last_turn_angle, 0, "angle is 0 (straight ahead)");

        // Test exit 1 (right = 90 deg)
        nav_request_reset();
        auto m1 = make_maneuver(0, 0, "Tại bùng binh, đi theo lối ra thứ 1", "");
        nav_on_devmgr_msg(m1.data(), m1.size());
        CHECK_EQ_U(g_last_turn_number, 1, "exit number is 1");
        CHECK_EQ_U(g_last_turn_angle, 90, "angle is 90 (right)");

        // Test exit 3 (left = 270 deg)
        nav_request_reset();
        auto m3 = make_maneuver(0, 0, "Tại vòng xuyến, đi theo lối ra thứ 3", "");
        nav_on_devmgr_msg(m3.data(), m3.size());
        CHECK_EQ_U(g_last_turn_number, 3, "exit number is 3");
        CHECK_EQ_U(g_last_turn_angle, 270, "angle is 270 (left)");
    }

    printf("\n=== Test 5: VietMap Departure / Straight Cruise (mtype=0, text='Đi về hướng Đông') ===\n");
    {
        nav_request_reset();
        auto m0 = make_maneuver(0, 0, "Đi về hướng Đông", "");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 14, "event defaults to EV_STRAIGHT (never dropped)");
        CHECK_EQ_U(g_last_turn_side, 3, "side is SIDE_NONE");
        CHECK_EQ_S(g_last_turn_road, "Đi về hướng Đông", "road shows departure heading");
    }

    printf("\n=== Test 6: Apple Maps regression check (mtype=1 at idx=1) ===\n");
    {
        nav_request_reset();
        auto m0 = make_maneuver(0, 11, "Start", ""); // idx 0 start route
        auto m1 = make_maneuver(1, 1, "Rẽ trái", "Võ Văn Kiệt"); // idx 1 turn left
        nav_on_devmgr_msg(m0.data(), m0.size());
        nav_on_devmgr_msg(m1.data(), m1.size());
        // Guidance with head=1
        auto g = make_guidance(350, 1, 1);
        nav_on_devmgr_msg(g.data(), g.size());
        CHECK_EQ_U(g_last_turn_event, 4, "event is EV_TURN");
        CHECK_EQ_U(g_last_turn_side, 1, "side is SIDE_LEFT");
        CHECK_EQ_S(g_last_turn_road, "Võ Văn Kiệt", "road name is Võ Văn Kiệt");
        CHECK_EQ_U(g_last_dist_m, 350, "distance is 350m");
    }

    printf("\n=== Test 7: VietMap Distance tick in guidance ===\n");
    {
        auto g = make_guidance(150, 0, 0); // count=0
        nav_on_devmgr_msg(g.data(), g.size());
        CHECK_EQ_U(g_last_dist_m, 150, "guidance distance updated without head");
    }

    printf("\n=== Test 8: Switch from Apple Maps to VietMap without reset ===\n");
    {
        // NO nav_request_reset()! g_last_head is still 1 from Test 6
        auto m0 = make_maneuver(0, 0, "Quay đầu", "");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 6, "event updated to EV_UTURN");
    }

    printf("\n=== Test 9: iOS NFD (Decomposed UTF-8) Turn Left: 'R\\x65\\xcc\\x83 tr\\x61\\xcc\\x81i' ===\n");
    {
        nav_request_reset();
        // "Rẽ trái" in NFD decomposed form (e + combining tilde, a + combining acute)
        const char nfd_turn_left[] = "R\x65\xcc\x83 tr\x61\xcc\x81i";
        auto m0 = make_maneuver(0, 0, nfd_turn_left, "Đường NFD");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 4, "NFD classified as EV_TURN");
        CHECK_EQ_U(g_last_turn_side, 1, "NFD classified as SIDE_LEFT");
    }

    printf("\n=== Test 10: iOS NFD Turn Right: 'R\\x65\\xcc\\x83 ph\\x61\\xcc\\x89i' ===\n");
    {
        nav_request_reset();
        // "Rẽ phải" in NFD decomposed form (e + combining tilde, a + combining hook above)
        const char nfd_turn_right[] = "R\x65\xcc\x83 ph\x61\xcc\x89i";
        auto m0 = make_maneuver(0, 0, nfd_turn_right, "");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 4, "NFD classified as EV_TURN");
        CHECK_EQ_U(g_last_turn_side, 2, "NFD classified as SIDE_RIGHT");
    }

    printf("\n=== Test 11: VietMap 'Đi tiếp' (continue) straight ===\n");
    {
        nav_request_reset();
        auto m0 = make_maneuver(0, 0, "Đi tiếp 2.5 km trên Quốc lộ 1A", "");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 14, "event is EV_STRAIGHT");
        CHECK_EQ_U(g_last_turn_side, 3, "side is SIDE_NONE");
    }

    printf("\n=== Test 12: VietMap 'Nhập vào đường' (merge) ===\n");
    {
        nav_request_reset();
        auto m0 = make_maneuver(0, 0, "Nhập vào đường cao tốc bên phải", "");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 3, "event is EV_SLIGHT");
        CHECK_EQ_U(g_last_turn_side, 2, "side is SIDE_RIGHT");
    }

    printf("\n=== Test 13: VietMap Turn Right onto 'Đường Nguyễn Trãi' (no left false positive) ===\n");
    {
        nav_request_reset();
        auto m0 = make_maneuver(0, 0, "Rẽ phải vào đường Nguyễn Trãi", "");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 4, "event is EV_TURN");
        CHECK_EQ_U(g_last_turn_side, 2, "side is SIDE_RIGHT (not flipped by 'Nguyễn Trãi')");
    }

    printf("\n=== Test 14: VietMap Sharp/Slight/Fork Right onto 'Nguyễn Trãi' ===\n");
    {
        nav_request_reset();
        auto m0 = make_maneuver(0, 0, "Rẽ gắt phải vào Nguyễn Trãi", "");
        nav_on_devmgr_msg(m0.data(), m0.size());
        CHECK_EQ_U(g_last_turn_event, 5, "event is EV_SHARP");
        CHECK_EQ_U(g_last_turn_side, 2, "sharp turn side is SIDE_RIGHT");

        nav_request_reset();
        auto m1 = make_maneuver(0, 0, "Chếch phải vào đường Nguyễn Trãi", "");
        nav_on_devmgr_msg(m1.data(), m1.size());
        CHECK_EQ_U(g_last_turn_event, 3, "event is EV_SLIGHT");
        CHECK_EQ_U(g_last_turn_side, 2, "slight turn side is SIDE_RIGHT");

        nav_request_reset();
        auto m2 = make_maneuver(0, 0, "Giữ bên phải hướng Nguyễn Trãi", "");
        nav_on_devmgr_msg(m2.data(), m2.size());
        CHECK_EQ_U(g_last_turn_event, 9, "event is EV_FORK");
        CHECK_EQ_U(g_last_turn_side, 2, "fork side is SIDE_RIGHT");
    }

    printf("\n=== Test 15: Long expressway distance (>100 km) ===\n");
    {
        auto g = make_guidance(150000, 0, 0);
        nav_on_devmgr_msg(g.data(), g.size());
        CHECK_EQ_U(g_last_dist_m, 150000, "150 km expressway distance accepted");
        CHECK_EQ_U(g_last_dist_unit, 3, "unit is KM (3)");
    }

    printf("\n=== Test 16: Stray mtype=0 inside Apple Maps burst does not corrupt head ===\n");
    {
        nav_request_reset();
        auto m0 = make_maneuver(0, 11, "Start", "");
        auto m1 = make_maneuver(1, 1, "Rẽ trái", "Võ Văn Kiệt");
        auto m2 = make_maneuver(2, 2, "Rẽ phải", "Nguyễn Văn Cừ");
        auto g1 = make_guidance(400, 2, 1);  // head=2 (m2)
        nav_on_devmgr_msg(m0.data(), m0.size());
        nav_on_devmgr_msg(m1.data(), m1.size());
        nav_on_devmgr_msg(m2.data(), m2.size());
        nav_on_devmgr_msg(g1.data(), g1.size());
        CHECK_EQ_U(g_last_turn_side, 2, "initial Apple head=2 is SIDE_RIGHT");
        CHECK_EQ_S(g_last_turn_road, "Nguyễn Văn Cừ", "initial Apple head=2 road");

        // Stray mtype=0 entry at idx=3 inside same burst must NOT reset g_last_head_valid or flash sel 0
        auto m3 = make_maneuver(3, 0, "Đi tiếp", "Hàm Nghi");
        nav_on_devmgr_msg(m3.data(), m3.size());
        CHECK_EQ_U(g_last_turn_side, 2, "Apple head=2 preserved (still SIDE_RIGHT)");
        CHECK_EQ_S(g_last_turn_road, "Nguyễn Văn Cừ", "Apple head=2 road preserved");
    }

    if (g_fail) {
        printf("\nTEST SUITE FAILED\n");
        return 1;
    }
    printf("\nALL TESTS PASSED\n");
    return 0;
}
