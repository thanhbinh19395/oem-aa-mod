# Cross-compile Makefile for standalone on-device test tools.
#
# Builds small ARM (Cortex-A9 NEON, glibc) executables that run directly
# on the Mazda CMU. Unlike mazda/ (LD_PRELOAD shared objects) these are
# plain programs you scp to the unit and run from a root shell. They link
# no OEM library at build time — each tool dlopen()s what it needs at
# runtime — so the build has no JCI-sysroot dependency beyond libc.
#
# Reuses the m3-toolchain submodule that lives under ../mazda.
#
# Targets:
#   make            -> build all tools into build/
#   make gsi_test   -> build just that tool
#   make clean      -> wipe build/
#
# Deploy (example):
#   scp build/gsi_test root@<cmu>:/tmp/ && ssh root@<cmu> /tmp/gsi_test 2

M3TOOLCHAIN := ../mazda/m3-toolchain
TRIPLE      := arm-cortexa9_neon-linux-gnueabi
CC          := $(M3TOOLCHAIN)/bin/$(TRIPLE)-gcc
CXX         := $(M3TOOLCHAIN)/bin/$(TRIPLE)-g++
SYSROOT     := $(M3TOOLCHAIN)/$(TRIPLE)/sysroot

CFLAGS  := --sysroot=$(SYSROOT) \
           -march=armv7-a -mtune=cortex-a9 -mfpu=neon \
           -O2 -std=gnu11 -Wall -Wextra
# -ldl: dlopen the OEM client lib.  -lpthread: condvar wait.
# -lrt: clock_gettime on the device's older glibc.
LDFLAGS := --sysroot=$(SYSROOT) -ldl -lpthread -lrt

# --- dbus-c++ tools (lane_test) --------------------------------
# lane_test talks to com.jci.vbs.navi through the dbus-c++ generated
# proxies (reference/dbus/generated_cmu.h, a local-only asset). The
# device ships libdbus-c++-1 + libdbus-1 (NOT the glib variant), so we
# use dbus-c++'s core dispatcher and link only those. -static-libstdc++
# because the device's libstdc++ predates the toolchain's.
DBUSXX_INC := -I$(SYSROOT)/usr/include/dbus-c++-1 \
              -I$(SYSROOT)/usr/include/dbus-1.0 \
              -I$(SYSROOT)/usr/lib/dbus-1.0/include \
              -I../reference/dbus
CXXFLAGS := --sysroot=$(SYSROOT) \
            -march=armv7-a -mtune=cortex-a9 -mfpu=neon \
            -O2 -std=gnu++11 -Wall -Wno-unused-parameter -Wno-deprecated \
            $(DBUSXX_INC)
# Static-link libdbus-c++ (its vtable symbols don't resolve cleanly from
# the shared lib, and static avoids a runtime dep on the device).
# libdbus-1 stays dynamic — it's present on the unit. -static-libstdc++
# because the device's libstdc++ predates the toolchain's.
DBUSXX_LDFLAGS := --sysroot=$(SYSROOT) -static-libstdc++ \
                 -Wl,-Bstatic -ldbus-c++-1 -Wl,-Bdynamic \
                 -ldbus-1 -lpthread -lrt

BUILD := build
TOOLS := gsi_test lane_test
BINS  := $(addprefix $(BUILD)/,$(TOOLS))

.PHONY: all clean $(TOOLS)
all: $(BINS)

# C tools: a single <name>.c -> build/<name>.
$(BUILD)/%: %.c | $(BUILD)
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

# lane_test is C++ (dbus-c++). Explicit rule overrides the .c pattern.
$(BUILD)/lane_test: lane_test.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $< -o $@ $(DBUSXX_LDFLAGS)

# Convenience: `make gsi_test` builds build/gsi_test.
$(TOOLS): %: $(BUILD)/%

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
