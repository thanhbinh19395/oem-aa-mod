// SPDX-License-Identifier: AGPL-3.0-or-later
//
// splim_seriald — USB serial → /tmp/splim for Mazda Connect (CMU150).
// Copyright (C) 2026 KID MIXER-MODER.
//
// Part of an AGPL-3.0 work — see NOTICE.md for full attribution.
//
// Reads line-oriented "SPLIM <km/h>" from an ESP32 USB CDC port (ttyACM*/ttyUSB*)
// and atomically writes "/tmp/splim" (RAM tmpfs, zero eMMC flash wear) as
// "<limit> <unix_ts>" — the format libpatch-blmjcicarplay::read_splim() consumes.
//
// EXCLUSIVITY: a lock file under /tmp keeps a second instance from opening the
// same writer path. Prefer a single writer of splim (do not run OCR / a competing
// UDP writer at the same time).

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/sysinfo.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

// Stored in /tmp (tmpfs RAM) instead of /data_persist (eMMC flash) so the
// periodic 3 s timestamp refresh incurs ZERO flash wear and auto-cleans on boot.
#define SPLIM_PATH     "/tmp/splim"
#define SPLIM_TMP      "/tmp/splim.tmp"
#define LOCK_PATH      "/tmp/splim_seriald.lock"
#define DEFAULT_BAUD   B115200
#define SPLIM_LINE_MAX 128   // not LINE_MAX: that name collides with <limits.h>
#define LOG_TAG        "[SPLIM-SERIAL] "
// The HUD treats splim as stale after 8 s and the ESP32 re-sends every 2 s, so an
// unchanged limit only needs a fresh timestamp this often.
#define REFRESH_SEC    3

static volatile sig_atomic_t g_stop = 0;
static int g_lock_fd = -1;

static void on_signal(int sig) { (void)sig; g_stop = 1; }

// Monotonic seconds via Linux sysinfo() (in libc on glibc 2.11+, no -lrt needed).
// Immune to RTC / GPS / NTP wall-clock jumps.
static long monotonic_sec(void)
{
    struct sysinfo si;
    if (sysinfo(&si) == 0)
        return si.uptime;
    return (long)time(NULL);
}

static int acquire_lock(void)
{
    g_lock_fd = open(LOCK_PATH, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (g_lock_fd < 0) {
        fprintf(stderr, LOG_TAG "open lock: %s\n", strerror(errno));
        return -1;
    }
    if (flock(g_lock_fd, LOCK_EX | LOCK_NB) != 0) {
        fprintf(stderr, LOG_TAG "already running (lock busy)\n");
        close(g_lock_fd);
        g_lock_fd = -1;
        return -1;
    }
    return 0;
}

static void release_lock(void)
{
    if (g_lock_fd >= 0) {
        flock(g_lock_fd, LOCK_UN);
        close(g_lock_fd);
        g_lock_fd = -1;
    }
}

static void serial_send(int fd, const char *msg)
{
    if (fd < 0 || !msg)
        return;
    size_t len = strlen(msg);
    // fd is O_NONBLOCK so a stalled USB CDC endpoint can never hang the daemon.
    (void)write(fd, msg, len);
}

static int write_splim(int limit)
{
    if (limit < 5 || limit > 200)
        return -1;

    char buf[48];
    int blen = snprintf(buf, sizeof(buf), "%d %ld\n", limit, (long)time(NULL));
    if (blen <= 0 || (size_t)blen >= sizeof(buf))
        return -1;

    int fd = open(SPLIM_TMP, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) {
        fprintf(stderr, LOG_TAG "open tmp: %s\n", strerror(errno));
        return -1;
    }
    ssize_t wn = write(fd, buf, (size_t)blen);
    int c_rc = close(fd);
    if (wn != (ssize_t)blen || c_rc != 0) {
        fprintf(stderr, LOG_TAG "write tmp: %s\n", strerror(errno));
        unlink(SPLIM_TMP);
        return -1;
    }
    if (rename(SPLIM_TMP, SPLIM_PATH) != 0) {
        fprintf(stderr, LOG_TAG "rename: %s\n", strerror(errno));
        unlink(SPLIM_TMP);
        return -1;
    }
    return 0;
}

static int configure_tty(int fd)
{
    struct termios tio;
    if (tcgetattr(fd, &tio) != 0)
        return -1;
    cfmakeraw(&tio);
    cfsetispeed(&tio, DEFAULT_BAUD);
    cfsetospeed(&tio, DEFAULT_BAUD);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;   // O_NONBLOCK + poll() drives waiting
    return tcsetattr(fd, TCSANOW, &tio);
}

static int open_device(const char *path)
{
    // Keep O_NONBLOCK after open: poll() handles waiting for RX, and non-blocking
    // writes prevent a stalled/unplugged CDC endpoint from hanging the daemon.
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
        fd = open(path, O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
        return -1;
    if (configure_tty(fd) != 0) {
        close(fd);
        return -1;
    }
    // Discard stale boot chatter.
    tcflush(fd, TCIFLUSH);
    // Greet ESP32 that CMU splim service is ready
    serial_send(fd, "CMU:OK\n");
    return fd;
}

// Prefer explicit path; otherwise pick the first /dev/ttyACM* then ttyUSB*.
static int open_auto(char *chosen, size_t chosen_len)
{
    static const char *const prefixes[] = { "ttyACM", "ttyUSB", NULL };
    DIR *d = opendir("/dev");
    if (!d)
        return -1;

    for (int p = 0; prefixes[p]; p++) {
        struct dirent *ent;
        rewinddir(d);
        while ((ent = readdir(d)) != NULL) {
            if (strncmp(ent->d_name, prefixes[p], strlen(prefixes[p])) != 0)
                continue;
            char path[64];
            snprintf(path, sizeof(path), "/dev/%s", ent->d_name);
            int fd = open_device(path);
            if (fd >= 0) {
                snprintf(chosen, chosen_len, "%s", path);
                closedir(d);
                return fd;
            }
        }
    }
    closedir(d);
    return -1;
}

// Accept "SPLIM 60", optional CR, leading log noise on the same line ignored
// only when the line starts with SPLIM (ESP32 also prints [BOOT]/ debug).
static int parse_line(const char *line, int *out_limit)
{
    const char *p = line;
    while (*p == ' ' || *p == '\t')
        p++;
    if (strncmp(p, "SPLIM", 5) != 0)
        return -1;
    p += 5;
    while (*p == ' ' || *p == '\t')
        p++;
    if (*p < '0' || *p > '9')
        return -1;
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p)
        return -1;
    // Only trailing blanks allowed. A line cut short by an unplug/reset and glued to
    // the next one ("SPLIM 6SPLIM 60") must be rejected, not read as limit 6.
    while (*end == ' ' || *end == '\t')
        end++;
    if (*end != '\0')
        return -1;
    if (v < 5 || v > 200)
        return -1;
    *out_limit = (int)v;
    return 0;
}

static void feed_byte(int fd, char *buf, size_t *len, int *discard,
                      unsigned char c, int *last_limit)
{
    static long   last_write_mono = -100;
    static time_t last_write_wall = 0;

    if (c == '\r')
        return;
    if (c == '\n') {
        if (*discard) {
            *discard = 0;
            *len = 0;
            return;
        }
        buf[*len] = '\0';
        int limit = 0;
        if (*len > 0) {
            if (parse_line(buf, &limit) == 0) {
                long   now_mono = monotonic_sec();
                time_t now_wall = time(NULL);
                // Rewrite on change, monotonic refresh cadence, or wall-clock jump.
                if (limit != *last_limit ||
                    now_mono - last_write_mono >= REFRESH_SEC ||
                    now_wall < last_write_wall ||
                    now_wall - last_write_wall >= REFRESH_SEC) {
                    if (write_splim(limit) == 0) {
                        last_write_mono = now_mono;
                        last_write_wall = now_wall;
                        if (limit != *last_limit) {
                            fprintf(stderr, LOG_TAG "splim=%d km/h\n", limit);
                            *last_limit = limit;
                        }
                    }
                }
                // Send ACK back to ESP32 so ESP32 knows the splim service wrote the limit
                if (fd >= 0) {
                    char ack[32];
                    snprintf(ack, sizeof(ack), "CMU:ACK %d\n", limit);
                    serial_send(fd, ack);
                }
            } else if (strncmp(buf, "PING", 4) == 0 || strncmp(buf, "#", 1) == 0) {
                // Reply to ping/keepalive from ESP32
                serial_send(fd, "CMU:PONG\n");
            }
        }
        *len = 0;
        return;
    }
    if (*discard)
        return;
    if (*len + 1 < SPLIM_LINE_MAX)
        buf[(*len)++] = (char)c;
    else {
        // Line exceeded SPLIM_LINE_MAX: ignore remaining bytes until '\n'
        // rather than treating the tail as a new line.
        *discard = 1;
        *len = 0;
    }
}

int main(int argc, char **argv)
{
    signal(SIGPIPE, SIG_IGN);
    signal(SIGTERM, on_signal);
    signal(SIGINT,  on_signal);

    // Close any inherited non-CLOEXEC fds from launcher (D-Bus sockets, logs, etc.).
    long max_fd = sysconf(_SC_OPEN_MAX);
    if (max_fd < 256) max_fd = 256;
    if (max_fd > 1024) max_fd = 1024;
    for (int fd = 3; fd < (int)max_fd; fd++)
        close(fd);

    if (acquire_lock() != 0)
        return 1;

    const char *want = NULL;
    if (argc >= 2 && argv[1][0])
        want = argv[1];
    else if (getenv("SPLIM_SERIAL_DEV") && getenv("SPLIM_SERIAL_DEV")[0])
        want = getenv("SPLIM_SERIAL_DEV");

    char path[64] = "";
    int fd = -1;
    int last_limit = -1;
    long last_hb_mono = 0;
    char line[SPLIM_LINE_MAX];
    size_t llen = 0;
    int discard = 0;

    fprintf(stderr, LOG_TAG "starting (want=%s)\n",
            want ? want : "auto ttyACM*/ttyUSB*");

    while (!g_stop) {
        if (fd < 0) {
            if (want) {
                snprintf(path, sizeof(path), "%s", want);
                fd = open_device(want);
            } else {
                fd = open_auto(path, sizeof(path));
            }
            if (fd < 0) {
                sleep(2);
                continue;
            }
            fprintf(stderr, LOG_TAG "opened %s\n", path);
            llen = 0;
            discard = 0;
            last_hb_mono = monotonic_sec();
        }

        // Wait with poll(), not a bare read(): once the ESP32 is unplugged the tty is
        // hung up and read() returns 0 forever, which would spin the CPU and never
        // reopen. poll() times out when idle (lets us notice g_stop) and reports a
        // hangup explicitly.
        struct pollfd pfd = { fd, POLLIN, 0 };
        int pr = poll(&pfd, 1, 1000);
        if (pr < 0) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, LOG_TAG "poll: %s — reopening\n", strerror(errno));
            close(fd);
            fd = -1;
            sleep(1);
            continue;
        }

        // Periodic monotonic heartbeat every 2 s regardless of RX chatter, so
        // ESP32 never falsely times out while receiving non-SPLIM debug lines.
        long now_mono = monotonic_sec();
        if (now_mono - last_hb_mono >= 2 && fd >= 0) {
            last_hb_mono = now_mono;
            serial_send(fd, "CMU:HB\n");
        }

        if (pr == 0)
            continue;   // idle

        if (pfd.revents & POLLIN) {
            unsigned char chunk[64];
            ssize_t n = read(fd, chunk, sizeof(chunk));
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
                    continue;
                fprintf(stderr, LOG_TAG "read error (%s) — reopening\n", strerror(errno));
                close(fd);
                fd = -1;
                sleep(1);
                continue;
            }
            if (n == 0) {
                fprintf(stderr, LOG_TAG "port lost (hangup) — reopening\n");
                close(fd);
                fd = -1;
                sleep(1);
                continue;
            }
            for (ssize_t i = 0; i < n; i++)
                feed_byte(fd, line, &llen, &discard, chunk[i], &last_limit);
        }

        if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
            fprintf(stderr, LOG_TAG "port hangup/error (0x%x) — reopening\n",
                    (unsigned)pfd.revents);
            close(fd);
            fd = -1;
            sleep(1);
            continue;
        }
    }

    if (fd >= 0)
        close(fd);
    release_lock();
    return 0;
}
