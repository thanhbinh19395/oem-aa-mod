// SPDX-License-Identifier: AGPL-3.0-or-later
//
// splim_udpd — shared speed-limit UDP listener for Mazda Connect (CMU150).
// Copyright (C) 2026 KID MIXER-MODER.
//
// Part of an AGPL-3.0 work — see NOTICE.md for full attribution.
//
// DIAGNOSTIC-ONLY build: the UDP wire format isn't confirmed yet, so this daemon
// does NOTHING but bind UDP:50505 and log every datagram it receives (sender,
// length, printable ASCII, raw hex) to /data_persist/splim_udpd.log. It does NOT
// parse the payload and does NOT write /data_persist/splim — that comes later,
// once the format is known from these logs.
//
// EXCLUSIVITY: SO_REUSEADDR is deliberately NOT set so a second instance's
// bind() fails with EADDRINUSE and the duplicate exits — the "only one daemon"
// backstop the launcher's pgrep guard relies on (see hud_send.cpp).

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define SPLIM_PORT   50505
#define SPLIM_LOG    "/data_persist/splim_udpd.log"

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig) { (void)sig; g_stop = 1; }

// Append one line per received datagram to SPLIM_LOG so the (still unconfirmed)
// wire format can be reverse-engineered from a real sender: sender addr, length,
// the printable ASCII view, and a raw hex dump. Line-buffered so `tail -f` sees
// packets live. (Writes to flash on every packet — this is a bring-up aid.)
static void log_packet(const struct sockaddr_in *src, const unsigned char *buf, ssize_t n)
{
    static FILE *lf = NULL;
    if (!lf) {
        lf = fopen(SPLIM_LOG, "a");
        if (!lf) return;
        setvbuf(lf, NULL, _IOLBF, 0);
    }

    fprintf(lf, "t=%ld from=%s:%u len=%zd ascii=\"",
            (long)time(NULL),
            src ? inet_ntoa(src->sin_addr) : "?",
            src ? (unsigned)ntohs(src->sin_port) : 0u,
            n);
    for (ssize_t i = 0; i < n; i++)
        fputc((buf[i] >= 0x20 && buf[i] < 0x7f) ? buf[i] : '.', lf);
    fprintf(lf, "\" hex=");
    for (ssize_t i = 0; i < n; i++) fprintf(lf, "%02x ", buf[i]);
    fputc('\n', lf);
}

int main(void)
{
    // SIGPIPE can't hurt a UDP socket, but ignore it defensively. Handle
    // TERM/INT so a supervisor can stop us cleanly.
    signal(SIGPIPE, SIG_IGN);
    signal(SIGTERM, on_signal);
    signal(SIGINT,  on_signal);

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        fprintf(stderr, "splim_udpd: socket: %s\n", strerror(errno));
        return 1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);   // any iface (CarPlay usb0/ncm0, etc.)
    addr.sin_port        = htons(SPLIM_PORT);

    // No SO_REUSEADDR: a second instance MUST fail here and exit (exclusivity).
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "splim_udpd: bind(:%d): %s (already running?)\n",
                SPLIM_PORT, strerror(errno));
        close(fd);
        return 1;
    }

    char buf[64];
    while (!g_stop) {
        struct sockaddr_in src;
        socklen_t srclen = sizeof(src);
        memset(&src, 0, sizeof(src));
        ssize_t n = recvfrom(fd, buf, sizeof(buf) - 1, 0,
                             (struct sockaddr *)&src, &srclen);
        if (n < 0) {
            if (errno == EINTR) continue;   // woken by a signal
            break;
        }
        buf[n] = '\0';

        // Log-only: record the raw datagram. No parse, no write to
        // /data_persist/splim yet — added once the format is confirmed.
        log_packet(&src, (const unsigned char *)buf, n);
    }

    close(fd);
    return 0;
}
