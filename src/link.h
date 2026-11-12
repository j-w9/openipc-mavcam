#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/types.h>

/* MAVLink transport to the flight controller.
 *   /dev/ttyS2:115200     serial
 *   udp:HOST:PORT         send to HOST:PORT, receive replies on the same socket
 *   udpin:PORT            listen on PORT, reply to whoever sent last
 *   tcp:HOST:PORT         TCP client (reconnects)
 */
struct link {
    enum { LINK_SERIAL, LINK_UDP, LINK_UDPIN, LINK_TCP } kind;
    char target[128];
    int port_or_baud;
    int fd;
    struct sockaddr_storage peer;
    socklen_t peer_len;
    long last_open_attempt_ms;

    /* Stream links (serial, TCP) queue what the port cannot take yet, so a
     * frame is always sent whole or not at all. UDP sends are atomic. */
    uint8_t txbuf[8192];
    size_t txlen;
    unsigned long tx_dropped;   /* frames discarded because the queue was full */
};

bool link_parse(struct link *l, const char *spec);
/* (re)open if needed; returns true if the link has a usable fd */
bool link_ensure_open(struct link *l, long now_ms);
ssize_t link_read(struct link *l, uint8_t *buf, size_t len);
/* queue or send one whole frame. false if it had to be dropped */
bool link_write(struct link *l, const uint8_t *buf, size_t len);
/* push queued bytes to the port; call when it is writable */
void link_flush(struct link *l);
/* bytes waiting to be written */
size_t link_backlog(const struct link *l);
