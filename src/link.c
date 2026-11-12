#include "link.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static bool split_host_port(const char *s, char *host, size_t hostlen, int *port)
{
    const char *colon = strrchr(s, ':');
    if (colon == NULL || colon == s) {
        return false;
    }
    const size_t n = (size_t)(colon - s);
    if (n >= hostlen) {
        return false;
    }
    memcpy(host, s, n);
    host[n] = 0;
    *port = atoi(colon + 1);
    return *port > 0 && *port < 65536;
}

bool link_parse(struct link *l, const char *spec)
{
    memset(l, 0, sizeof(*l));
    l->fd = -1;
    l->last_open_attempt_ms = -100000;
    if (strncmp(spec, "udp:", 4) == 0) {
        l->kind = LINK_UDP;
        return split_host_port(spec + 4, l->target, sizeof(l->target), &l->port_or_baud);
    }
    if (strncmp(spec, "tcp:", 4) == 0) {
        l->kind = LINK_TCP;
        return split_host_port(spec + 4, l->target, sizeof(l->target), &l->port_or_baud);
    }
    if (strncmp(spec, "udpin:", 6) == 0) {
        l->kind = LINK_UDPIN;
        l->port_or_baud = atoi(spec + 6);
        return l->port_or_baud > 0 && l->port_or_baud < 65536;
    }
    l->kind = LINK_SERIAL;
    l->port_or_baud = 115200;
    const char *colon = strrchr(spec, ':');
    size_t n = strlen(spec);
    if (colon != NULL) {
        n = (size_t)(colon - spec);
        l->port_or_baud = atoi(colon + 1);
    }
    if (n == 0 || n >= sizeof(l->target)) {
        return false;
    }
    memcpy(l->target, spec, n);
    l->target[n] = 0;
    return l->port_or_baud > 0;
}

static speed_t baud_const(int baud)
{
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return B0;
    }
}

static int open_serial(const char *dev, int baud)
{
    const speed_t sp = baud_const(baud);
    if (sp == B0) {
        fprintf(stderr, "mavcam: unsupported baud rate %d\n", baud);
        return -1;
    }
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }
    struct termios t;
    if (tcgetattr(fd, &t) != 0) {
        close(fd);
        return -1;
    }
    cfmakeraw(&t);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(tcflag_t)(CRTSCTS | CSTOPB | PARENB);
    cfsetispeed(&t, sp);
    cfsetospeed(&t, sp);
    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static int open_inet(struct link *l)
{
    char portstr[16];
    snprintf(portstr, sizeof(portstr), "%d", l->port_or_baud);
    struct addrinfo hints = {
        .ai_family = l->kind == LINK_UDPIN ? AF_INET : AF_UNSPEC,
        .ai_socktype = l->kind == LINK_TCP ? SOCK_STREAM : SOCK_DGRAM,
        .ai_flags = l->kind == LINK_UDPIN ? AI_PASSIVE : 0,
    };
    struct addrinfo *res = NULL;
    if (getaddrinfo(l->kind == LINK_UDPIN ? NULL : l->target, portstr, &hints, &res) != 0 || res == NULL) {
        return -1;
    }
    int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd >= 0) {
        int one = 1;
        bool ok = true;
        if (l->kind == LINK_UDPIN) {
            setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
            ok = bind(fd, res->ai_addr, res->ai_addrlen) == 0;
            l->peer_len = 0;
        } else if (l->kind == LINK_UDP) {
            memcpy(&l->peer, res->ai_addr, res->ai_addrlen);
            l->peer_len = res->ai_addrlen;
        } else {
            ok = connect(fd, res->ai_addr, res->ai_addrlen) == 0;
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        }
        if (!ok) {
            close(fd);
            fd = -1;
        } else {
            fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
        }
    }
    freeaddrinfo(res);
    return fd;
}

bool link_ensure_open(struct link *l, long now_ms)
{
    if (l->fd >= 0) {
        return true;
    }
    if (now_ms - l->last_open_attempt_ms < 1000) {
        return false;
    }
    l->last_open_attempt_ms = now_ms;
    l->fd = l->kind == LINK_SERIAL ? open_serial(l->target, l->port_or_baud) : open_inet(l);
    return l->fd >= 0;
}

static void link_close(struct link *l)
{
    if (l->fd >= 0) {
        close(l->fd);
        l->fd = -1;
    }
    l->txlen = 0;
}

ssize_t link_read(struct link *l, uint8_t *buf, size_t len)
{
    if (l->fd < 0) {
        return -1;
    }
    ssize_t r;
    if (l->kind == LINK_UDPIN) {
        struct sockaddr_storage from;
        socklen_t flen = sizeof(from);
        r = recvfrom(l->fd, buf, len, 0, (struct sockaddr *)&from, &flen);
        if (r > 0) {
            l->peer = from;
            l->peer_len = flen;
        }
    } else if (l->kind == LINK_UDP) {
        r = recvfrom(l->fd, buf, len, 0, NULL, NULL);
    } else {
        r = read(l->fd, buf, len);
    }
    if (r < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || errno == ECONNREFUSED) {
            return 0;
        }
        link_close(l);
        return -1;
    }
    if (r == 0 && (l->kind == LINK_TCP || l->kind == LINK_SERIAL)) {
        /* TCP peer closed, or serial device went away */
        if (l->kind == LINK_TCP) {
            link_close(l);
            return -1;
        }
    }
    return r;
}

static bool is_stream(const struct link *l)
{
    return l->kind == LINK_SERIAL || l->kind == LINK_TCP;
}

void link_flush(struct link *l)
{
    while (l->fd >= 0 && l->txlen > 0) {
        const ssize_t w = write(l->fd, l->txbuf, l->txlen);
        if (w > 0) {
            l->txlen -= (size_t)w;
            memmove(l->txbuf, l->txbuf + w, l->txlen);
        } else if (w < 0 && errno == EINTR) {
            continue;
        } else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;   /* port is full: try again when it is writable */
        } else {
            if (l->kind == LINK_TCP) {
                link_close(l);
            }
            l->txlen = 0;
            break;
        }
    }
}

size_t link_backlog(const struct link *l)
{
    return is_stream(l) ? l->txlen : 0;
}

bool link_write(struct link *l, const uint8_t *buf, size_t len)
{
    if (l->fd < 0) {
        return false;
    }
    if (!is_stream(l)) {
        if (l->peer_len == 0) {
            return false;   /* udpin: nobody has talked to us yet */
        }
        return sendto(l->fd, buf, len, 0, (const struct sockaddr *)&l->peer, l->peer_len) == (ssize_t)len;
    }
    if (len > sizeof(l->txbuf) - l->txlen) {
        l->tx_dropped++;    /* never send part of a frame */
        return false;
    }
    memcpy(l->txbuf + l->txlen, buf, len);
    l->txlen += len;
    link_flush(l);
    return true;
}
