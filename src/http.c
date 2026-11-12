#include "http.h"

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static int wait_fd(int fd, short events, long deadline)
{
    long left = deadline - now_ms();
    if (left <= 0) {
        return 0;
    }
    struct pollfd p = { .fd = fd, .events = events };
    int r;
    do {
        r = poll(&p, 1, (int)left);
    } while (r < 0 && errno == EINTR);
    return r > 0;
}

static int connect_to(const char *host, int port, long deadline)
{
    char portstr[16];
    snprintf(portstr, sizeof(portstr), "%d", port);
    struct addrinfo hints = { .ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;
    if (getaddrinfo(host, portstr, &hints, &res) != 0) {
        return -1;
    }
    int fd = -1;
    for (struct addrinfo *ai = res; ai != NULL; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) {
            continue;
        }
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
        int r = connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (r < 0 && errno == EINPROGRESS && wait_fd(fd, POLLOUT, deadline)) {
            int err = 0;
            socklen_t len = sizeof(err);
            getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);
            r = err == 0 ? 0 : -1;
        }
        if (r == 0) {
            break;
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);
    return fd;
}

long http_get(const char *host, int port, const char *path, int timeout_ms,
              size_t max_body, int *status, char **body)
{
    *body = NULL;
    *status = 0;
    const long deadline = now_ms() + timeout_ms;
    int fd = connect_to(host, port, deadline);
    if (fd < 0) {
        return -1;
    }

    char req[512];
    int n = snprintf(req, sizeof(req),
                     "GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n", path, host);
    if (n <= 0 || n >= (int)sizeof(req)) {
        close(fd);
        return -1;
    }
    for (int off = 0; off < n;) {
        if (!wait_fd(fd, POLLOUT, deadline)) {
            close(fd);
            return -1;
        }
        ssize_t w = send(fd, req + off, n - off, 0);
        if (w < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                continue;
            }
            close(fd);
            return -1;
        }
        off += (int)w;
    }

    size_t cap = 4096, len = 0;
    const size_t max_total = max_body + 8192;   /* allow for headers */
    char *buf = malloc(cap + 1);
    int failed = buf == NULL;
    while (!failed) {
        if (!wait_fd(fd, POLLIN, deadline)) {
            failed = 1;
            break;
        }
        if (len == cap) {
            if (cap >= max_total) {
                failed = 1;
                break;
            }
            cap *= 2;
            char *nb = realloc(buf, cap + 1);
            if (nb == NULL) {
                failed = 1;
                break;
            }
            buf = nb;
        }
        ssize_t r = recv(fd, buf + len, cap - len, 0);
        if (r == 0) {
            break;
        }
        if (r < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                continue;
            }
            failed = 1;
            break;
        }
        len += (size_t)r;
    }
    close(fd);
    if (failed) {
        free(buf);
        return -1;
    }
    buf[len] = 0;

    /* status line: HTTP/1.x NNN ... */
    if (len < 12 || strncmp(buf, "HTTP/", 5) != 0) {
        free(buf);
        return -1;
    }
    const char *sp = memchr(buf, ' ', len);
    *status = sp ? atoi(sp + 1) : 0;

    /* find the end of the headers without assuming a text body */
    size_t hdr_end = 0;
    for (size_t i = 0; i + 3 < len; i++) {
        if (memcmp(buf + i, "\r\n\r\n", 4) == 0) {
            hdr_end = i + 4;
            break;
        }
    }
    if (hdr_end == 0 || len - hdr_end > max_body) {
        free(buf);
        return -1;
    }
    const size_t blen = len - hdr_end;
    memmove(buf, buf + hdr_end, blen);
    buf[blen] = 0;
    *body = buf;
    return (long)blen;
}
