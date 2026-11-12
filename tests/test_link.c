/*
 * Unit test for the stream-link output queue.
 *
 * A socketpair with a tiny send buffer stands in for a UART that cannot keep
 * up. Whatever happens, the far end must only ever see whole frames, in order.
 */
#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "../src/link.h"

#define FRAME 200

static void make_frame(uint8_t *f, unsigned n)
{
    /* every byte of a frame carries its number, so a splice is detectable */
    f[0] = 0xFD;
    for (int i = 1; i < FRAME; i++) {
        f[i] = (uint8_t)(n & 0x7f);
    }
}

int main(void)
{
    int sv[2];
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, sv) == 0);
    int small = 1024;
    setsockopt(sv[0], SOL_SOCKET, SO_SNDBUF, &small, sizeof(small));
    fcntl(sv[0], F_SETFL, fcntl(sv[0], F_GETFL, 0) | O_NONBLOCK);
    fcntl(sv[1], F_SETFL, fcntl(sv[1], F_GETFL, 0) | O_NONBLOCK);

    struct link l;
    memset(&l, 0, sizeof(l));
    l.kind = LINK_TCP;
    l.fd = sv[0];

    static uint8_t got[4 * 1024 * 1024];
    size_t got_len = 0;
    unsigned accepted = 0, refused = 0;
    uint8_t frame[FRAME];
    static uint8_t order[20000];   /* frame numbers accepted, in order */

    /* write far faster than the reader drains */
    for (unsigned n = 0; n < 20000; n++) {
        make_frame(frame, n);
        if (link_write(&l, frame, sizeof(frame))) {
            order[accepted++] = (uint8_t)(n & 0x7f);
        } else {
            refused++;
        }
        assert(link_backlog(&l) <= sizeof(l.txbuf));
        if (n % 7 == 0) {   /* a slow reader */
            ssize_t r = read(sv[1], got + got_len, 150);
            if (r > 0) {
                got_len += (size_t)r;
            }
            link_flush(&l);
        }
    }
    /* let everything queued drain */
    for (int spins = 0; spins < 100000 && (link_backlog(&l) > 0 || spins < 10); spins++) {
        link_flush(&l);
        ssize_t r = read(sv[1], got + got_len, sizeof(got) - got_len);
        if (r > 0) {
            got_len += (size_t)r;
        }
    }

    assert(refused > 0 && "the test never backed the link up");
    assert(l.tx_dropped == refused);
    assert(link_backlog(&l) == 0);
    assert(got_len == (size_t)accepted * FRAME && "bytes were lost or a frame was sent in part");
    for (unsigned i = 0; i < accepted; i++) {
        const uint8_t *f = got + (size_t)i * FRAME;
        assert(f[0] == 0xFD);
        for (int b = 1; b < FRAME; b++) {
            assert(f[b] == order[i] && "frames were spliced or reordered");
        }
    }
    printf("link queue: %u frames accepted and delivered whole, %u refused whole\n", accepted, refused);
    return 0;
}
