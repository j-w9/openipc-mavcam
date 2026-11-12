/*
 * Read-only MAVLink FTP server.
 *
 * Serves the camera definition file ("camdef.xml") and the media directory, so
 * a ground station can fetch the settings description and download photos and
 * recordings. One session at a time.
 */
#include "mavcam.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

enum {
    OP_NONE = 0, OP_TERMINATE = 1, OP_RESET = 2, OP_LIST = 3, OP_OPEN_RO = 4, OP_READ = 5,
    OP_CRC32 = 14, OP_BURST_READ = 15, OP_ACK = 128, OP_NAK = 129,
};
enum {
    ERR_FAIL = 1, ERR_FAIL_ERRNO = 2, ERR_INVALID_SIZE = 3, ERR_INVALID_SESSION = 4,
    ERR_NO_SESSIONS = 5, ERR_EOF = 6, ERR_UNKNOWN_CMD = 7, ERR_NOT_FOUND = 10,
};

#define HDR_LEN 12
#define MAX_DATA (MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - HDR_LEN)   /* 239 */
#define MAX_BURST_PACKETS 64

struct ftp_hdr {
    uint16_t seq;
    uint8_t session, opcode, size, req_opcode, burst_complete;
    uint32_t offset;
};

static struct {
    bool open;
    FILE *file;               /* NULL when serving the in-memory definition file */
    const char *mem;
    uint32_t size;

    bool burst;               /* a burst read is in progress */
    uint32_t burst_offset;
    double tokens;            /* rate limiter: bytes we may send now */
    long last_refill_ms;
    long burst_start_ms;
    int burst_packets;
    uint16_t burst_seq;
    uint8_t burst_chunk;
    uint8_t peer_sys, peer_comp;
    int peer_dst;
} s;

static void put_hdr(uint8_t *p, const struct ftp_hdr *h)
{
    p[0] = (uint8_t)(h->seq & 0xff);
    p[1] = (uint8_t)(h->seq >> 8);
    p[2] = h->session;
    p[3] = h->opcode;
    p[4] = h->size;
    p[5] = h->req_opcode;
    p[6] = h->burst_complete;
    p[7] = 0;
    p[8] = (uint8_t)(h->offset & 0xff);
    p[9] = (uint8_t)((h->offset >> 8) & 0xff);
    p[10] = (uint8_t)((h->offset >> 16) & 0xff);
    p[11] = (uint8_t)(h->offset >> 24);
}

static void reply(const struct ftp_hdr *h, const void *data, uint8_t sys, uint8_t comp, int dst)
{
    uint8_t payload[MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN];
    memset(payload, 0, sizeof(payload));
    put_hdr(payload, h);
    if (h->size > 0 && data != NULL) {
        memcpy(payload + HDR_LEN, data, h->size);
    }
    mavlink_message_t msg;
    mavlink_msg_file_transfer_protocol_pack(mavcam_sysid(), mavcam_compid(), &msg, 0, sys, comp, payload);
    mavcam_send(&msg, dst);
}

static void close_session(void)
{
    if (s.file != NULL) {
        fclose(s.file);
    }
    memset(&s, 0, sizeof(s));
}

/* Turn a client path into a real one. Returns:
 *   0  root of the virtual tree (media directory plus the definition file)
 *   1  the definition file
 *   2  a path inside the media directory, written to out
 *  -1  rejected */
static int resolve(const char *in, char *out, size_t outlen)
{
    /* some clients pass the "[;comp=N]" selector of an mftp:// address through as part of the path */
    while (*in == '/') {
        in++;
    }
    if (strncmp(in, "[;comp=", 7) == 0) {
        const char *end = strchr(in, ']');
        if (end == NULL) {
            return -1;
        }
        in = end + 1;
    }
    while (*in == '/' || *in == '.') {
        if (in[0] == '.' && in[1] == '.') {
            return -1;
        }
        in++;
    }
    if (*in == 0) {
        return 0;
    }
    if (strcmp(in, CAMDEF_NAME) == 0) {
        return 1;
    }
    if (strstr(in, "..") != NULL) {
        return -1;   /* no climbing out of the media directory */
    }
    const int n = snprintf(out, outlen, "%s/%s", mavcam_media_dir(), in);
    return n > 0 && n < (int)outlen ? 2 : -1;
}

static bool read_at(uint32_t offset, uint8_t *buf, uint8_t want, uint8_t *got)
{
    if (offset >= s.size) {
        *got = 0;
        return true;
    }
    uint32_t n = s.size - offset;
    if (n > want) {
        n = want;
    }
    if (s.file == NULL) {
        memcpy(buf, s.mem + offset, n);
    } else if (fseeko(s.file, (off_t)offset, SEEK_SET) != 0 || fread(buf, 1, n, s.file) != n) {
        return false;
    }
    *got = (uint8_t)n;
    return true;
}

static void nak(const struct ftp_hdr *req, uint8_t err, uint8_t sys, uint8_t comp, int dst)
{
    struct ftp_hdr h = { .seq = (uint16_t)(req->seq + 1), .session = req->session, .opcode = OP_NAK,
                         .size = 1, .req_opcode = req->opcode, .offset = req->offset };
    reply(&h, &err, sys, comp, dst);
}

static void do_list(const struct ftp_hdr *req, const char *path, uint8_t sys, uint8_t comp, int dst)
{
    char real[512];
    const int kind = resolve(path, real, sizeof(real));
    if (kind < 0 || kind == 1) {
        nak(req, ERR_NOT_FOUND, sys, comp, dst);
        return;
    }
    DIR *d = opendir(kind == 0 ? mavcam_media_dir() : real);

    char data[MAX_DATA];
    size_t used = 0;
    uint32_t index = 0;
    char entry[300];

    if (kind == 0) {
        size_t len;
        params_camdef(&len);
        if (index++ >= req->offset) {
            used = (size_t)snprintf(data, sizeof(data), "F%s\t%zu", CAMDEF_NAME, len) + 1;
        }
    }
    struct dirent *de;
    while (d != NULL && (de = readdir(d)) != NULL) {
        if (strcmp(de->d_name, ".") == 0 || strcmp(de->d_name, "..") == 0) {
            continue;
        }
        if (index++ < req->offset) {
            continue;
        }
        char full[800];
        snprintf(full, sizeof(full), "%s/%s", kind == 0 ? mavcam_media_dir() : real, de->d_name);
        struct stat st;
        if (stat(full, &st) != 0) {
            continue;
        }
        int n;
        if (S_ISDIR(st.st_mode)) {
            n = snprintf(entry, sizeof(entry), "D%s", de->d_name);
        } else {
            n = snprintf(entry, sizeof(entry), "F%s\t%lld", de->d_name, (long long)st.st_size);
        }
        if (n <= 0 || (size_t)n + 1 > sizeof(data)) {
            continue;   /* name too long to ever fit */
        }
        if (used + (size_t)n + 1 > sizeof(data)) {
            break;      /* packet full: the client asks again from the next index */
        }
        memcpy(data + used, entry, (size_t)n + 1);
        used += (size_t)n + 1;
    }
    if (d != NULL) {
        closedir(d);
    }
    if (used == 0) {
        nak(req, ERR_EOF, sys, comp, dst);
        return;
    }
    struct ftp_hdr h = { .seq = (uint16_t)(req->seq + 1), .session = req->session, .opcode = OP_ACK,
                         .size = (uint8_t)used, .req_opcode = req->opcode, .offset = req->offset };
    reply(&h, data, sys, comp, dst);
}

static void do_open(const struct ftp_hdr *req, const char *path, uint8_t sys, uint8_t comp, int dst)
{
    if (s.open) {
        /* a client that lost its ACK re-opens: allow the same peer to start over */
        if (s.peer_sys != sys || s.peer_comp != comp) {
            nak(req, ERR_NO_SESSIONS, sys, comp, dst);
            return;
        }
        close_session();
    }
    char real[512];
    const int kind = resolve(path, real, sizeof(real));
    if (kind == 1) {
        size_t len;
        s.mem = params_camdef(&len);
        s.size = (uint32_t)len;
        if (s.mem == NULL) {
            nak(req, ERR_NOT_FOUND, sys, comp, dst);
            return;
        }
    } else if (kind == 2) {
        struct stat st;
        if (stat(real, &st) != 0 || !S_ISREG(st.st_mode) || (s.file = fopen(real, "rb")) == NULL) {
            s.file = NULL;
            nak(req, ERR_NOT_FOUND, sys, comp, dst);
            return;
        }
        s.size = st.st_size > 0xffffffffLL ? 0xffffffffu : (uint32_t)st.st_size;
    } else {
        nak(req, ERR_NOT_FOUND, sys, comp, dst);
        return;
    }
    s.open = true;
    s.peer_sys = sys;
    s.peer_comp = comp;
    s.peer_dst = dst;
    mavcam_log("ftp open %s (%u bytes)", path, s.size);

    struct ftp_hdr h = { .seq = (uint16_t)(req->seq + 1), .session = 0, .opcode = OP_ACK,
                         .size = 4, .req_opcode = req->opcode };
    const uint8_t size_le[4] = { (uint8_t)s.size, (uint8_t)(s.size >> 8), (uint8_t)(s.size >> 16),
                                 (uint8_t)(s.size >> 24) };
    reply(&h, size_le, sys, comp, dst);
}

bool ftp_handle_message(const mavlink_message_t *msg, int src)
{
    if (msg->msgid != MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
        return false;
    }
    mavlink_file_transfer_protocol_t f;
    mavlink_msg_file_transfer_protocol_decode(msg, &f);
    /* FTP is never broadcast: only answer requests addressed to this component */
    if (f.target_system != mavcam_sysid() || f.target_component != mavcam_compid()) {
        return false;
    }
    const uint8_t *p = f.payload;
    const struct ftp_hdr req = {
        .seq = (uint16_t)(p[0] | (p[1] << 8)), .session = p[2], .opcode = p[3], .size = p[4],
        .offset = (uint32_t)p[8] | ((uint32_t)p[9] << 8) | ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24),
    };
    const uint8_t sys = msg->sysid, comp = msg->compid;
    if (req.size > MAX_DATA) {
        nak(&req, ERR_INVALID_SIZE, sys, comp, src);
        return true;
    }
    char path[MAX_DATA + 1];
    memcpy(path, p + HDR_LEN, req.size);
    path[req.size] = 0;

    struct ftp_hdr ack = { .seq = (uint16_t)(req.seq + 1), .session = req.session, .opcode = OP_ACK,
                           .req_opcode = req.opcode, .offset = req.offset };

    switch (req.opcode) {
    case OP_NONE:
        reply(&ack, NULL, sys, comp, src);
        break;

    case OP_TERMINATE:
    case OP_RESET:
        close_session();
        reply(&ack, NULL, sys, comp, src);
        break;

    case OP_LIST:
        do_list(&req, path, sys, comp, src);
        break;

    case OP_OPEN_RO:
        do_open(&req, path, sys, comp, src);
        break;

    case OP_READ: {
        if (!s.open || req.session != 0) {
            nak(&req, ERR_INVALID_SESSION, sys, comp, src);
            break;
        }
        uint8_t data[MAX_DATA], got = 0;
        const uint8_t want = req.size == 0 || req.size > MAX_DATA ? MAX_DATA : req.size;
        if (!read_at(req.offset, data, want, &got)) {
            nak(&req, ERR_FAIL, sys, comp, src);
        } else if (got == 0) {
            nak(&req, ERR_EOF, sys, comp, src);
        } else {
            ack.size = got;
            reply(&ack, data, sys, comp, src);
        }
        break;
    }

    case OP_BURST_READ:
        if (!s.open || req.session != 0) {
            nak(&req, ERR_INVALID_SESSION, sys, comp, src);
            break;
        }
        if (req.offset >= s.size) {
            s.burst = false;
            nak(&req, ERR_EOF, sys, comp, src);
            break;
        }
        s.burst = true;
        s.burst_offset = req.offset;
        s.burst_seq = (uint16_t)(req.seq + 1);
        s.burst_packets = 0;
        s.burst_start_ms = 0;
        s.burst_chunk = req.size == 0 || req.size > MAX_DATA ? MAX_DATA : req.size;
        s.peer_sys = sys;
        s.peer_comp = comp;
        s.peer_dst = src;
        break;

    default:
        nak(&req, ERR_UNKNOWN_CMD, sys, comp, src);
        break;
    }
    return true;
}

bool ftp_periodic(void)
{
    if (!s.burst) {
        return false;
    }
    /* Pace the burst.
     * On a serial port: wait until what is already queued has drained, rather
     * than piling file data on top of telemetry. */
    if (mavcam_backlog(s.peer_dst) > 2 * MAVLINK_MAX_PACKET_LEN) {
        return true;
    }
    /* On any link: stay under the configured byte rate, so a download cannot
     * swamp a radio link that has no way to push back. */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    const long now = ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
    const double rate = mavcam_ftp_rate();
    if (s.burst_start_ms == 0) {
        s.burst_start_ms = now;
    }
    if (s.last_refill_ms == 0 || now - s.last_refill_ms > 1000) {
        s.last_refill_ms = now;   /* idle since the last transfer: no back credit */
    }
    s.tokens += (double)(now - s.last_refill_ms) * rate / 1000.0;
    s.last_refill_ms = now;
    if (s.tokens > rate / 10 + MAVLINK_MAX_PACKET_LEN) {
        s.tokens = rate / 10 + MAVLINK_MAX_PACKET_LEN;   /* at most 100 ms of credit */
    }
    if (s.tokens < MAVLINK_MAX_PACKET_LEN) {
        return true;
    }
    uint8_t data[MAX_DATA], got = 0;
    if (!read_at(s.burst_offset, data, s.burst_chunk, &got) || got == 0) {
        s.burst = false;
        return false;
    }
    s.tokens -= got + HDR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    s.burst_packets++;
    /* End the burst after about a second (or MAX_BURST_PACKETS) even if the
     * file is not finished: clients such as pymavlink give up on a transfer if
     * they go a few seconds without being asked to request more. */
    const bool last = s.burst_offset + got >= s.size || s.burst_packets >= MAX_BURST_PACKETS ||
                      now - s.burst_start_ms >= 1000;
    const struct ftp_hdr h = { .seq = s.burst_seq++, .session = 0, .opcode = OP_ACK, .size = got,
                               .req_opcode = OP_BURST_READ, .burst_complete = last, .offset = s.burst_offset };
    reply(&h, data, s.peer_sys, s.peer_comp, s.peer_dst);
    s.burst_offset += got;
    if (last) {
        s.burst = false;
    }
    return s.burst;
}
