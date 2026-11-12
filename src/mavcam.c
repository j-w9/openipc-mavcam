/*
 * mavcam - MAVLink Camera Protocol v2 component for OpenIPC cameras.
 *
 * Sits on the flight-controller link (serial, UDP or TCP), presents the camera
 * as a native MAVLink camera component, and turns camera commands into calls
 * to the streamer's localhost HTTP API.
 */
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <time.h>
#include <unistd.h>

#include <stdarg.h>

#include "mavcam.h"
#include "link.h"
#include "worker.h"

#define MAVCAM_VERSION_MAJOR 0
#define MAVCAM_VERSION_MINOR 1

#define FC_WAIT_MS 5000   /* how long to wait for an autopilot before announcing without one */

struct config {
    const char *link_spec;
    int sysid;              /* 0 = adopt the autopilot's system ID */
    int compid;
    const char *vendor;
    const char *model;
    const char *media_dir;
    const char *stream_uri;
    const char *stream_name;
    int stream_type;        /* VIDEO_STREAM_TYPE */
    int hfov_deg;
    bool geotag;
    bool verbose;
    long ftp_rate;          /* bytes per second */
    const char *camdef_uri; /* NULL = served by us over MAVLink FTP */
    const char *signing_key_file;
    bool dump_camdef;
};

struct interval {
    uint32_t msgid;
    long interval_ms;       /* 0 = only on request */
    long last_ms;
};

static struct {
    struct config cfg;
    struct link link;
    struct link ground_tx;      /* radio / ground link: where we send */
    struct link ground_rx;      /* optional separate listening port for the ground link */
    bool have_ground_tx, have_ground_rx;
    int reply_dst;              /* link the request being handled arrived on */
    struct streamer streamer;
    struct stream_props props;

    uint8_t sysid;
    bool sysid_locked;
    uint8_t fc_sysid, fc_compid;
    bool fc_seen;

    long boot_ms;
    long last_heartbeat_ms;
    long last_props_ms;
    long last_record_poll_ms;
    long last_pos_req_ms;
    long last_pos_ms;

    /* capture state */
    bool recording;
    long record_start_ms;
    uint32_t record_elapsed_ms;
    int image_count;
    float image_interval_s;
    int images_remaining;   /* -1 = until stopped, 0 = idle */
    long next_image_ms;
    int zoom_level;
    int zoom_rate;          /* continuous zoom: -1 out, 0 stopped, 1 in */
    long last_zoom_step_ms;
    bool zoom_busy, props_busy, record_poll_busy;   /* a job of that kind is queued */
    int image_seq;          /* file name counter (images taken, including failures) */
    uint8_t mode;           /* CAMERA_MODE */

    /* last position from the autopilot, for CAMERA_IMAGE_CAPTURED */
    bool have_pos;
    int32_t lat, lon, alt, relative_alt;

    struct interval intervals[4];
} g;

static volatile sig_atomic_t g_stop;

static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static uint32_t boot_ms(void)
{
    return (uint32_t)(now_ms() - g.boot_ms);
}

void mavcam_log(const char *fmt, ...)
{
    if (!g.cfg.verbose) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    fputs("mavcam: ", stderr);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
}
#define LOG(...) mavcam_log(__VA_ARGS__)

/* shared MAVLink channel state: 0 = flight controller link, 1 = ground link */
static mavlink_status_t chan_status[MAVLINK_COMM_NUM_BUFFERS];
static mavlink_message_t chan_buffer[MAVLINK_COMM_NUM_BUFFERS];
mavlink_status_t *mavlink_get_channel_status(uint8_t chan) { return &chan_status[chan]; }
mavlink_message_t *mavlink_get_channel_buffer(uint8_t chan) { return &chan_buffer[chan]; }

/* ---- MAVLink 2 signing ---------------------------------------------------
 * With a key, everything this camera originates is signed, so a flight
 * controller that requires signing accepts it. Forwarded frames are passed on
 * exactly as received, signature included. */
static mavlink_signing_t signing;
static mavlink_signing_streams_t signing_streams;

static bool accept_unsigned(const mavlink_status_t *status, uint32_t msgid)
{
    (void)status;
    (void)msgid;
    return true;   /* whether to trust unsigned traffic is the flight controller's decision */
}

static bool load_signing_key(const char *path)
{
    uint8_t raw[160];
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        return false;
    }
    const size_t n = fread(raw, 1, sizeof(raw), f);
    fclose(f);
    if (n == 32) {
        memcpy(signing.secret_key, raw, 32);   /* raw key */
    } else {
        /* 64 hex digits, optionally followed by whitespace */
        size_t digits = 0;
        for (size_t i = 0; i < n && digits < 64; i++) {
            const int c = raw[i];
            int v;
            if (c >= '0' && c <= '9') {
                v = c - '0';
            } else if (c >= 'a' && c <= 'f') {
                v = c - 'a' + 10;
            } else if (c >= 'A' && c <= 'F') {
                v = c - 'A' + 10;
            } else if (c == ' ' || c == '\n' || c == '\r' || c == '\t') {
                continue;
            } else {
                return false;
            }
            if (digits % 2 == 0) {
                signing.secret_key[digits / 2] = (uint8_t)(v << 4);
            } else {
                signing.secret_key[digits / 2] |= (uint8_t)v;
            }
            digits++;
        }
        if (digits != 64) {
            return false;
        }
    }
    signing.link_id = 1;
    signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
    signing.accept_unsigned_callback = accept_unsigned;
    /* Timestamps are 10 us units since 2015. Cameras often boot with no clock;
     * the library then adopts the newest timestamp it sees on signed frames
     * from the flight controller, which arrive before we send anything. */
    const time_t utc = time(NULL);
    signing.timestamp = utc > 1600000000 ? (uint64_t)(utc - 1420070400) * 100000ull : 0;
    for (int c = 0; c < MAVLINK_COMM_NUM_BUFFERS; c++) {
        chan_status[c].signing = &signing;
        chan_status[c].signing_streams = &signing_streams;
    }
    return true;
}

void mavcam_send(const mavlink_message_t *msg, int dst)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    if (dst & DST_FC) {
        link_write(&g.link, buf, len);
    }
    if ((dst & DST_GROUND) && g.have_ground_tx) {
        link_write(&g.ground_tx, buf, len);
    }
}

size_t mavcam_backlog(int dst)
{
    size_t n = 0;
    if (dst & DST_FC) {
        n = link_backlog(&g.link);
    }
    if ((dst & DST_GROUND) && g.have_ground_tx && link_backlog(&g.ground_tx) > n) {
        n = link_backlog(&g.ground_tx);
    }
    return n;
}

double mavcam_ftp_rate(void) { return (double)g.cfg.ftp_rate; }
uint8_t mavcam_sysid(void) { return g.sysid; }
uint8_t mavcam_compid(void) { return (uint8_t)g.cfg.compid; }
const struct streamer *mavcam_streamer(void) { return &g.streamer; }
const char *mavcam_media_dir(void) { return g.cfg.media_dir; }
const char *mavcam_vendor(void) { return g.cfg.vendor; }
const char *mavcam_model(void) { return g.cfg.model; }

/* ---- outgoing messages ------------------------------------------------- */

static void send_heartbeat(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(g.sysid, g.cfg.compid, &msg, MAV_TYPE_CAMERA,
                               MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    mavcam_send(&msg, DST_ALL);
}

static void send_ack_to(uint8_t sys, uint8_t comp, int dst, uint16_t command, uint8_t result)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(g.sysid, g.cfg.compid, &msg, command, result, 0, 0, sys, comp);
    mavcam_send(&msg, dst);
}

static void send_ack(const mavlink_message_t *req, uint16_t command, uint8_t result)
{
    send_ack_to(req->sysid, req->compid, g.reply_dst, command, result);
}

static void send_camera_information(void)
{
    mavlink_camera_information_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.time_boot_ms = boot_ms();
    strncpy((char *)ci.vendor_name, g.cfg.vendor, sizeof(ci.vendor_name));
    strncpy((char *)ci.model_name, g.cfg.model, sizeof(ci.model_name));
    ci.firmware_version = (uint32_t)MAVCAM_VERSION_MAJOR | ((uint32_t)MAVCAM_VERSION_MINOR << 8);
    ci.resolution_h = g.props.width;
    ci.resolution_v = g.props.height;
    ci.flags = CAMERA_CAP_FLAGS_CAPTURE_VIDEO | CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
               CAMERA_CAP_FLAGS_HAS_MODES |
               CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
               CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
               CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM;
    if (streamer_zoom_levels(&g.streamer) > 1) {
        ci.flags |= CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM;
    }
    /* settings are described by a definition file served over MAVLink FTP */
    ci.cam_definition_version = CAMDEF_VERSION;
    if (g.cfg.camdef_uri != NULL) {
        snprintf(ci.cam_definition_uri, sizeof(ci.cam_definition_uri), "%s", g.cfg.camdef_uri);
    } else {
        snprintf(ci.cam_definition_uri, sizeof(ci.cam_definition_uri), "mftp://[;comp=%d]/%s",
                 g.cfg.compid, CAMDEF_NAME);
    }
    mavlink_message_t msg;
    mavlink_msg_camera_information_encode(g.sysid, g.cfg.compid, &msg, &ci);
    mavcam_send(&msg, DST_ALL);
}

static void send_camera_settings(void)
{
    mavlink_camera_settings_t cs;
    memset(&cs, 0, sizeof(cs));
    cs.time_boot_ms = boot_ms();
    cs.mode_id = g.mode;
    const int levels = streamer_zoom_levels(&g.streamer);
    cs.zoomLevel = levels > 1 ? 100.0f * g.zoom_level / (levels - 1) : NAN;
    cs.focusLevel = NAN;
    mavlink_message_t msg;
    mavlink_msg_camera_settings_encode(g.sysid, g.cfg.compid, &msg, &cs);
    mavcam_send(&msg, DST_ALL);
}

static bool media_space(float *total_mib, float *avail_mib)
{
    struct statvfs sv;
    if (statvfs(g.cfg.media_dir, &sv) != 0) {
        return false;
    }
    const double mib = 1024.0 * 1024.0;
    *total_mib = (float)((double)sv.f_blocks * sv.f_frsize / mib);
    *avail_mib = (float)((double)sv.f_bavail * sv.f_frsize / mib);
    return true;
}

static void send_storage_information(void)
{
    mavlink_storage_information_t si;
    memset(&si, 0, sizeof(si));
    si.time_boot_ms = boot_ms();
    si.storage_id = 1;
    si.storage_count = 1;
    float total = 0, avail = 0;
    if (media_space(&total, &avail) && access(g.cfg.media_dir, W_OK) == 0) {
        si.status = STORAGE_STATUS_READY;
        si.total_capacity = total;
        si.available_capacity = avail;
        si.used_capacity = total - avail;
    } else {
        si.status = STORAGE_STATUS_EMPTY;
    }
    si.read_speed = NAN;
    si.write_speed = NAN;
    mavlink_message_t msg;
    mavlink_msg_storage_information_encode(g.sysid, g.cfg.compid, &msg, &si);
    mavcam_send(&msg, DST_ALL);
}

static void send_capture_status(void)
{
    mavlink_camera_capture_status_t st;
    memset(&st, 0, sizeof(st));
    st.time_boot_ms = boot_ms();
    st.image_status = g.images_remaining != 0 ? 2 : 0;   /* 2: interval set but idle */
    st.video_status = g.recording ? 1 : 0;
    st.image_interval = g.images_remaining != 0 ? g.image_interval_s : 0;
    if (g.recording) {
        st.recording_time_ms = g.record_elapsed_ms ? g.record_elapsed_ms
                                                   : (uint32_t)(now_ms() - g.record_start_ms);
    }
    float total = 0, avail = 0;
    if (media_space(&total, &avail)) {
        st.available_capacity = avail;
    }
    st.image_count = g.image_count;
    mavlink_message_t msg;
    mavlink_msg_camera_capture_status_encode(g.sysid, g.cfg.compid, &msg, &st);
    mavcam_send(&msg, DST_ALL);
}

static void send_video_stream_information(void)
{
    mavlink_video_stream_information_t vi;
    memset(&vi, 0, sizeof(vi));
    vi.stream_id = 1;
    vi.count = 1;
    vi.type = (uint8_t)g.cfg.stream_type;
    vi.flags = VIDEO_STREAM_STATUS_FLAGS_RUNNING;
    vi.framerate = g.props.fps;
    vi.resolution_h = g.props.width;
    vi.resolution_v = g.props.height;
    vi.bitrate = g.props.bitrate_kbps * 1000u;
    vi.hfov = (uint16_t)g.cfg.hfov_deg;
    strncpy(vi.name, g.cfg.stream_name, sizeof(vi.name));
    strncpy(vi.uri, g.cfg.stream_uri, sizeof(vi.uri));
#ifdef HAVE_ENUM_VIDEO_STREAM_ENCODING
    vi.encoding = g.props.h265 ? VIDEO_STREAM_ENCODING_H265 : VIDEO_STREAM_ENCODING_H264;
#endif
    mavlink_message_t msg;
    mavlink_msg_video_stream_information_encode(g.sysid, g.cfg.compid, &msg, &vi);
    mavcam_send(&msg, DST_ALL);
}

static void send_video_stream_status(void)
{
    mavlink_video_stream_status_t vs;
    memset(&vs, 0, sizeof(vs));
    vs.stream_id = 1;
    vs.flags = VIDEO_STREAM_STATUS_FLAGS_RUNNING;
    vs.framerate = g.props.fps;
    vs.resolution_h = g.props.width;
    vs.resolution_v = g.props.height;
    vs.bitrate = g.props.bitrate_kbps * 1000u;
    vs.hfov = (uint16_t)g.cfg.hfov_deg;
    mavlink_message_t msg;
    mavlink_msg_video_stream_status_encode(g.sysid, g.cfg.compid, &msg, &vs);
    mavcam_send(&msg, DST_ALL);
}

/* send one of the messages we can produce on request. false if not ours */
static bool send_by_id(uint32_t msgid)
{
    switch (msgid) {
    case MAVLINK_MSG_ID_CAMERA_INFORMATION:       send_camera_information(); return true;
    case MAVLINK_MSG_ID_CAMERA_SETTINGS:          send_camera_settings(); return true;
    case MAVLINK_MSG_ID_STORAGE_INFORMATION:      send_storage_information(); return true;
    case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:    send_capture_status(); return true;
    case MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION: send_video_stream_information(); return true;
    case MAVLINK_MSG_ID_VIDEO_STREAM_STATUS:      send_video_stream_status(); return true;
    default: return false;
    }
}

/* ---- capture ----------------------------------------------------------- */

static struct interval *find_interval(uint32_t msgid);

/* Everything that talks to the streamer runs as a worker job: the "work" half
 * on the worker thread, the "done" half back on the main thread. */

static void image_work(struct job *j)
{
    j->ok = streamer_snapshot(&g.streamer, j->path);
}

static void image_done(struct job *j)
{
    LOG("image %s: %s", j->path, j->ok ? "ok" : "FAILED");

    mavlink_camera_image_captured_t ic;
    memset(&ic, 0, sizeof(ic));
    ic.time_boot_ms = boot_ms();
    const time_t utc = time(NULL);
    ic.time_utc = utc > 1600000000 ? (uint64_t)utc * 1000ull : 0;   /* 0 if the clock is unset */
    if (g.have_pos) {
        ic.lat = g.lat;
        ic.lon = g.lon;
        ic.alt = g.alt;
        ic.relative_alt = g.relative_alt;
    }
    ic.q[0] = 1;
    ic.capture_result = j->ok ? 1 : 0;
    if (j->ok) {
        g.image_count++;
        ic.image_index = g.image_count;
        strncpy(ic.file_url, j->path, sizeof(ic.file_url) - 1);
    } else {
        ic.image_index = -1;
    }
    mavlink_message_t msg;
    mavlink_msg_camera_image_captured_encode(g.sysid, g.cfg.compid, &msg, &ic);
    mavcam_send(&msg, DST_ALL);
    if (g.images_remaining == 0) {
        send_capture_status();
    }
}

static void take_image(void)
{
    struct job j = { .work = image_work, .done = image_done };
    snprintf(j.path, sizeof(j.path), "%s/IMG_%05d.jpg", g.cfg.media_dir, ++g.image_seq);
    worker_submit(&j);
}

static void set_recording(bool on)
{
    if (on && !g.recording) {
        g.record_start_ms = now_ms();
    }
    g.recording = on;
    if (!on) {
        g.record_elapsed_ms = 0;
    }
}

static void record_work(struct job *j)
{
    j->ok = streamer_record(&g.streamer, j->i != 0);
}

static void record_done(struct job *j)
{
    const bool start = j->i != 0;
    LOG("record %s%s", start ? "start" : "stop", j->ok ? "" : " FAILED");
    if (!j->ok) {
        send_ack_to(j->req_sys, j->req_comp, j->req_dst, j->command, MAV_RESULT_FAILED);
        return;
    }
    set_recording(start);
    if (start && j->f > 0) {
        /* param2: frequency at which to send CAMERA_CAPTURE_STATUS */
        struct interval *iv = find_interval(MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS);
        long ms = j->f < 0.001f ? 1000000 : (long)(1000.0f / j->f);
        iv->interval_ms = ms < 100 ? 100 : ms;
    }
    send_ack_to(j->req_sys, j->req_comp, j->req_dst, j->command, MAV_RESULT_ACCEPTED);
    send_capture_status();
}

static void zoom_work(struct job *j)
{
    j->ok = streamer_set_zoom(&g.streamer, j->i);
}

static void zoom_done(struct job *j)
{
    g.zoom_busy = false;
    if (j->ok) {
        g.zoom_level = j->i;
        g.last_props_ms = 0;   /* zoom changes the encoded resolution */
    } else {
        g.zoom_rate = 0;
    }
    if (j->command != 0) {
        send_ack_to(j->req_sys, j->req_comp, j->req_dst, j->command,
                    j->ok ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED);
    }
    if (j->ok) {
        send_camera_settings();
    }
}

static void props_work(struct job *j)
{
    j->ok = streamer_get_props(&g.streamer, &j->props);
}

static void props_done(struct job *j)
{
    g.props_busy = false;
    g.props = j->props;   /* fields the streamer did not report keep their old values */
}

static void record_poll_work(struct job *j)
{
    j->ok = streamer_record_status(&g.streamer, &j->rs);
}

static void record_poll_done(struct job *j)
{
    g.record_poll_busy = false;
    if (!j->ok) {
        return;
    }
    if (j->rs.active != g.recording) {
        LOG("streamer reports recording %s", j->rs.active ? "on" : "off");
        set_recording(j->rs.active);
        send_capture_status();
    }
    g.record_elapsed_ms = j->rs.active ? j->rs.elapsed_ms : 0;
}

/* ---- incoming commands ------------------------------------------------- */

static struct interval *find_interval(uint32_t msgid)
{
    for (size_t i = 0; i < sizeof(g.intervals) / sizeof(g.intervals[0]); i++) {
        if (g.intervals[i].msgid == msgid) {
            return &g.intervals[i];
        }
    }
    return NULL;
}

/* Command parameters are floats from the network: NaN, infinities and huge
 * values must be rejected before any conversion to an integer. */
static bool in_range(float v, float lo, float hi)
{
    return v >= lo && v <= hi;   /* false for NaN */
}

/* returns a MAV_RESULT, or -1 for "not a command we implement" */
static int handle_command(const mavlink_message_t *req, const mavlink_command_long_t *c)
{
    switch (c->command) {
    case MAV_CMD_REQUEST_MESSAGE: {
        if (!in_range(c->param1, 0, 16777215.0f)) {
            return -1;
        }
        const uint32_t id = (uint32_t)c->param1;
        if (id != MAVLINK_MSG_ID_CAMERA_INFORMATION && id != MAVLINK_MSG_ID_CAMERA_SETTINGS &&
            id != MAVLINK_MSG_ID_STORAGE_INFORMATION && id != MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS &&
            id != MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION && id != MAVLINK_MSG_ID_VIDEO_STREAM_STATUS) {
            return -1;
        }
        /* the protocol wants the ACK before the requested message */
        send_ack(req, c->command, MAV_RESULT_ACCEPTED);
        send_by_id(id);
        return -2;   /* already acknowledged */
    }

    /* pre-REQUEST_MESSAGE request commands, still sent by some ground stations */
    case MAV_CMD_REQUEST_CAMERA_INFORMATION:
    case MAV_CMD_REQUEST_CAMERA_SETTINGS:
    case MAV_CMD_REQUEST_STORAGE_INFORMATION:
    case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
    case MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
    case MAV_CMD_REQUEST_VIDEO_STREAM_STATUS: {
        send_ack(req, c->command, MAV_RESULT_ACCEPTED);
        switch (c->command) {
        case MAV_CMD_REQUEST_CAMERA_INFORMATION:       send_camera_information(); break;
        case MAV_CMD_REQUEST_CAMERA_SETTINGS:          send_camera_settings(); break;
        case MAV_CMD_REQUEST_STORAGE_INFORMATION:      send_storage_information(); break;
        case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:    send_capture_status(); break;
        case MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION: send_video_stream_information(); break;
        default:                                       send_video_stream_status(); break;
        }
        return -2;
    }

    case MAV_CMD_SET_MESSAGE_INTERVAL: {
        if (!in_range(c->param1, 0, 16777215.0f)) {
            return -1;
        }
        struct interval *iv = find_interval((uint32_t)c->param1);
        if (iv == NULL) {
            return -1;
        }
        const float us = c->param2;
        if (us != us) {
            return MAV_RESULT_DENIED;       /* NaN */
        }
        if (us <= 0) {
            iv->interval_ms = 0;            /* disabled / default: on request only */
        } else if (us >= 3.6e9f) {
            iv->interval_ms = 3600000;      /* an hour is as slow as it gets */
        } else {
            long ms = (long)(us / 1000.0f);
            iv->interval_ms = ms < 100 ? 100 : ms;   /* cap at 10 Hz */
        }
        return MAV_RESULT_ACCEPTED;
    }

    case MAV_CMD_SET_CAMERA_MODE:
        if (c->param2 != CAMERA_MODE_IMAGE && c->param2 != CAMERA_MODE_VIDEO) {
            return MAV_RESULT_DENIED;
        }
        g.mode = (uint8_t)c->param2;
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_IMAGE_START_CAPTURE: {
        const float interval = c->param2;
        if (!in_range(c->param3, 0, 1000000.0f) || !in_range(interval, 0, 86400.0f)) {
            return MAV_RESULT_DENIED;   /* also catches NaN and infinities */
        }
        const int count = (int)c->param3;
        if (count != 1 && !(interval > 0)) {
            return MAV_RESULT_DENIED;   /* multi-image capture needs an interval */
        }
        g.image_interval_s = interval > 0 ? interval : 0;
        g.images_remaining = count == 0 ? -1 : count;
        g.next_image_ms = now_ms();
        return MAV_RESULT_ACCEPTED;
    }

    case MAV_CMD_IMAGE_STOP_CAPTURE:
        g.images_remaining = 0;
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_VIDEO_START_CAPTURE:
    case MAV_CMD_VIDEO_STOP_CAPTURE: {
        const bool start = c->command == MAV_CMD_VIDEO_START_CAPTURE;
        if (c->param1 != 0 && c->param1 != 1) {
            return MAV_RESULT_DENIED;   /* only stream 1 (or 0 = all) exists */
        }
        struct job j = { .work = record_work, .done = record_done, .i = start, .f = c->param2,
                         .req_sys = req->sysid, .req_comp = req->compid, .req_dst = g.reply_dst,
                         .command = c->command };
        if (!worker_submit(&j)) {
            return MAV_RESULT_TEMPORARILY_REJECTED;
        }
        return -2;   /* acknowledged when the streamer has answered */
    }

    case MAV_CMD_SET_CAMERA_ZOOM: {
        const int levels = streamer_zoom_levels(&g.streamer);
        if (levels <= 1) {
            return MAV_RESULT_UNSUPPORTED;
        }
        int level;
        if (c->param1 == ZOOM_TYPE_RANGE) {
            if (!(c->param2 >= 0 && c->param2 <= 100)) {
                return MAV_RESULT_DENIED;
            }
            level = (int)lroundf(c->param2 / 100.0f * (levels - 1));
        } else if (c->param1 == ZOOM_TYPE_STEP) {
            level = g.zoom_level + (c->param2 > 0 ? 1 : c->param2 < 0 ? -1 : 0);
        } else if (c->param1 == ZOOM_TYPE_CONTINUOUS) {
            /* zoom in / out one preset at a time until told to stop */
            g.zoom_rate = c->param2 > 0 ? 1 : c->param2 < 0 ? -1 : 0;
            g.last_zoom_step_ms = 0;
            return MAV_RESULT_ACCEPTED;
        } else {
            return MAV_RESULT_UNSUPPORTED;   /* focal length */
        }
        g.zoom_rate = 0;
        if (level < 0) {
            level = 0;
        }
        if (level > levels - 1) {
            level = levels - 1;
        }
        if (level == g.zoom_level && !g.zoom_busy) {
            return MAV_RESULT_ACCEPTED;
        }
        struct job j = { .work = zoom_work, .done = zoom_done, .i = level,
                         .req_sys = req->sysid, .req_comp = req->compid, .req_dst = g.reply_dst,
                         .command = c->command };
        if (!worker_submit(&j)) {
            return MAV_RESULT_TEMPORARILY_REJECTED;
        }
        g.zoom_busy = true;
        return -2;
    }

    default:
        return -1;
    }
}

static void handle_message(const mavlink_message_t *msg, int src)
{
    g.reply_dst = src;
    if (params_handle_message(msg, src) || ftp_handle_message(msg, src)) {
        return;
    }
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(msg, &hb);
        /* a real autopilot on the FC link: not a GCS, not a peripheral */
        if (src != DST_FC || hb.autopilot == MAV_AUTOPILOT_INVALID || hb.type == MAV_TYPE_GCS) {
            break;
        }
        if (!g.fc_seen) {
            LOG("autopilot found: sysid %u compid %u", msg->sysid, msg->compid);
        }
        g.fc_seen = true;
        g.fc_sysid = msg->sysid;
        g.fc_compid = msg->compid;
        if (!g.sysid_locked && g.sysid != msg->sysid) {
            LOG("adopting system ID %u", msg->sysid);
            g.sysid = msg->sysid;
        }
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        if (src != DST_FC || !g.fc_seen || msg->sysid != g.fc_sysid) {
            break;
        }
        mavlink_global_position_int_t p;
        mavlink_msg_global_position_int_decode(msg, &p);
        g.lat = p.lat;
        g.lon = p.lon;
        g.alt = p.alt;
        g.relative_alt = p.relative_alt;
        g.have_pos = true;
        g.last_pos_ms = now_ms();
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_INT:
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t c;
        if (msg->msgid == MAVLINK_MSG_ID_COMMAND_INT) {
            /* same commands with the last three parameters as scaled integers */
            mavlink_command_int_t ci;
            mavlink_msg_command_int_decode(msg, &ci);
            c = (mavlink_command_long_t) {
                .param1 = ci.param1, .param2 = ci.param2, .param3 = ci.param3, .param4 = ci.param4,
                .param5 = (float)ci.x, .param6 = (float)ci.y, .param7 = ci.z,
                .command = ci.command, .target_system = ci.target_system,
                .target_component = ci.target_component,
            };
        } else {
            mavlink_msg_command_long_decode(msg, &c);
        }
        if (c.target_system != 0 && c.target_system != g.sysid) {
            break;
        }
        const bool to_us = c.target_component == g.cfg.compid;
        if (!to_us && c.target_component != 0) {
            break;
        }
        const int result = handle_command(msg, &c);
        if (result >= 0) {
            send_ack(msg, c.command, (uint8_t)result);
            if (result == MAV_RESULT_ACCEPTED) {
                /* let the FC and GCS see state changes without waiting for a poll */
                switch (c.command) {
                case MAV_CMD_IMAGE_START_CAPTURE:
                case MAV_CMD_IMAGE_STOP_CAPTURE:
                    send_capture_status();
                    break;
                case MAV_CMD_SET_CAMERA_MODE:
                    send_camera_settings();
                    break;
                default:
                    break;
                }
            }
        } else if (result == -1 && to_us) {
            /* only refuse commands addressed to us; broadcasts may be for others */
            send_ack(msg, c.command, MAV_RESULT_UNSUPPORTED);
        }
        break;
    }

    default:
        break;
    }
}

/* ---- periodic work ----------------------------------------------------- */

static void request_position_stream(void)
{
    /* the FC may not stream telemetry on a camera link, so ask for what we use */
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(g.sysid, g.cfg.compid, &msg, g.fc_sysid, g.fc_compid,
                                  MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                  MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000000, 0, 0, 0, 0, 0);
    mavcam_send(&msg, DST_FC);
}

static void periodic(void)
{
    const long now = now_ms();

    /* When adopting the autopilot's system ID, stay quiet until it has been
     * seen: announcing under a guessed ID would show up as a phantom system,
     * and some ground software (MAVSDK) never finishes discovery if a camera's
     * heartbeat reaches it before the autopilot's. Give up waiting after a few
     * seconds so the camera still works with no autopilot attached. */
    const bool waiting_for_fc = !g.sysid_locked && !g.fc_seen && now - g.boot_ms < FC_WAIT_MS;
    if (!waiting_for_fc && now - g.last_heartbeat_ms >= 1000) {
        g.last_heartbeat_ms = now;
        send_heartbeat();
    }

    /* at most one poll of each kind in the queue, so a slow streamer cannot back it up */
    if (!g.props_busy && (g.last_props_ms == 0 || now - g.last_props_ms >= 10000)) {
        struct job j = { .work = props_work, .done = props_done, .props = g.props };
        if (worker_submit(&j)) {
            g.last_props_ms = now;
            g.props_busy = true;
        }
    }

    if (!g.record_poll_busy && now - g.last_record_poll_ms >= 1000) {
        struct job j = { .work = record_poll_work, .done = record_poll_done };
        if (worker_submit(&j)) {
            g.last_record_poll_ms = now;
            g.record_poll_busy = true;
        }
    }

    if (g.zoom_rate != 0 && !g.zoom_busy && now - g.last_zoom_step_ms >= 1000) {
        const int level = g.zoom_level + g.zoom_rate;
        if (level < 0 || level >= streamer_zoom_levels(&g.streamer)) {
            g.zoom_rate = 0;   /* reached the end of the range */
        } else {
            struct job j = { .work = zoom_work, .done = zoom_done, .i = level };
            if (worker_submit(&j)) {
                g.last_zoom_step_ms = now;
                g.zoom_busy = true;
            }
        }
    }

    if (g.cfg.geotag && g.fc_seen && now - g.last_pos_ms > 5000 && now - g.last_pos_req_ms > 5000) {
        g.last_pos_req_ms = now;
        g.have_pos = g.have_pos && now - g.last_pos_ms < 10000;
        request_position_stream();
    }

    if (g.images_remaining != 0 && now >= g.next_image_ms) {
        take_image();
        if (g.images_remaining > 0) {
            g.images_remaining--;
        }
        g.next_image_ms = now + (long)(g.image_interval_s * 1000.0f);
    }

    for (size_t i = 0; i < sizeof(g.intervals) / sizeof(g.intervals[0]); i++) {
        struct interval *iv = &g.intervals[i];
        if (iv->interval_ms > 0 && now - iv->last_ms >= iv->interval_ms) {
            iv->last_ms = now;
            send_by_id(iv->msgid);
        }
    }
}

/* ---- routing ----------------------------------------------------------- */

/* Read one link. Frames are handled locally and passed on to the other side,
 * so the flight controller and the ground keep talking through us. */
static void read_link(struct link *l, uint8_t chan, int src)
{
    uint8_t buf[512];
    ssize_t n;
    mavlink_message_t msg;
    mavlink_status_t status;
    while ((n = link_read(l, buf, sizeof(buf))) > 0) {
        for (ssize_t i = 0; i < n; i++) {
            const uint8_t framing = mavlink_frame_char(chan, buf[i], &msg, &status);
            if (framing == MAVLINK_FRAMING_INCOMPLETE) {
                continue;
            }
            const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msg.msgid);
            if (framing == MAVLINK_FRAMING_BAD_CRC && entry != NULL) {
                continue;   /* corrupt */
            }
            /* a "bad CRC" on a message we have no definition for is just an
             * unknown message (e.g. ArduPilot-specific): pass it on untouched */
            const bool known = framing == MAVLINK_FRAMING_OK;
            if (msg.sysid == g.sysid && msg.compid == g.cfg.compid) {
                continue;   /* our own message coming back */
            }

            bool only_for_us = false;
            if (known && entry != NULL &&
                (entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM) &&
                (entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT)) {
                const uint8_t *payload = (const uint8_t *)_MAV_PAYLOAD(&msg);
                only_for_us = payload[entry->target_system_ofs] == g.sysid &&
                              payload[entry->target_component_ofs] == g.cfg.compid;
            }
            if (known) {
                handle_message(&msg, src);
            }
            if (!only_for_us) {
                mavcam_send(&msg, src == DST_FC ? DST_GROUND : DST_FC);
            }
        }
    }
}

/* ---- setup ------------------------------------------------------------- */

static bool ftp_busy;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s [options]\n"
        "  -l, --link SPEC        FC link: /dev/ttyS2:115200 | udp:HOST:PORT | udpin:PORT | tcp:HOST:PORT\n"
        "                         (default /dev/ttyS2:115200)\n"
        "  -s, --streamer TYPE    waybeam | majestic (default waybeam)\n"
        "  -H, --http HOST:PORT   streamer HTTP API (default 127.0.0.1:80)\n"
        "  -d, --media-dir DIR    where photos are written and storage is measured (default /mnt/mmcblk0p1)\n"
        "  -u, --stream-uri URI   URI advertised to the ground station (default 5600)\n"
        "  -t, --stream-type N    VIDEO_STREAM_TYPE: 0 RTSP, 1 RTP/UDP, 2 TCP MPEG, 3 MPEG-TS (default 1)\n"
        "      --stream-name NAME stream name (default \"OpenIPC\")\n"
        "      --sysid N          MAVLink system ID (default: adopt the autopilot's)\n"
        "      --compid N         MAVLink component ID (default 100, MAV_COMP_ID_CAMERA)\n"
        "      --vendor NAME      vendor name in CAMERA_INFORMATION (default \"OpenIPC\")\n"
        "      --model NAME       model name in CAMERA_INFORMATION (default \"SSC338Q\")\n"
        "      --hfov DEG         horizontal field of view to report (default 0 = unknown)\n"
        "      --width N --height N --fps N --bitrate KBPS\n"
        "                         fallbacks used until the streamer reports its own values\n"
        "      --no-geotag        do not request position from the autopilot\n"
        "  -g, --ground-out H:P   also act as the telemetry forwarder: pass MAVLink between the FC\n"
        "                         link and this UDP address (e.g. 127.0.0.1:14550 for wfb-ng)\n"
        "      --ground-in PORT   UDP port to listen on for the ground side (e.g. 14551)\n"
        "      --ftp-rate BPS     ceiling for file-transfer data in bytes per second (default 16000)\n"
        "      --camdef-uri URI   advertise this address for the camera definition file instead of\n"
        "                         serving it over MAVLink FTP (e.g. an http:// copy)\n"
        "      --dump-camdef      print the camera definition XML and exit\n"
        "      --signing-key-file FILE\n"
        "                         sign this camera's messages with the MAVLink 2 key in FILE\n"
        "                         (32 raw bytes, or 64 hex digits); needed if the flight\n"
        "                         controller has signing enabled\n"
        "      --exec-param NAME:GETCMD:SETCMD\n"
        "                         extra integer setting backed by shell commands; SETCMD is run\n"
        "                         with the new value appended. May be repeated\n"
        "  -v, --verbose\n",
        argv0);
}

int main(int argc, char **argv)
{
    g.cfg = (struct config) {
        .link_spec = "/dev/ttyS2:115200",
        .sysid = 0,
        .compid = MAV_COMP_ID_CAMERA,
        .vendor = "OpenIPC",
        .model = "SSC338Q",
        .media_dir = "/mnt/mmcblk0p1",
        .stream_uri = "5600",
        .stream_name = "OpenIPC",
        .stream_type = VIDEO_STREAM_TYPE_RTPUDP,
        .geotag = true,
        .ftp_rate = 16000,
    };
    g.streamer = (struct streamer) { .type = STREAMER_WAYBEAM, .host = "127.0.0.1", .port = 80, .timeout_ms = 1000 };
    g.props = (struct stream_props) { .width = 1920, .height = 1080, .fps = 60, .bitrate_kbps = 8192, .h265 = true };
    g.mode = CAMERA_MODE_VIDEO;

    enum { O_SYSID = 256, O_COMPID, O_VENDOR, O_MODEL, O_HFOV, O_WIDTH, O_HEIGHT, O_FPS, O_BITRATE, O_NOGEO, O_SNAME, O_GIN, O_EXEC, O_FTPRATE, O_CAMDEF_URI, O_DUMP, O_SIGNKEY };
    static const struct option opts[] = {
        { "link", required_argument, 0, 'l' }, { "streamer", required_argument, 0, 's' },
        { "http", required_argument, 0, 'H' }, { "media-dir", required_argument, 0, 'd' },
        { "stream-uri", required_argument, 0, 'u' }, { "stream-type", required_argument, 0, 't' },
        { "stream-name", required_argument, 0, O_SNAME },
        { "sysid", required_argument, 0, O_SYSID }, { "compid", required_argument, 0, O_COMPID },
        { "vendor", required_argument, 0, O_VENDOR }, { "model", required_argument, 0, O_MODEL },
        { "hfov", required_argument, 0, O_HFOV }, { "width", required_argument, 0, O_WIDTH },
        { "height", required_argument, 0, O_HEIGHT }, { "fps", required_argument, 0, O_FPS },
        { "bitrate", required_argument, 0, O_BITRATE }, { "no-geotag", no_argument, 0, O_NOGEO },
        { "ground-out", required_argument, 0, 'g' }, { "ground-in", required_argument, 0, O_GIN },
        { "exec-param", required_argument, 0, O_EXEC }, { "ftp-rate", required_argument, 0, O_FTPRATE },
        { "camdef-uri", required_argument, 0, O_CAMDEF_URI }, { "dump-camdef", no_argument, 0, O_DUMP },
        { "signing-key-file", required_argument, 0, O_SIGNKEY },
        { "verbose", no_argument, 0, 'v' }, { "help", no_argument, 0, 'h' }, { 0, 0, 0, 0 },
    };
    int o;
    while ((o = getopt_long(argc, argv, "l:s:H:d:u:t:g:vh", opts, NULL)) != -1) {
        switch (o) {
        case 'l': g.cfg.link_spec = optarg; break;
        case 's':
            if (strcmp(optarg, "waybeam") == 0) {
                g.streamer.type = STREAMER_WAYBEAM;
            } else if (strcmp(optarg, "majestic") == 0) {
                g.streamer.type = STREAMER_MAJESTIC;
            } else {
                fprintf(stderr, "mavcam: unknown streamer '%s'\n", optarg);
                return 2;
            }
            break;
        case 'H': {
            const char *colon = strrchr(optarg, ':');
            const size_t n = colon ? (size_t)(colon - optarg) : strlen(optarg);
            if (n == 0 || n >= sizeof(g.streamer.host)) {
                fprintf(stderr, "mavcam: bad --http value\n");
                return 2;
            }
            memcpy(g.streamer.host, optarg, n);
            g.streamer.host[n] = 0;
            g.streamer.port = colon ? atoi(colon + 1) : 80;
            break;
        }
        case 'd': g.cfg.media_dir = optarg; break;
        case 'u': g.cfg.stream_uri = optarg; break;
        case 't': g.cfg.stream_type = atoi(optarg); break;
        case O_SNAME: g.cfg.stream_name = optarg; break;
        case O_SYSID: g.cfg.sysid = atoi(optarg); break;
        case O_COMPID: g.cfg.compid = atoi(optarg); break;
        case O_VENDOR: g.cfg.vendor = optarg; break;
        case O_MODEL: g.cfg.model = optarg; break;
        case O_HFOV: g.cfg.hfov_deg = atoi(optarg); break;
        case O_WIDTH: g.props.width = (uint16_t)atoi(optarg); break;
        case O_HEIGHT: g.props.height = (uint16_t)atoi(optarg); break;
        case O_FPS: g.props.fps = (float)atof(optarg); break;
        case O_BITRATE: g.props.bitrate_kbps = (uint32_t)atol(optarg); break;
        case O_NOGEO: g.cfg.geotag = false; break;
        case 'g': {
            char spec[160];
            snprintf(spec, sizeof(spec), "udp:%s", optarg);
            if (!link_parse(&g.ground_tx, spec)) {
                fprintf(stderr, "mavcam: bad --ground-out value '%s'\n", optarg);
                return 2;
            }
            g.have_ground_tx = true;
            break;
        }
        case O_GIN: {
            char spec[32];
            snprintf(spec, sizeof(spec), "udpin:%s", optarg);
            if (!link_parse(&g.ground_rx, spec)) {
                fprintf(stderr, "mavcam: bad --ground-in value '%s'\n", optarg);
                return 2;
            }
            g.have_ground_rx = true;
            break;
        }
        case O_CAMDEF_URI: g.cfg.camdef_uri = optarg; break;
        case O_SIGNKEY: g.cfg.signing_key_file = optarg; break;
        case O_DUMP: g.cfg.dump_camdef = true; break;
        case O_FTPRATE:
            g.cfg.ftp_rate = atol(optarg);
            if (g.cfg.ftp_rate < 1000) {
                fprintf(stderr, "mavcam: --ftp-rate must be at least 1000 bytes per second\n");
                return 2;
            }
            break;
        case O_EXEC:
            if (!params_add_exec(optarg)) {
                fprintf(stderr, "mavcam: bad --exec-param '%s' (want NAME:GETCMD:SETCMD)\n", optarg);
                return 2;
            }
            break;
        case 'v': g.cfg.verbose = true; break;
        default: usage(argv[0]); return o == 'h' ? 0 : 2;
        }
    }
    if (g.cfg.sysid < 0 || g.cfg.sysid > 255 || g.cfg.compid < 1 || g.cfg.compid > 255) {
        fprintf(stderr, "mavcam: --sysid must be 0-255 and --compid 1-255\n");
        return 2;
    }
    if (!link_parse(&g.link, g.cfg.link_spec)) {
        fprintf(stderr, "mavcam: bad --link value '%s'\n", g.cfg.link_spec);
        return 2;
    }

    if (g.cfg.signing_key_file != NULL && !load_signing_key(g.cfg.signing_key_file)) {
        fprintf(stderr, "mavcam: cannot read a 32-byte key from '%s'\n", g.cfg.signing_key_file);
        return 2;
    }
    g.sysid_locked = g.cfg.sysid != 0;
    g.sysid = g.sysid_locked ? (uint8_t)g.cfg.sysid : 1;
    g.boot_ms = now_ms();
    g.intervals[0].msgid = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
    g.intervals[1].msgid = MAVLINK_MSG_ID_STORAGE_INFORMATION;
    g.intervals[2].msgid = MAVLINK_MSG_ID_VIDEO_STREAM_STATUS;
    g.intervals[3].msgid = MAVLINK_MSG_ID_CAMERA_SETTINGS;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    LOG("link %s, streamer %s at %s:%d, media %s", g.cfg.link_spec,
        g.streamer.type == STREAMER_WAYBEAM ? "waybeam" : "majestic",
        g.streamer.host, g.streamer.port, g.cfg.media_dir);

    if (!worker_start()) {
        fprintf(stderr, "mavcam: could not start the worker thread\n");
        return 1;
    }
    params_init();
    if (g.cfg.dump_camdef) {
        size_t len;
        const char *xml = params_camdef(&len);
        fwrite(xml, 1, len, stdout);
        return 0;
    }

    bool was_open = false;
    while (!g_stop) {
        const long now = now_ms();
        const bool open = link_ensure_open(&g.link, now);
        if (open != was_open) {
            LOG("link %s", open ? "open" : "closed, retrying");
            was_open = open;
        }
        if (!open) {
            usleep(100 * 1000);
            continue;
        }
        if (g.have_ground_tx) {
            link_ensure_open(&g.ground_tx, now);
        }
        if (g.have_ground_rx) {
            link_ensure_open(&g.ground_rx, now);
        }

        struct pollfd p[4];
        int np = 0;
        p[np++] = (struct pollfd) { .fd = worker_fd(), .events = POLLIN };
        /* wake when a backed-up port can take more */
        p[np++] = (struct pollfd) { .fd = g.link.fd,
                                    .events = (short)(POLLIN | (link_backlog(&g.link) ? POLLOUT : 0)) };
        if (g.have_ground_tx && g.ground_tx.fd >= 0) {
            p[np++] = (struct pollfd) { .fd = g.ground_tx.fd, .events = POLLIN };
        }
        if (g.have_ground_rx && g.ground_rx.fd >= 0) {
            p[np++] = (struct pollfd) { .fd = g.ground_rx.fd, .events = POLLIN };
        }
        /* spin faster while a file transfer has data queued */
        if (poll(p, (nfds_t)np, ftp_busy ? 2 : 50) > 0) {
            read_link(&g.link, MAVLINK_COMM_0, DST_FC);
            if (g.have_ground_tx) {
                read_link(&g.ground_tx, MAVLINK_COMM_1, DST_GROUND);
            }
            if (g.have_ground_rx) {
                read_link(&g.ground_rx, MAVLINK_COMM_1, DST_GROUND);
            }
        }
        worker_dispatch();
        link_flush(&g.link);
        if (g.link.fd >= 0) {
            periodic();
            ftp_busy = ftp_periodic();
        }
    }
    return 0;
}
