#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Adapter between camera-protocol operations and the OpenIPC streamer's
 * localhost HTTP API. Two streamers are supported:
 *   waybeam  - open source (OpenIPC/waybeam_venc)
 *   majestic - OpenIPC's default streamer
 */
enum streamer_type {
    STREAMER_WAYBEAM = 0,
    STREAMER_MAJESTIC = 1,
};

struct streamer {
    enum streamer_type type;
    char host[64];
    int port;
    int timeout_ms;
};

struct stream_props {
    uint16_t width, height;
    float fps;
    uint32_t bitrate_kbps;
    bool h265;
};

struct record_status {
    bool active;
    uint32_t elapsed_ms;   /* 0 if the streamer does not report it */
};

/* read / write one streamer config field as text (e.g. "video0.bitrate") */
bool streamer_get_field(const struct streamer *s, const char *field, char *out, size_t outlen);
bool streamer_set_field(const struct streamer *s, const char *field, const char *value);

/* number of digital zoom steps, including "no zoom" (1 means zoom unsupported) */
int streamer_zoom_levels(const struct streamer *s);

/* fills only the fields the streamer reports; others are left untouched.
 * returns false if the streamer could not be reached at all */
bool streamer_get_props(const struct streamer *s, struct stream_props *p);

bool streamer_record(const struct streamer *s, bool start);
bool streamer_record_status(const struct streamer *s, struct record_status *st);

/* fetch one JPEG and write it to path */
bool streamer_snapshot(const struct streamer *s, const char *path);

/* level 0 = no zoom .. streamer_zoom_levels()-1 = maximum */
bool streamer_set_zoom(const struct streamer *s, int level);
