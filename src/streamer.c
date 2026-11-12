#include "streamer.h"
#include "http.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define MAX_JSON   (64 * 1024)
#define MAX_JPEG   (8 * 1024 * 1024)

static const char *const waybeam_zoom[] = {
    "off", "zoom-1.25x", "zoom-1.50x", "zoom-1.75x", "zoom-2x", "zoom-3x", "zoom-4x",
};
#define WAYBEAM_ZOOM_LEVELS ((int)(sizeof(waybeam_zoom) / sizeof(waybeam_zoom[0])))

/* GET a small text/JSON resource. Returns malloc'd body on HTTP 200, else NULL */
static char *get_text(const struct streamer *s, const char *path)
{
    int status;
    char *body;
    if (http_get(s->host, s->port, path, s->timeout_ms, MAX_JSON, &status, &body) < 0) {
        return NULL;
    }
    if (status != 200) {
        free(body);
        return NULL;
    }
    return body;
}

/* true if the request returned HTTP 200 and the body does not say "ok":false */
static bool get_ok(const struct streamer *s, const char *path)
{
    char *body = get_text(s, path);
    if (body == NULL) {
        return false;
    }
    const bool ok = strstr(body, "\"ok\":false") == NULL && strstr(body, "\"ok\": false") == NULL;
    free(body);
    return ok;
}

/* Copy the scalar after "key": in a JSON body into out (quotes stripped).
 * Not a JSON parser: enough for the flat replies these streamers give. */
static bool json_scalar(const char *body, const char *key, char *out, size_t outlen)
{
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(body, pat);
    if (p == NULL) {
        return false;
    }
    p += strlen(pat);
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    if (*p != ':') {
        return false;
    }
    p++;
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    size_t n = 0;
    if (*p == '"') {
        for (p++; *p && *p != '"' && n + 1 < outlen; p++) {
            out[n++] = *p;
        }
    } else {
        for (; *p && *p != ',' && *p != '}' && !isspace((unsigned char)*p) && n + 1 < outlen; p++) {
            out[n++] = *p;
        }
    }
    out[n] = 0;
    return n > 0;
}

/* read one config field as a string. waybeam: /api/v1/get?<field>,
 * majestic: /api/v1/get?key=<field>. Falls back to the bare body. */
bool streamer_get_field(const struct streamer *s, const char *field, char *out, size_t outlen)
{
    char path[160];
    snprintf(path, sizeof(path),
             s->type == STREAMER_MAJESTIC ? "/api/v1/get?key=%s" : "/api/v1/get?%s", field);
    char *body = get_text(s, path);
    if (body == NULL) {
        return false;
    }
    bool ok = json_scalar(body, "value", out, outlen);
    if (!ok && body[0] != '{') {
        /* plain-text reply */
        size_t n = 0;
        for (const char *p = body; *p && n + 1 < outlen; p++) {
            if (*p != '"' && *p != '\r' && *p != '\n') {
                out[n++] = *p;
            }
        }
        out[n] = 0;
        ok = n > 0;
    }
    free(body);
    return ok;
}

static bool parse_size(const char *v, uint16_t *w, uint16_t *h)
{
    unsigned a, b;
    if (sscanf(v, "%ux%u", &a, &b) == 2 && a > 0 && b > 0 && a < 65536 && b < 65536) {
        *w = (uint16_t)a;
        *h = (uint16_t)b;
        return true;
    }
    static const struct { const char *name; uint16_t w, h; } presets[] = {
        { "720p", 1280, 720 }, { "1080p", 1920, 1080 }, { "1440p", 2560, 1440 }, { "4k", 3840, 2160 },
    };
    for (size_t i = 0; i < sizeof(presets) / sizeof(presets[0]); i++) {
        if (strcasecmp(v, presets[i].name) == 0) {
            *w = presets[i].w;
            *h = presets[i].h;
            return true;
        }
    }
    return false;   /* e.g. "auto": keep the configured default */
}

bool streamer_set_field(const struct streamer *s, const char *field, const char *value)
{
    /* field names and values here are ours or validated numbers/options: no escaping needed */
    char path[192];
    const int n = snprintf(path, sizeof(path), "/api/v1/set?%s=%s", field, value);
    return n > 0 && n < (int)sizeof(path) && get_ok(s, path);
}

int streamer_zoom_levels(const struct streamer *s)
{
    return s->type == STREAMER_WAYBEAM ? WAYBEAM_ZOOM_LEVELS : 1;
}

bool streamer_get_props(const struct streamer *s, struct stream_props *p)
{
    char v[64];
    bool reached = false;
    if (streamer_get_field(s, "video0.size", v, sizeof(v))) {
        reached = true;
        if (!parse_size(v, &p->width, &p->height) && s->type == STREAMER_WAYBEAM) {
            /* "auto": the sensor's native size. waybeam reports what the
             * pipeline is actually using under runtime.active_precrop. */
            char *cfg = get_text(s, "/api/v1/config");
            const char *crop = cfg != NULL ? strstr(cfg, "\"active_precrop\"") : NULL;
            char w[16], h[16];
            if (crop != NULL && json_scalar(crop, "w", w, sizeof(w)) && json_scalar(crop, "h", h, sizeof(h))) {
                const long wi = atol(w), hi = atol(h);
                if (wi > 0 && wi < 65536 && hi > 0 && hi < 65536) {
                    p->width = (uint16_t)wi;
                    p->height = (uint16_t)hi;
                }
            }
            free(cfg);
        }
    }
    if (streamer_get_field(s, "video0.fps", v, sizeof(v))) {
        reached = true;
        const float f = (float)atof(v);
        if (f > 0) {
            p->fps = f;
        }
    }
    if (streamer_get_field(s, "video0.bitrate", v, sizeof(v))) {
        reached = true;
        const long b = atol(v);
        if (b > 0) {
            p->bitrate_kbps = (uint32_t)b;
        }
    }
    if (s->type == STREAMER_WAYBEAM) {
        p->h265 = true;   /* waybeam is HEVC only */
    } else if (streamer_get_field(s, "video0.codec", v, sizeof(v))) {
        p->h265 = strcasecmp(v, "h265") == 0 || strcasecmp(v, "hevc") == 0;
    }
    return reached;
}

bool streamer_record(const struct streamer *s, bool start)
{
    if (s->type == STREAMER_WAYBEAM) {
        return get_ok(s, start ? "/api/v1/record/start" : "/api/v1/record/stop");
    }
    /* majestic has no dedicated start/stop call; toggle the recorder's enable key */
    return get_ok(s, start ? "/api/v1/set?records.enabled=true" : "/api/v1/set?records.enabled=false");
}

bool streamer_record_status(const struct streamer *s, struct record_status *st)
{
    char v[32];
    st->elapsed_ms = 0;
    if (s->type == STREAMER_WAYBEAM) {
        char *body = get_text(s, "/api/v1/record/status");
        if (body == NULL) {
            return false;
        }
        const bool got = json_scalar(body, "active", v, sizeof(v));
        if (got) {
            st->active = strcmp(v, "true") == 0;
            char e[32];
            if (st->active && json_scalar(body, "elapsed_ms", e, sizeof(e))) {
                st->elapsed_ms = (uint32_t)strtoul(e, NULL, 10);
            }
        }
        free(body);
        return got;
    }
    /* majestic reports its recorder in Prometheus text form:
     *   records_enabled 1        records.enabled is on
     *   records_stood_down 0     1 while deliberately paused */
    char *body = get_text(s, "/metrics/records");
    if (body != NULL) {
        long enabled = -1, stood_down = 0;
        for (const char *line = body; line != NULL && *line; ) {
            if (strncmp(line, "records_enabled ", 16) == 0) {
                enabled = atol(line + 16);
            } else if (strncmp(line, "records_stood_down ", 19) == 0) {
                stood_down = atol(line + 19);
            }
            line = strchr(line, '\n');
            if (line != NULL) {
                line++;
            }
        }
        free(body);
        if (enabled >= 0) {
            st->active = enabled == 1 && stood_down == 0;
            return true;
        }
    }
    /* older builds: fall back to the config key */
    if (!streamer_get_field(s, "records.enabled", v, sizeof(v))) {
        return false;
    }
    st->active = strcmp(v, "true") == 0;
    return true;
}

bool streamer_snapshot(const struct streamer *s, const char *path)
{
    int status;
    char *body;
    const char *url = s->type == STREAMER_WAYBEAM ? "/api/v1/snapshot.jpg" : "/image.jpg";
    /* a snapshot can take longer than a config call */
    const long len = http_get(s->host, s->port, url, s->timeout_ms * 3, MAX_JPEG, &status, &body);
    if (len < 0) {
        return false;
    }
    bool ok = false;
    if (status == 200 && len > 2 && (unsigned char)body[0] == 0xFF && (unsigned char)body[1] == 0xD8) {
        FILE *f = fopen(path, "wb");
        if (f != NULL) {
            ok = fwrite(body, 1, (size_t)len, f) == (size_t)len;
            ok = fclose(f) == 0 && ok;
            if (!ok) {
                remove(path);
            }
        }
    }
    free(body);
    return ok;
}

bool streamer_set_zoom(const struct streamer *s, int level)
{
    if (s->type != STREAMER_WAYBEAM || level < 0 || level >= WAYBEAM_ZOOM_LEVELS) {
        return false;
    }
    char path[96];
    snprintf(path, sizeof(path), "/api/v1/set?video0.framing=%s", waybeam_zoom[level]);
    return get_ok(s, path);
}
