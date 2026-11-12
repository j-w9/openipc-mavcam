/*
 * Camera settings over the MAVLink extended parameter protocol (PARAM_EXT),
 * and the camera definition XML that tells a ground station what they are.
 */
#include "mavcam.h"
#include "worker.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_PARAMS 24
#define MAX_OPTIONS 8

struct cam_param {
    char name[17];
    uint8_t type;                 /* MAV_PARAM_EXT_TYPE: UINT8 (bool), UINT32, INT32 or REAL32 */
    const char *desc;
    double min, max;
    const char *field;            /* streamer config field, or NULL for exec params */
    const char *options[MAX_OPTIONS];   /* enum: value i <-> options[i] as the field's text */
    const char *labels[MAX_OPTIONS];    /* names shown to the user; NULL = same as options */
    int noptions;
    char *get_cmd, *set_cmd;      /* exec params */
    double value;                 /* last known value */
    bool known;
};

static struct cam_param params[MAX_PARAMS];
static int nparams;
static char *camdef;
static size_t camdef_len;

static const struct cam_param waybeam_params[] = {
    { .name = "BITRATE",  .type = MAV_PARAM_EXT_TYPE_UINT32, .desc = "Video bitrate (kbps)",
      .min = 256, .max = 40000, .field = "video0.bitrate" },
    { .name = "FPS",      .type = MAV_PARAM_EXT_TYPE_UINT32, .desc = "Frame rate (Hz)",
      .min = 1, .max = 144, .field = "video0.fps" },
    { .name = "GOP",      .type = MAV_PARAM_EXT_TYPE_REAL32, .desc = "Keyframe interval (s), 0 = all keyframes",
      .min = 0, .max = 10, .field = "video0.gop_size" },
    { .name = "RES",      .type = MAV_PARAM_EXT_TYPE_UINT32, .desc = "Encode resolution (restarts the video pipeline)",
      /* waybeam reports the size as WxH whatever form it was set in */
      .field = "video0.size", .options = { "auto", "1280x720", "1920x1080" },
      .labels = { "auto", "720p", "1080p" }, .noptions = 3 },
    { .name = "QP_DELTA", .type = MAV_PARAM_EXT_TYPE_INT32,  .desc = "Keyframe QP relative to other frames",
      .min = -12, .max = 12, .field = "video0.qp_delta" },
    { .name = "ROI_QP",   .type = MAV_PARAM_EXT_TYPE_INT32,  .desc = "Centre-priority QP offset, 0 = off",
      .min = -20, .max = 20, .field = "fpv.roi_qp" },
    { .name = "GAIN_MAX", .type = MAV_PARAM_EXT_TYPE_UINT32, .desc = "Auto-exposure gain ceiling, 0 = sensor default",
      .min = 0, .max = 1000000, .field = "isp.gain_max" },
    { .name = "AWB_MODE", .type = MAV_PARAM_EXT_TYPE_UINT32, .desc = "White balance mode",
      .field = "isp.awb_mode", .options = { "auto", "ct_manual" }, .noptions = 2 },
    { .name = "AWB_CT",   .type = MAV_PARAM_EXT_TYPE_UINT32, .desc = "Manual white balance colour temperature (K)",
      .min = 1500, .max = 15000, .field = "isp.awb_ct" },
    { .name = "MIRROR",   .type = MAV_PARAM_EXT_TYPE_UINT8,  .desc = "Mirror image (restarts the video pipeline)",
      .min = 0, .max = 1, .field = "image.mirror" },
    { .name = "FLIP",     .type = MAV_PARAM_EXT_TYPE_UINT8,  .desc = "Flip image (restarts the video pipeline)",
      .min = 0, .max = 1, .field = "image.flip" },
};

static const struct cam_param majestic_params[] = {
    { .name = "BITRATE",  .type = MAV_PARAM_EXT_TYPE_UINT32, .desc = "Video bitrate (kbps)",
      .min = 256, .max = 40000, .field = "video0.bitrate" },
    { .name = "FPS",      .type = MAV_PARAM_EXT_TYPE_UINT32, .desc = "Frame rate (Hz)",
      .min = 1, .max = 120, .field = "video0.fps" },
    { .name = "GOP",      .type = MAV_PARAM_EXT_TYPE_REAL32, .desc = "Keyframe interval (s)",
      .min = 0, .max = 10, .field = "video0.gopSize" },
};

static bool is_bool(const struct cam_param *p)
{
    return p->type == MAV_PARAM_EXT_TYPE_UINT8;
}

bool params_add_exec(const char *spec)
{
    /* NAME:GETCMD:SETCMD - the set command is run with the new value appended */
    const char *c1 = strchr(spec, ':');
    const char *c2 = c1 ? strchr(c1 + 1, ':') : NULL;
    if (c1 == NULL || c2 == NULL || c1 == spec || c1 - spec > 16 || c2 == c1 + 1 || c2[1] == 0 ||
        nparams >= MAX_PARAMS) {
        return false;
    }
    struct cam_param *p = &params[nparams];
    memset(p, 0, sizeof(*p));
    memcpy(p->name, spec, (size_t)(c1 - spec));
    for (const char *c = p->name; *c; c++) {
        if (!((*c >= 'A' && *c <= 'Z') || (*c >= '0' && *c <= '9') || *c == '_')) {
            return false;
        }
    }
    p->type = MAV_PARAM_EXT_TYPE_INT32;
    p->desc = "Custom setting";
    p->min = -2147483647.0;
    p->max = 2147483647.0;
    p->get_cmd = strndup(c1 + 1, (size_t)(c2 - c1 - 1));
    p->set_cmd = strdup(c2 + 1);
    nparams++;
    return true;
}

/* true if d can be stored in the parameter's wire type (false for NaN and infinities) */
static bool value_fits(const struct cam_param *p, double d)
{
    switch (p->type) {
    case MAV_PARAM_EXT_TYPE_UINT8:  return d >= 0 && d <= 255;
    case MAV_PARAM_EXT_TYPE_UINT32: return d >= 0 && d <= 4294967295.0;
    case MAV_PARAM_EXT_TYPE_INT32:  return d >= -2147483648.0 && d <= 2147483647.0;
    default:                        return d >= -3.4e38 && d <= 3.4e38;
    }
}

/* Read the current value from the streamer or the get command. Blocking, and
 * free of side effects on the table, so it can run on the worker thread. */
static bool param_fetch(const struct cam_param *p, double *out)
{
    char v[64] = "";
    if (p->field != NULL) {
        if (!streamer_get_field(mavcam_streamer(), p->field, v, sizeof(v))) {
            return false;
        }
    } else {
        FILE *f = popen(p->get_cmd, "r");
        if (f == NULL) {
            return false;
        }
        const bool got = fgets(v, sizeof(v), f) != NULL;
        if (pclose(f) != 0 || !got) {
            return false;
        }
        v[strcspn(v, "\r\n")] = 0;
    }
    if (p->noptions > 0) {
        for (int i = 0; i < p->noptions; i++) {
            if (strcmp(v, p->options[i]) == 0) {
                *out = i;
                return true;
            }
        }
        return false;   /* a value we have no option for, e.g. a custom resolution */
    }
    if (is_bool(p)) {
        *out = strcmp(v, "true") == 0 || strcmp(v, "1") == 0;
    } else {
        char *end;
        const double d = strtod(v, &end);
        /* the streamer's answer is not trusted: it must fit the wire type */
        if (end == v || !value_fits(p, d)) {
            return false;
        }
        *out = d;
    }
    return true;
}

/* blocking; no side effects on the table */
static bool param_apply(const struct cam_param *p, double value)
{
    char text[32];
    if (p->noptions > 0) {
        snprintf(text, sizeof(text), "%s", p->options[(int)value]);
    } else if (is_bool(p)) {
        snprintf(text, sizeof(text), "%s", value != 0 ? "true" : "false");
    } else if (p->type == MAV_PARAM_EXT_TYPE_REAL32) {
        snprintf(text, sizeof(text), "%g", value);
    } else {
        snprintf(text, sizeof(text), "%ld", (long)value);
    }
    bool ok;
    if (p->field != NULL) {
        ok = streamer_set_field(mavcam_streamer(), p->field, text);
    } else {
        /* text is a number we formatted, so it is safe to append to the command */
        char cmd[512];
        const int n = snprintf(cmd, sizeof(cmd), "%s %s", p->set_cmd, text);
        ok = n > 0 && n < (int)sizeof(cmd) && system(cmd) == 0;
    }
    return ok;
}

static void encode_value(const struct cam_param *p, char out[MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_LEN])
{
    memset(out, 0, MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_LEN);
    switch (p->type) {
    case MAV_PARAM_EXT_TYPE_UINT8:  { uint8_t v = (uint8_t)p->value; memcpy(out, &v, sizeof(v)); break; }
    case MAV_PARAM_EXT_TYPE_UINT32: { uint32_t v = (uint32_t)p->value; memcpy(out, &v, sizeof(v)); break; }
    case MAV_PARAM_EXT_TYPE_INT32:  { int32_t v = (int32_t)p->value; memcpy(out, &v, sizeof(v)); break; }
    default:                        { float v = (float)p->value; memcpy(out, &v, sizeof(v)); break; }
    }
}

static double decode_value(uint8_t type, const char *in)
{
    switch (type) {
    case MAV_PARAM_EXT_TYPE_UINT8:  { uint8_t v; memcpy(&v, in, sizeof(v)); return v; }
    case MAV_PARAM_EXT_TYPE_UINT32: { uint32_t v; memcpy(&v, in, sizeof(v)); return v; }
    case MAV_PARAM_EXT_TYPE_INT32:  { int32_t v; memcpy(&v, in, sizeof(v)); return v; }
    default:                        { float v; memcpy(&v, in, sizeof(v)); return v; }
    }
}

static struct cam_param *find_param(const char *id)
{
    char name[17];
    memcpy(name, id, 16);
    name[16] = 0;
    for (int i = 0; i < nparams; i++) {
        if (strcmp(params[i].name, name) == 0) {
            return &params[i];
        }
    }
    return NULL;
}

/* Send a message from its packed wire-layout struct.
 *
 * The generated mavlink_msg_param_ext_*_encode() helpers copy param_value as a
 * C string and stop at the first zero byte, which corrupts binary values such
 * as the 32-bit integer 8192 (00 20 00 00). Copying the struct avoids that. */
static void send_packed(mavlink_message_t *msg, uint32_t msgid, const void *packet,
                        uint8_t min_len, uint8_t len, uint8_t crc_extra)
{
    memset(msg, 0, sizeof(*msg));
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), packet, len);
    msg->msgid = msgid;
    mavlink_finalize_message(msg, mavcam_sysid(), mavcam_compid(), min_len, len, crc_extra);
    mavcam_send(msg, DST_ALL);
}

static void send_value(struct cam_param *p)
{
    mavlink_param_ext_value_t v;
    memset(&v, 0, sizeof(v));
    strncpy(v.param_id, p->name, sizeof(v.param_id));
    encode_value(p, v.param_value);
    v.param_type = p->type;
    v.param_count = (uint16_t)nparams;
    v.param_index = (uint16_t)(p - params);
    mavlink_message_t msg;
    send_packed(&msg, MAVLINK_MSG_ID_PARAM_EXT_VALUE, &v, MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN,
                MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_CRC);
}

static void xml_escape(FILE *f, const char *s)
{
    for (; *s; s++) {
        switch (*s) {
        case '<': fputs("&lt;", f); break;
        case '>': fputs("&gt;", f); break;
        case '&': fputs("&amp;", f); break;
        case '"': fputs("&quot;", f); break;
        default: fputc(*s, f); break;
        }
    }
}

static const char *type_name(uint8_t type)
{
    switch (type) {
    case MAV_PARAM_EXT_TYPE_UINT8:  return "bool";
    case MAV_PARAM_EXT_TYPE_UINT32: return "uint32";
    case MAV_PARAM_EXT_TYPE_INT32:  return "int32";
    default:                        return "float";
    }
}

static void build_camdef(void)
{
    free(camdef);
    camdef = NULL;
    camdef_len = 0;
    FILE *f = open_memstream(&camdef, &camdef_len);
    if (f == NULL) {
        return;
    }
    fputs("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<mavlinkcamera>\n", f);
    fprintf(f, "  <definition version=\"%d\">\n    <model>", CAMDEF_VERSION);
    xml_escape(f, mavcam_model());
    fputs("</model>\n    <vendor>", f);
    xml_escape(f, mavcam_vendor());
    fputs("</vendor>\n  </definition>\n  <parameters>\n", f);
    for (int i = 0; i < nparams; i++) {
        const struct cam_param *p = &params[i];
        fprintf(f, "    <parameter name=\"%s\" type=\"%s\" default=\"%g\"", p->name, type_name(p->type),
                p->known ? p->value : 0.0);
        if (p->noptions == 0 && !is_bool(p)) {
            fprintf(f, " min=\"%g\" max=\"%g\"", p->min, p->max);
        }
        fputs(">\n      <description>", f);
        xml_escape(f, p->desc);
        fputs("</description>\n", f);
        if (p->noptions > 0) {
            fputs("      <options>\n", f);
            for (int o = 0; o < p->noptions; o++) {
                fprintf(f, "        <option name=\"%s\" value=\"%d\"/>\n",
                        p->labels[o] != NULL ? p->labels[o] : p->options[o], o);
            }
            fputs("      </options>\n", f);
        }
        fputs("    </parameter>\n", f);
    }
    fputs("  </parameters>\n</mavlinkcamera>\n", f);
    fclose(f);
}

void params_init(void)
{
    /* exec params were added while parsing options; put the streamer's first */
    struct cam_param exec[MAX_PARAMS];
    const int nexec = nparams;
    memcpy(exec, params, sizeof(exec[0]) * (size_t)nexec);
    nparams = 0;

    const bool wb = mavcam_streamer()->type == STREAMER_WAYBEAM;
    const struct cam_param *tbl = wb ? waybeam_params : majestic_params;
    const size_t n = wb ? sizeof(waybeam_params) / sizeof(waybeam_params[0])
                        : sizeof(majestic_params) / sizeof(majestic_params[0]);
    for (size_t i = 0; i < n && nparams < MAX_PARAMS; i++) {
        params[nparams] = tbl[i];
        if (params[nparams].noptions > 0) {
            params[nparams].min = 0;
            params[nparams].max = params[nparams].noptions - 1;
        }
        nparams++;
    }
    for (int i = 0; i < nexec && nparams < MAX_PARAMS; i++) {
        params[nparams++] = exec[i];
    }
    for (int i = 0; i < nparams; i++) {
        double v;
        if (param_fetch(&params[i], &v)) {
            params[i].value = v;
            params[i].known = true;
        }
    }
    build_camdef();
}

const char *params_camdef(size_t *len)
{
    *len = camdef_len;
    return camdef;
}

static void read_work(struct job *j)
{
    j->ok = param_fetch(j->ptr, &j->d);
}

static void read_done(struct job *j)
{
    struct cam_param *p = j->ptr;
    if (j->ok) {
        p->value = j->d;
        p->known = true;
    }
    send_value(p);   /* the last known value if the read failed */
}

static void queue_read(struct cam_param *p)
{
    struct job j = { .work = read_work, .done = read_done, .ptr = p };
    worker_submit(&j);   /* if the queue is full the requester simply asks again */
}

static void send_set_ack(const struct cam_param *p, const mavlink_param_ext_set_t *s, uint8_t result)
{
    mavlink_param_ext_ack_t ack;
    memset(&ack, 0, sizeof(ack));
    memcpy(ack.param_id, s->param_id, sizeof(ack.param_id));
    memcpy(ack.param_value, s->param_value, sizeof(ack.param_value));
    ack.param_type = s->param_type;
    ack.param_result = result;
    if (result != PARAM_ACK_ACCEPTED && p != NULL && p->known) {
        encode_value(p, ack.param_value);   /* report the value still in force */
    }
    mavlink_message_t out;
    send_packed(&out, MAVLINK_MSG_ID_PARAM_EXT_ACK, &ack, MAVLINK_MSG_ID_PARAM_EXT_ACK_MIN_LEN,
                MAVLINK_MSG_ID_PARAM_EXT_ACK_LEN, MAVLINK_MSG_ID_PARAM_EXT_ACK_CRC);
}

/* a set in flight: the request is kept so the acknowledgement can echo it */
struct set_request {
    struct cam_param *p;
    mavlink_param_ext_set_t msg;
};

static void set_work(struct job *j)
{
    const struct set_request *r = j->ptr;
    j->ok = param_apply(r->p, j->d);
}

static void set_done(struct job *j)
{
    struct set_request *r = j->ptr;
    mavcam_log("param %s = %g%s", r->p->name, j->d, j->ok ? "" : " FAILED");
    if (j->ok) {
        r->p->value = j->d;
        r->p->known = true;
    }
    send_set_ack(r->p, &r->msg, j->ok ? PARAM_ACK_ACCEPTED : PARAM_ACK_FAILED);
    free(r);
}

static bool for_us(uint8_t target_system, uint8_t target_component)
{
    return (target_system == 0 || target_system == mavcam_sysid()) &&
           (target_component == 0 || target_component == mavcam_compid());
}

bool params_handle_message(const mavlink_message_t *msg, int src)
{
    (void)src;
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: {
        mavlink_param_ext_request_list_t r;
        mavlink_msg_param_ext_request_list_decode(msg, &r);
        if (!for_us(r.target_system, r.target_component)) {
            return false;
        }
        for (int i = 0; i < nparams; i++) {
            queue_read(&params[i]);
        }
        return true;
    }

    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: {
        mavlink_param_ext_request_read_t r;
        mavlink_msg_param_ext_request_read_decode(msg, &r);
        if (!for_us(r.target_system, r.target_component)) {
            return false;
        }
        struct cam_param *p = NULL;
        if (r.param_index >= 0) {
            p = r.param_index < nparams ? &params[r.param_index] : NULL;
        } else {
            p = find_param(r.param_id);
        }
        if (p != NULL) {
            queue_read(p);
        }
        return true;
    }

    case MAVLINK_MSG_ID_PARAM_EXT_SET: {
        mavlink_param_ext_set_t s;
        mavlink_msg_param_ext_set_decode(msg, &s);
        if (!for_us(s.target_system, s.target_component)) {
            return false;
        }
        struct cam_param *p = find_param(s.param_id);
        if (p == NULL || s.param_type != p->type) {
            send_set_ack(p, &s, PARAM_ACK_VALUE_UNSUPPORTED);
            return true;
        }
        const double v = decode_value(s.param_type, s.param_value);
        const bool integral = p->type != MAV_PARAM_EXT_TYPE_REAL32;
        if (isnan(v) || v < p->min || v > p->max || (integral && v != floor(v))) {
            send_set_ack(p, &s, PARAM_ACK_VALUE_UNSUPPORTED);
            return true;
        }
        struct set_request *r = malloc(sizeof(*r));
        if (r == NULL) {
            send_set_ack(p, &s, PARAM_ACK_FAILED);
            return true;
        }
        r->p = p;
        r->msg = s;
        struct job j = { .work = set_work, .done = set_done, .ptr = r, .d = v };
        if (!worker_submit(&j)) {
            free(r);
            send_set_ack(p, &s, PARAM_ACK_FAILED);
        }
        return true;
    }

    default:
        return false;
    }
}
