#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* One set of MAVLink channel state (parser + transmit sequence numbers) shared
 * by every source file, instead of a private copy per file. */
#define MAVLINK_COMM_NUM_BUFFERS 2
#include <mavlink/mavlink_types.h>
#define MAVLINK_GET_CHANNEL_STATUS mavlink_get_channel_status
#define MAVLINK_GET_CHANNEL_BUFFER mavlink_get_channel_buffer
mavlink_status_t *mavlink_get_channel_status(uint8_t chan);
mavlink_message_t *mavlink_get_channel_buffer(uint8_t chan);
#include <mavlink/common/mavlink.h>

#include "streamer.h"

/* where a message goes */
enum {
    DST_FC = 1,       /* flight controller link */
    DST_GROUND = 2,   /* ground / radio link, if configured */
    DST_ALL = 3,
};

void mavcam_send(const mavlink_message_t *msg, int dst);
/* bytes queued but not yet written on the slowest of the given links */
size_t mavcam_backlog(int dst);
/* ceiling for file-transfer data, bytes per second */
double mavcam_ftp_rate(void);
uint8_t mavcam_sysid(void);
uint8_t mavcam_compid(void);
const struct streamer *mavcam_streamer(void);
const char *mavcam_media_dir(void);
const char *mavcam_vendor(void);
const char *mavcam_model(void);
void mavcam_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* ---- params.c: camera settings (PARAM_EXT) and the camera definition file */

/* add a setting backed by shell commands, from "NAME:GETCMD:SETCMD". false if malformed */
bool params_add_exec(const char *spec);
void params_init(void);
/* true if the message was a PARAM_EXT request for this camera */
bool params_handle_message(const mavlink_message_t *msg, int src);
/* the camera definition XML; valid until the next call */
const char *params_camdef(size_t *len);
#define CAMDEF_VERSION 1
#define CAMDEF_NAME "camdef.xml"

/* ---- ftp.c: read-only MAVLink FTP (definition file and media directory) */

bool ftp_handle_message(const mavlink_message_t *msg, int src);
/* send queued burst data; returns true while a transfer is in progress */
bool ftp_periodic(void);
