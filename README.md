# mavcam

A small service for OpenIPC cameras that makes the camera a native MAVLink
camera (Camera Protocol v2). It listens for standard MAVLink camera commands on
the flight-controller link and carries them out through the streamer's
localhost HTTP API.

```
flight controller  <--serial / UDP / TCP-->  mavcam  <--HTTP on localhost-->  waybeam or majestic
```

On ArduPilot the camera is then picked up by the MAVLink camera-v2 backend
(`CAMn_TYPE=6`), with no camera-specific code in the autopilot.

> **Status: not yet run on a camera.** Everything here has been tested on a
> desk — against ArduPilot SITL, a mock streamer, MAVSDK, sanitizers and a
> fuzzer — but never flashed or booted on hardware. See [Status](#status).

## What it does

| MAVLink | Action |
|---|---|
| `HEARTBEAT` (type camera, component 100) | Sent at 1 Hz. Adopts the autopilot's system ID unless `--sysid` is given |
| `MAV_CMD_REQUEST_MESSAGE` and the older `MAV_CMD_REQUEST_*` | `CAMERA_INFORMATION`, `CAMERA_SETTINGS`, `STORAGE_INFORMATION`, `CAMERA_CAPTURE_STATUS`, `VIDEO_STREAM_INFORMATION`, `VIDEO_STREAM_STATUS` |
| `MAV_CMD_SET_MESSAGE_INTERVAL` | Streams capture status, storage, stream status or settings (max 10 Hz) |
| `MAV_CMD_VIDEO_START_CAPTURE` / `STOP_CAPTURE` | Starts / stops recording in the streamer |
| `MAV_CMD_IMAGE_START_CAPTURE` / `STOP_CAPTURE` | Saves JPEG snapshots to the media directory, single or at an interval; emits `CAMERA_IMAGE_CAPTURED` with position |
| `MAV_CMD_SET_CAMERA_ZOOM` (range, step, continuous) | Digital zoom presets (waybeam only) |
| `MAV_CMD_SET_CAMERA_MODE` | Image / video mode |
| `PARAM_EXT_REQUEST_LIST` / `READ` / `SET` | Camera settings, read from and written to the streamer |
| `FILE_TRANSFER_PROTOCOL` (read-only) | Serves the camera definition file and the media directory |

Commands are accepted as `COMMAND_LONG` or `COMMAND_INT`. Everything that
talks to the streamer runs on a worker thread, so a slow or absent streamer
delays only the camera's own replies, not MAVLink forwarding.

It also asks the autopilot for `GLOBAL_POSITION_INT` at 1 Hz to geotag photos,
since ArduPilot does not stream telemetry on a unicast camera link, and it
notices recordings started or stopped outside MAVLink (web page, RC script).

### Settings

`CAMERA_INFORMATION` points at `mftp://[;comp=100]/camdef.xml`, a camera
definition file generated at start-up and served over MAVLink FTP.

| Streamer | Settings |
|---|---|
| waybeam | `BITRATE`, `FPS`, `GOP`, `RES`, `QP_DELTA`, `ROI_QP`, `GAIN_MAX`, `AWB_MODE`, `AWB_CT`, `MIRROR`, `FLIP` |
| majestic | `BITRATE`, `FPS`, `GOP` |

Anything else can be added as an integer setting backed by shell commands, for
example radio transmit power (the commands are yours to choose for your image):

```sh
mavcam --exec-param 'TXPOWER:cat /tmp/txpower:/usr/bin/set-txpower'
```

The get command must print a number; the set command is run with the new
value appended.

### Forwarding

On a camera with a single flight-controller UART, mavcam can take the place of
`mavfwd`: with `--ground-out` (and optionally `--ground-in`) it passes all
MAVLink between the flight controller and the radio link, including messages
it has no definition for, while answering camera traffic itself. Traffic
addressed only to the camera is not passed to the flight controller, and file
transfer data goes only to the link that asked for it.

Streamer differences:

| | waybeam | majestic |
|---|---|---|
| Record start/stop | `/api/v1/record/start`, `/stop`, `/status` | toggles `records.enabled` via `/api/v1/set` |
| Snapshot | `/api/v1/snapshot.jpg` | `/image.jpg` |
| Zoom | `video0.framing=zoom-…` | not supported |

## Build

```sh
make                 # native, for development
make test            # desk tests against tools/mock_streamer.py (needs pymavlink)
make CC=arm-openipc-linux-gnueabihf-gcc     # for the camera
```

No dependencies beyond libc. `third_party/mavlink` holds generated MAVLink C
headers (common dialect).

## Run

```sh
# on the camera, dedicated camera UART
mavcam --link /dev/ttyS2:115200 --streamer waybeam

# on the camera, single UART shared with telemetry (replaces mavfwd; same ports it uses)
mavcam --link /dev/ttyS2:115200 --ground-out 127.0.0.1:14551 --ground-in 14550

# on a desk, against the mock streamer and a MAVLink endpoint
python3 tools/mock_streamer.py --port 8080 &
./mavcam -v --link udp:127.0.0.1:14550 --http 127.0.0.1:8080 --media-dir /tmp
```

`mavcam --help` lists all options.

Behaviour worth knowing:

- **System ID.** By default mavcam adopts the autopilot's system ID and stays
  silent until it has seen the autopilot's heartbeat (or five seconds have
  passed, so it still works with no autopilot attached). `--sysid` fixes it.
- **Signing.** If the flight controller has MAVLink 2 signing enabled it drops
  unsigned traffic, camera included. Give mavcam the same key with
  `--signing-key-file` (32 raw bytes or 64 hex digits). Forwarded frames are
  passed on exactly as received.
- **Serial back-pressure.** On a UART, frames are queued and written whole or
  not at all, and file-transfer data waits for the queue to drain, so a
  download cannot corrupt or crowd out telemetry. `--ftp-rate` caps
  file-transfer data on any link (default 16000 bytes per second).
- **Definition file address.** By default `CAMERA_INFORMATION` advertises
  `mftp://[;comp=100]/camdef.xml`. QGroundControl's parser honours the
  component selector. MAVSDK 3.x ignores it and asks the autopilot instead, so
  it cannot fetch a definition file from a separate camera component; for
  MAVSDK, host a copy over HTTP (`mavcam --dump-camdef > camdef.xml`) and
  advertise that with `--camdef-uri`.

ArduPilot side: `CAM1_TYPE=6` and the camera's serial port set to MAVLink2.
On a dedicated camera UART also set `MAVn_OPTIONS` bit 4 (unicast) on that
link, as recommended in `libraries/AP_Camera/README.md`. When the same UART
carries ground-station telemetry through mavcam, leave it as a normal link.

## Firmware images

`deploy/build-image.sh` makes a ready-to-flash package from OpenIPC's released
firmware for a device: it adds `/usr/bin/mavcam`, applies the launcher patch,
sets `telemetry.router: mavcam` in `/etc/wfb.yaml`, and repacks the root
filesystem. The kernel is passed through untouched.

```sh
deploy/build-image.sh runcam-wifilink emax-wyvern-link    # needs Docker
# -> deploy/out/ssc338q_wfbng_runcam-wifilink-nor-mavcam.tgz
# -> deploy/out/ssc338q_wfbng_emax-wyvern-link-nor-mavcam.tgz

deploy/build-image.sh ssc338q_waybeam_generic-nor         # OpenIPC's waybeam image
ROUTER_DEFAULT=msposd deploy/build-image.sh runcam-wifilink   # ship mavcam but leave it off
```

`runcam-wifilink` is OpenIPC's one target for both WiFiLink versions. The
waybeam image is OpenIPC's generic SSC338Q build, not a per-device one.

Flash it with the camera's own updater, which checks the checksums, the SoC and
that the image fits before writing anything:

```sh
scp -O deploy/out/ssc338q_wfbng_runcam-wifilink-nor-mavcam.tgz root@<camera-ip>:/tmp/fw.tgz
ssh root@<camera-ip> sysupgrade --archive=/tmp/fw.tgz
```

Settings survive an upgrade, so a camera whose `/etc/wfb.yaml` was ever edited
keeps its old `router:` line; set it to `mavcam` afterwards, or add `-n` to
`sysupgrade` to start from the image's defaults (this wipes all settings,
including the link key and channel). To go back, flash the stock package the
same way.

What was checked on a desk, for each package:

- Only three things differ from the stock root filesystem: `/usr/bin/mavcam`,
  `/usr/bin/wifibroadcast` and `/etc/wfb.yaml` (plus a marker file); ownership
  and modes are unchanged, the kernel is byte-identical, and the checksums
  inside the package verify.
- Under ARM emulation, inside the image's own filesystem: `mavcam` loads
  against the image's C library, runs, and sends MAVLink 2 heartbeats on both
  the flight-controller and ground links; the image's shell parses the patched
  launcher and reads `router=mavcam` from the config.

**None of these images has been flashed or booted on a camera.** In particular
the launcher's new branch, the UART, and the real streamer's replies have only
been exercised in emulation or against a mock.

## Installing by hand

The same changes the image builder makes, applied to a running camera. Not yet
tried on hardware. The steps follow the stock FPV image's layout, where
`/usr/bin/wifibroadcast` starts either `msposd` or `mavfwd` on the FC UART
according to `telemetry.router` in `/etc/wfb.yaml`.

```sh
# 1. build with the OpenIPC toolchain for the SSC338Q / SSC30KQ
make CC=/path/to/toolchain.sigmastar-infinity6e/bin/arm-openipc-linux-gnueabihf-gcc CFLAGS=-Os

# 2. copy it to the camera
scp -O mavcam root@<camera-ip>:/usr/bin/mavcam

# 3. teach the launcher about it (adds a "mavcam" router and stops it on shutdown)
scp -O deploy/wifibroadcast-mavcam.patch root@<camera-ip>:/tmp/
ssh root@<camera-ip> 'cp /usr/bin/wifibroadcast /usr/bin/wifibroadcast.orig &&
                      patch /usr/bin/wifibroadcast < /tmp/wifibroadcast-mavcam.patch'
#    if the image has no patch(1), apply the two small edits in the patch by hand

# 4. select it and restart the link
ssh root@<camera-ip> "sed -i 's/router: .*/router: mavcam/' /etc/wfb.yaml && wifibroadcast start"
```

To go back: set `router:` to `msposd` or `mavfwd` again, or restore
`/usr/bin/wifibroadcast.orig`.

Selecting `mavcam` replaces `msposd`, so the on-camera MSP OSD is off; the
ground station gets MAVLink telemetry on the usual port instead.

Flight controller: the UART wired to the camera set to MAVLink2 at 115200
(`SERIALn_PROTOCOL=2`, `SERIALn_BAUD=115`) and `CAM1_TYPE=6`, then reboot.

The patch is against the launcher in OpenIPC firmware master as of November
2026; `--streamer` is chosen by whether `/usr/bin/waybeam` exists.

## Status

Nothing here has run on a camera. What has been checked, all on a desk:

| Check | What it covers |
|---|---|
| `make test` | Camera protocol, forwarding, settings, file transfer, serial transport on a pseudo-terminal throttled to 115200 baud, the output queue, and a ground link with 10 % loss each way and reordering, against `tools/mock_streamer.py` with pymavlink as flight controller and ground station |
| `python3 tests/test_sitl.py <ardupilot>` | ArduCopter SITL with `CAM1_TYPE=6`: dedicated unicast camera link, single-UART layout with the ground station reaching the autopilot through mavcam, MAVLink signing, RC switches for record and photo, `DO_DIGICAM_CONTROL` |
| `python3 tests/check_mavsdk.py <ardupilot> [--http-definition]` | MAVSDK 3.x camera plugin as an independent client (needs `pip install 'mavsdk<4'`) |
| `python3 tests/fuzz.py` | Random and adversarial MAVLink on both links, with the streamer answering garbage; meant for a sanitizer build |
| Sanitizers | All of `make test` under AddressSanitizer + UndefinedBehaviorSanitizer + LeakSanitizer, and again under ThreadSanitizer, on Linux |
| Source cross-check | Routes, field names, value formats and defaults compared against waybeam's source |

Not checked, because it needs hardware: booting the firmware images, the real
UART, a running waybeam or majestic, a real flight controller and ground
station, and the radio link.

Known gaps:

- Majestic: the endpoints used (`/api/v1/get`, `/api/v1/set`, `/image.jpg`,
  `/metrics/records`) exist in the shipped binary and follow its documentation,
  but its replies have not been seen from a running instance. In particular
  the reply format of `/api/v1/get?key=` is a guess with a fallback.
- It cannot share the FC serial port with `msposd`: MAVLink and MSP DisplayPort
  need separate UARTs.
- It forwards MAVLink but does not do the other things `mavfwd` does (running
  a script on RC channel changes, injecting SoC temperature).
- File transfer is read-only and one session at a time. Downloading recordings
  over a telemetry-rate link is slow; it is meant for the definition file and
  photos.
- No tracking or focus: these cameras have a fixed-focus lens and no tracker,
  so the commands are answered as unsupported and the capability flags are
  not set.
- Zoom is the streamer's digital presets. Each step restarts waybeam's video
  pipeline, so continuous zoom moves one preset per second with a visible
  hitch at each step.

## License

GPL-3.0; see [LICENSE](LICENSE). `third_party/mavlink` holds MAVLink C headers
generated by pymavlink's mavgen; the MAVLink project licenses generated C
headers under the MIT license.
