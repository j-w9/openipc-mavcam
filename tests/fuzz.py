#!/usr/bin/env python3
"""Throw malformed input at mavcam from both sides: random and adversarial
MAVLink on the link, and a streamer that answers with garbage.

    python3 tests/fuzz.py [seconds]

Meant to be run against a sanitizer build; passes if mavcam is still alive and
still answers correctly afterwards.
"""
import math
import os
import random
import socket
import struct
import subprocess
import sys
import tempfile
import time

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import mock_streamer  # noqa: E402

mav = mavutil.mavlink
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 60
rng = random.Random(int(os.environ.get("FUZZ_SEED", "1")))
ODD_FLOATS = [0.0, -0.0, 1.0, -1.0, 100.0, 1e9, -1e9, 1e38, math.nan, math.inf, -math.inf, 2.5, 65536.0, 4294967296.0]
PATHS = [b"", b"/", b".", b"..", b"../../etc/passwd", b"/../x", b"camdef.xml", b"/camdef.xml", b"[;comp=", b"[;comp=100]",
         b"[;comp=100]/../../x", b"a" * 239, b"/" * 239, b"%s%n%x", b"sub/../..", b"\x00", b"\xff" * 50, b"./././camdef.xml"]
CAM_CMDS = [mav.MAV_CMD_REQUEST_MESSAGE, mav.MAV_CMD_SET_MESSAGE_INTERVAL, mav.MAV_CMD_VIDEO_START_CAPTURE,
            mav.MAV_CMD_VIDEO_STOP_CAPTURE, mav.MAV_CMD_IMAGE_START_CAPTURE, mav.MAV_CMD_IMAGE_STOP_CAPTURE,
            mav.MAV_CMD_SET_CAMERA_ZOOM, mav.MAV_CMD_SET_CAMERA_MODE, mav.MAV_CMD_SET_CAMERA_FOCUS,
            mav.MAV_CMD_REQUEST_CAMERA_INFORMATION, mav.MAV_CMD_STORAGE_FORMAT, 0, 65535]
PARAM_NAMES = [b"BITRATE", b"FPS", b"GOP", b"RES", b"MIRROR", b"QP_DELTA", b"TXPOWER", b"", b"X" * 16, b"\xff" * 16, b"RES\x00junk"]


def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def f():
    return rng.choice(ODD_FLOATS) if rng.random() < 0.7 else rng.uniform(-1e6, 1e6)


def ftp_payload():
    op = rng.choice([0, 1, 2, 3, 4, 5, 14, 15, 128, 129, rng.randrange(256)])
    data = rng.choice(PATHS) if rng.random() < 0.7 else bytes(rng.randrange(256) for _ in range(rng.randrange(240)))
    size = rng.choice([len(data), 0, 239, 255, rng.randrange(256)])
    hdr = struct.pack("<HBBBBBxI", rng.randrange(65536), rng.choice([0, 1, 255]), op, size, 0, 0,
                      rng.choice([0, 1, 5, 2**31, 2**32 - 1, rng.randrange(2**32)]))
    return list((hdr + data).ljust(251, b"\0")[:251])


class RawSender:
    """Send raw bytes the same way a pymavlink connection would."""

    def __init__(self, conn):
        self.conn = conn

    def send(self, data):
        self.conn.write(data)


def one(conn, raw):
    kind = rng.randrange(8)
    target_comp = rng.choice([100, 100, 100, 0, 1])
    if kind == 0:
        raw.send(bytes(rng.randrange(256) for _ in range(rng.randrange(1, 300))))
    elif kind == 1:
        conn.mav.file_transfer_protocol_send(0, 7, target_comp, ftp_payload())
    elif kind == 2:
        conn.mav.param_ext_set_send(7, target_comp, rng.choice(PARAM_NAMES),
                                    bytes(rng.randrange(256) for _ in range(128)), rng.randrange(0, 14))
    elif kind == 3:
        conn.mav.param_ext_request_read_send(7, target_comp, rng.choice(PARAM_NAMES),
                                             rng.choice([-1, 0, 5, 11, 12, 32767, -32768]))
    elif kind == 4:
        conn.mav.param_ext_request_list_send(7, target_comp)
    elif kind == 5:
        conn.mav.command_long_send(7, target_comp, rng.choice(CAM_CMDS), rng.randrange(256), f(), f(), f(), f(), f(), f(), f())
    elif kind == 6:
        conn.mav.command_int_send(7, target_comp, rng.randrange(256), rng.choice(CAM_CMDS), 0, 0, f(), f(), f(), f(),
                                  rng.randrange(-2**31, 2**31), rng.randrange(-2**31, 2**31), f())
    else:
        # a frame that claims to be valid MAVLink 2 but has a random message id and body
        body = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 256)))
        raw.send(b"\xfd" + bytes([len(body), 0, 0, rng.randrange(256), 7, 1]) +
                 struct.pack("<I", rng.randrange(2**24))[:3] + body + bytes(rng.randrange(256) for _ in range(2)))


def main():
    server, state = mock_streamer.serve(0)
    tmp = tempfile.mkdtemp()
    os.mkdir(os.path.join(tmp, "sub"))
    fc_port = free_port()
    gcs = mavutil.mavlink_connection("udpin:127.0.0.1:0", source_system=255, source_component=190)
    setter = os.path.join(tmp, "set")
    with open(setter, "w") as fh:
        fh.write("#!/bin/sh\nexit 0\n")
    os.chmod(setter, 0o755)
    proc = subprocess.Popen([os.path.join(ROOT, "mavcam"), "--link", f"udpin:{fc_port}",
                             "--ground-out", f"127.0.0.1:{gcs.port.getsockname()[1]}",
                             "--http", f"127.0.0.1:{server.server_address[1]}", "--media-dir", tmp,
                             "--exec-param", f"TXPOWER:echo 5:{setter}", "--ftp-rate", "200000"])
    fc = mavutil.mavlink_connection(f"udpout:127.0.0.1:{fc_port}", source_system=7, source_component=1)
    fc_raw = RawSender(fc)
    for _ in range(20):
        fc.mav.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
        if gcs.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3):
            break
    gcs.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    gcs_raw = RawSender(gcs)

    sent = 0
    start = time.time()
    last_hb = 0
    while time.time() - start < SECONDS and proc.poll() is None:
        t = time.time() - start
        state.hostile = int(t / 5) % 2 == 1     # alternate: sane streamer, hostile streamer
        if time.time() - last_hb > 1:
            last_hb = time.time()
            fc.mav.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
        for _ in range(20):
            if rng.random() < 0.5:
                one(fc, fc_raw)
            else:
                one(gcs, gcs_raw)
            sent += 1
        for c in (fc, gcs):
            while c.recv_match(blocking=False) is not None:
                pass
        time.sleep(0.005)

    state.hostile = False
    alive = proc.poll() is None
    print(f"sent {sent} fuzz inputs in {time.time() - start:.0f} s; mavcam {'alive' if alive else 'DIED, exit ' + str(proc.returncode)}")
    ok = False
    if alive:
        time.sleep(3)   # let queued work drain
        for c in (fc, gcs):
            while c.recv_match(blocking=False) is not None:
                pass
        for _ in range(5):
            gcs.mav.command_long_send(7, 100, mav.MAV_CMD_REQUEST_MESSAGE, 0, mav.MAVLINK_MSG_ID_CAMERA_INFORMATION, 0, 0, 0, 0, 0, 0)
            end = time.time() + 2
            while time.time() < end and not ok:
                m = gcs.recv_match(type="CAMERA_INFORMATION", blocking=True, timeout=0.3)
                ok = m is not None and bytes(m.vendor_name).rstrip(b"\0") == b"OpenIPC"
            if ok:
                break
        print("still answers correctly afterwards:", ok)
        escaped = [n for n in os.listdir(os.path.dirname(tmp)) if n in ("x", "passwd")]
        print("files written outside the media directory:", escaped or "none")
        proc.terminate()
        proc.wait(timeout=10)
        print("exit code after SIGTERM:", proc.returncode)
    os._exit(0 if alive and ok else 1)


if __name__ == "__main__":
    try:
        main()
    finally:
        subprocess.run(["pkill", "-f", os.path.join(ROOT, "mavcam") + " --link"], check=False)
