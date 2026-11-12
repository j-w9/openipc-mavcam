#!/usr/bin/env python3
"""Integration test against ArduPilot SITL: ArduCopter with CAM1_TYPE=6 (MAVLink
camera v2) on a unicast link to mavcam, which drives the mock streamer.

  python3 tests/test_sitl.py /path/to/ardupilot

Needs build/sitl/bin/arducopter in that tree (./waf configure --board sitl && ./waf copter).
"""
import os
import socket
import subprocess
import sys
import tempfile
import time
import unittest

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import mock_streamer  # noqa: E402

mav = mavutil.mavlink
ARDUPILOT = None
CAM_COMPID = 100

# SERIAL2 is the camera link. MAVLink instances are numbered in serial-port
# order among MAVLink ports: SERIAL0 -> MAV1, SERIAL1 -> MAV2, SERIAL2 -> MAV3.
PARAMS = """
SERIAL2_PROTOCOL 2
CAM1_TYPE 6
MAV3_OPTIONS 16
"""


def free_port(kind):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class SitlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.media = os.path.join(cls.tmp.name, "media")
        os.mkdir(cls.media)
        parm = os.path.join(cls.tmp.name, "mavcam.parm")
        with open(parm, "w") as f:
            f.write(PARAMS)
        cls.server, cls.state = mock_streamer.serve(0)

        gcs_port = free_port(socket.SOCK_STREAM)
        cam_port = free_port(socket.SOCK_DGRAM)
        cls.logs = open(os.path.join(cls.tmp.name, "sitl.log"), "w")
        defaults = os.path.join(ARDUPILOT, "Tools/autotest/default_params/copter.parm")
        cls.sitl = subprocess.Popen([
            os.path.join(ARDUPILOT, "build/sitl/bin/arducopter"),
            "--model", "quad", "--speedup", "1", "-w",
            "--defaults", f"{defaults},{parm}",
            "--serial0", f"tcp:{gcs_port}",
            "--serial2", f"udpclient:127.0.0.1:{cam_port}",
        ], cwd=cls.tmp.name, stdout=cls.logs, stderr=subprocess.STDOUT)
        cls.camlog = open(os.path.join(cls.tmp.name, "mavcam.log"), "w")
        cls.cam = subprocess.Popen([
            os.path.join(ROOT, "mavcam"), "-v",
            "--link", f"udpin:{cam_port}",
            "--streamer", "waybeam",
            "--http", f"127.0.0.1:{cls.server.server_address[1]}",
            "--media-dir", cls.media,
            "--model", "SITLCam",
        ], stderr=cls.camlog)

        deadline = time.time() + 30
        cls.gcs = None
        while time.time() < deadline and cls.gcs is None:
            try:
                cls.gcs = mavutil.mavlink_connection(f"tcp:127.0.0.1:{gcs_port}",
                                                     source_system=255, source_component=190)
            except OSError:
                time.sleep(0.5)
        assert cls.gcs is not None, "could not connect to SITL"
        cls.gcs.wait_heartbeat(timeout=30)
        cls.fc_sysid = cls.gcs.target_system

    @classmethod
    def tearDownClass(cls):
        for p in (cls.cam, cls.sitl):
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        if cls.gcs is not None:
            cls.gcs.close()
        cls.server.shutdown()
        cls.logs.close()
        cls.camlog.close()
        if os.environ.get("KEEP_LOGS"):
            for name in ("sitl.log", "mavcam.log"):
                print(f"\n--- {name}\n" + open(os.path.join(cls.tmp.name, name)).read()[-3000:])
        cls.tmp.cleanup()

    def gcs_heartbeat(self):
        self.gcs.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def recv(self, mtype, timeout=10.0, condition=None):
        deadline = time.time() + timeout
        last_hb = 0
        while time.time() < deadline:
            if time.time() - last_hb > 1:
                self.gcs_heartbeat()
                last_hb = time.time()
            m = self.gcs.recv_match(type=mtype, blocking=True, timeout=0.5)
            if m is not None and (condition is None or condition(m)):
                return m
        return None

    def from_camera(self, m):
        return m.get_srcSystem() == self.fc_sysid and m.get_srcComponent() == CAM_COMPID

    def command_ack(self, target_comp, cmd, *params, timeout=10):
        p = list(params) + [0] * (7 - len(params))
        self.gcs.mav.command_long_send(self.fc_sysid, target_comp, cmd, 0, *p)
        ack = self.recv("COMMAND_ACK", timeout=timeout, condition=lambda m: m.command == cmd)
        return ack

    def test_01_camera_heartbeat_reaches_gcs(self):
        # the backend only starts looking for the camera 10 s after boot
        hb = self.recv("HEARTBEAT", timeout=40, condition=self.from_camera)
        self.assertIsNotNone(hb, "camera heartbeat was not relayed to the GCS")
        self.assertEqual(hb.type, mav.MAV_TYPE_CAMERA)

    def test_02_camera_information_direct(self):
        # addressed to the camera's own component: routed across the unicast link
        ack = self.command_ack(CAM_COMPID, mav.MAV_CMD_REQUEST_MESSAGE,
                               mav.MAVLINK_MSG_ID_CAMERA_INFORMATION)
        self.assertIsNotNone(ack, "no ACK for a request addressed to the camera")
        self.assertEqual(ack.get_srcComponent(), CAM_COMPID)
        self.assertEqual(ack.result, mav.MAV_RESULT_ACCEPTED)
        # the reply is a broadcast, relayed by the camera backend
        self.gcs.mav.command_long_send(self.fc_sysid, CAM_COMPID, mav.MAV_CMD_REQUEST_MESSAGE, 0,
                                       mav.MAVLINK_MSG_ID_CAMERA_INFORMATION, 0, 0, 0, 0, 0, 0)
        ci = self.recv("CAMERA_INFORMATION", condition=self.from_camera)
        self.assertIsNotNone(ci, "CAMERA_INFORMATION was not relayed")
        self.assertEqual(bytes(ci.model_name).rstrip(b"\0"), b"SITLCam")

    def test_03_camera_information_cached_by_fc(self):
        # ask the flight controller; it answers from its cache with the camera's identity
        self.gcs.mav.command_long_send(self.fc_sysid, 1, mav.MAV_CMD_REQUEST_MESSAGE, 0,
                                       mav.MAVLINK_MSG_ID_CAMERA_INFORMATION, 0, 0, 0, 0, 0, 0)
        ci = self.recv("CAMERA_INFORMATION", condition=self.from_camera)
        self.assertIsNotNone(ci, "FC did not serve cached CAMERA_INFORMATION")
        self.assertEqual(bytes(ci.vendor_name).rstrip(b"\0"), b"OpenIPC")

    def test_04_video_stream_information_cached_by_fc(self):
        self.gcs.mav.command_long_send(self.fc_sysid, 1, mav.MAV_CMD_REQUEST_MESSAGE, 0,
                                       mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, 0, 0, 0, 0, 0, 0)
        vi = self.recv("VIDEO_STREAM_INFORMATION", condition=self.from_camera)
        self.assertIsNotNone(vi, "FC did not serve cached VIDEO_STREAM_INFORMATION")
        self.assertEqual((vi.resolution_h, vi.resolution_v), (1920, 1080))
        self.assertEqual(vi.uri, "5600")

    def test_05_record_through_fc(self):
        # the path a mission item or RC switch takes: FC backend -> camera
        ack = self.command_ack(1, mav.MAV_CMD_VIDEO_START_CAPTURE, 0, 0)
        self.assertIsNotNone(ack)
        self.assertEqual(ack.result, mav.MAV_RESULT_ACCEPTED)
        deadline = time.time() + 5
        while time.time() < deadline and not self.state.snapshot()["recording"]:
            time.sleep(0.1)
        self.assertTrue(self.state.snapshot()["recording"], "streamer was not told to record")
        st = self.recv("CAMERA_CAPTURE_STATUS", condition=lambda m: self.from_camera(m) and m.video_status == 1)
        self.assertIsNotNone(st, "GCS did not see recording status")

        ack = self.command_ack(1, mav.MAV_CMD_VIDEO_STOP_CAPTURE, 0)
        self.assertEqual(ack.result, mav.MAV_RESULT_ACCEPTED)
        deadline = time.time() + 5
        while time.time() < deadline and self.state.snapshot()["recording"]:
            time.sleep(0.1)
        self.assertFalse(self.state.snapshot()["recording"])

    def test_06_photo_through_fc(self):
        before = len(os.listdir(self.media))
        ack = self.command_ack(1, mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1)
        self.assertIsNotNone(ack)
        self.assertEqual(ack.result, mav.MAV_RESULT_ACCEPTED)
        img = self.recv("CAMERA_IMAGE_CAPTURED", condition=self.from_camera)
        self.assertIsNotNone(img, "CAMERA_IMAGE_CAPTURED was not relayed")
        self.assertEqual(img.capture_result, 1)
        self.assertEqual(len(os.listdir(self.media)), before + 1)

    def test_07_photo_is_geotagged_from_fc_position(self):
        # mavcam asked the FC for GLOBAL_POSITION_INT; SITL starts at a non-zero home
        deadline = time.time() + 20
        img = None
        while time.time() < deadline:
            self.command_ack(1, mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1)
            img = self.recv("CAMERA_IMAGE_CAPTURED", condition=self.from_camera)
            if img is not None and img.lat != 0:
                break
            time.sleep(1)
        self.assertIsNotNone(img)
        self.assertNotEqual(img.lat, 0, "photo has no position: FC did not stream it to the camera")
        self.assertNotEqual(img.lon, 0)


class SingleUartTest(unittest.TestCase):
    """The stock OpenIPC layout: one FC UART, with the ground station reaching the
    flight controller only through mavcam (which replaces mavfwd)."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        parm = os.path.join(cls.tmp.name, "mavcam.parm")
        with open(parm, "w") as f:
            f.write("SERIAL2_PROTOCOL 2\nCAM1_TYPE 6\n")   # an ordinary (not unicast) link
        cls.server, cls.state = mock_streamer.serve(0)
        cam_port = free_port(socket.SOCK_DGRAM)
        cls.logs = open(os.path.join(cls.tmp.name, "sitl.log"), "w")
        defaults = os.path.join(ARDUPILOT, "Tools/autotest/default_params/copter.parm")
        cls.sitl = subprocess.Popen([
            os.path.join(ARDUPILOT, "build/sitl/bin/arducopter"),
            "--model", "quad", "--speedup", "1", "-w",
            "--defaults", f"{defaults},{parm}",
            "--serial0", f"tcp:{free_port(socket.SOCK_STREAM)}",
            "--serial2", f"udpclient:127.0.0.1:{cam_port}",
        ], cwd=cls.tmp.name, stdout=cls.logs, stderr=subprocess.STDOUT)
        cls.gcs = mavutil.mavlink_connection("udpin:127.0.0.1:0", source_system=255, source_component=190)
        cls.cam = subprocess.Popen([
            os.path.join(ROOT, "mavcam"),
            "--link", f"udpin:{cam_port}",
            "--ground-out", f"127.0.0.1:{cls.gcs.port.getsockname()[1]}",
            "--http", f"127.0.0.1:{cls.server.server_address[1]}",
            "--media-dir", cls.tmp.name,
        ])
        cls.fc_sysid = 1

    @classmethod
    def tearDownClass(cls):
        for p in (cls.cam, cls.sitl):
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        cls.gcs.close()
        cls.server.shutdown()
        cls.logs.close()
        cls.tmp.cleanup()

    def recv(self, mtype, timeout=15.0, condition=None):
        deadline = time.time() + timeout
        last_hb = 0
        while time.time() < deadline:
            m = self.gcs.recv_match(type=mtype, blocking=True, timeout=0.5)
            if time.time() - last_hb > 1:
                self.gcs.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = time.time()
            if m is not None and (condition is None or condition(m)):
                return m
        return None

    def test_01_fc_and_camera_both_visible_through_mavcam(self):
        fc = self.recv("HEARTBEAT", timeout=40, condition=lambda m: m.get_srcComponent() == 1)
        self.assertIsNotNone(fc, "FC heartbeat was not forwarded to the ground")
        self.assertEqual(fc.autopilot, mav.MAV_AUTOPILOT_ARDUPILOTMEGA)
        cam = self.recv("HEARTBEAT", condition=lambda m: m.get_srcComponent() == CAM_COMPID)
        self.assertIsNotNone(cam)
        self.assertEqual(cam.get_srcSystem(), fc.get_srcSystem())

    def test_02_fc_telemetry_and_parameters_work_through_mavcam(self):
        # ArduPilot-dialect messages (unknown to mavcam) must survive the trip too
        # nothing is streamed on a telemetry port until a ground station asks
        self.gcs.mav.command_long_send(self.fc_sysid, 1, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       mav.MAVLINK_MSG_ID_AHRS, 200000, 0, 0, 0, 0, 0)
        self.assertIsNotNone(self.recv("AHRS"), "ArduPilot-specific telemetry did not come through")
        self.gcs.mav.param_request_read_send(self.fc_sysid, 1, b"CAM1_TYPE", -1)
        m = self.recv("PARAM_VALUE", condition=lambda m: m.param_id == "CAM1_TYPE")
        self.assertIsNotNone(m, "parameter request did not make the round trip through mavcam")
        self.assertEqual(m.param_value, 6)

    def test_03_record_through_fc(self):
        # GCS -> mavcam -> FC -> (camera backend) -> mavcam -> streamer
        deadline = time.time() + 40
        ack = None
        while time.time() < deadline:
            self.gcs.mav.command_long_send(self.fc_sysid, 1, mav.MAV_CMD_VIDEO_START_CAPTURE, 0,
                                           0, 0, 0, 0, 0, 0, 0)
            ack = self.recv("COMMAND_ACK", timeout=3,
                            condition=lambda m: m.command == mav.MAV_CMD_VIDEO_START_CAPTURE
                            and m.get_srcComponent() == 1)
            # the FC refuses until its backend has discovered the camera
            if ack is not None and ack.result == mav.MAV_RESULT_ACCEPTED:
                break
        self.assertIsNotNone(ack)
        self.assertEqual(ack.result, mav.MAV_RESULT_ACCEPTED)
        deadline = time.time() + 5
        while time.time() < deadline and not self.state.snapshot()["recording"]:
            time.sleep(0.1)
        self.assertTrue(self.state.snapshot()["recording"], "streamer was not told to record")
        self.assertEqual(self.state.snapshot()["record_starts"], 1, "record was started more than once")


class Stack:
    """ArduCopter SITL with the camera on a unicast link, plus a GCS on SERIAL0."""

    def __init__(self, extra_params="", cam_args=(), start_camera=True):
        self.tmp = tempfile.TemporaryDirectory()
        parm = os.path.join(self.tmp.name, "mavcam.parm")
        with open(parm, "w") as f:
            f.write(PARAMS + extra_params)
        self.server, self.state = mock_streamer.serve(0)
        gcs_port = free_port(socket.SOCK_STREAM)
        self.cam_port = free_port(socket.SOCK_DGRAM)
        defaults = os.path.join(ARDUPILOT, "Tools/autotest/default_params/copter.parm")
        self.sitl = subprocess.Popen([
            os.path.join(ARDUPILOT, "build/sitl/bin/arducopter"),
            "--model", "quad", "--speedup", "1", "-w",
            "--defaults", f"{defaults},{parm}",
            "--serial0", f"tcp:{gcs_port}",
            "--serial2", f"udpclient:127.0.0.1:{self.cam_port}",
        ], cwd=self.tmp.name, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.cam = None
        self.cam_args = list(cam_args)
        self.gcs = None
        deadline = time.time() + 30
        while time.time() < deadline and self.gcs is None:
            try:
                self.gcs = mavutil.mavlink_connection(f"tcp:127.0.0.1:{gcs_port}",
                                                      source_system=255, source_component=190)
            except OSError:
                time.sleep(0.5)
        self.gcs.wait_heartbeat(timeout=30)
        self.fc_sysid = self.gcs.target_system
        if start_camera:
            self.start_camera()

    def start_camera(self):
        self.cam = subprocess.Popen([
            os.path.join(ROOT, "mavcam"), "--link", f"udpin:{self.cam_port}",
            "--http", f"127.0.0.1:{self.server.server_address[1]}",
            "--media-dir", self.tmp.name] + self.cam_args)

    def close(self):
        for p in (self.cam, self.sitl):
            if p is None:
                continue
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        self.gcs.close()
        self.server.shutdown()
        self.tmp.cleanup()

    def recv(self, mtype, timeout=10.0, condition=None, tick=None):
        deadline = time.time() + timeout
        last = 0
        while time.time() < deadline:
            if time.time() - last > 0.5:
                self.gcs.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                if tick is not None:
                    tick()
                last = time.time()
            m = self.gcs.recv_match(type=mtype, blocking=True, timeout=0.3)
            if m is not None and (condition is None or condition(m)):
                return m
        return None

    def camera_known_to_fc(self, timeout=35):
        """True once the FC serves cached CAMERA_INFORMATION for the camera."""
        def ask():
            self.gcs.mav.command_long_send(self.fc_sysid, 1, mav.MAV_CMD_REQUEST_MESSAGE, 0,
                                           mav.MAVLINK_MSG_ID_CAMERA_INFORMATION, 0, 0, 0, 0, 0, 0)
        return self.recv("CAMERA_INFORMATION", timeout=timeout, tick=ask,
                         condition=lambda m: m.get_srcComponent() == CAM_COMPID)


class SigningTest(unittest.TestCase):
    """A flight controller with MAVLink signing enabled drops unsigned traffic,
    so the camera has to sign with the same key to be seen at all."""

    KEY = bytes(range(32))

    def run_with(self, give_key):
        st = Stack(start_camera=False)
        try:
            keyfile = os.path.join(st.tmp.name, "key")
            with open(keyfile, "w") as f:
                f.write(self.KEY.hex() + "\n")
            st.gcs.mav.setup_signing_send(st.fc_sysid, 1, list(self.KEY),
                                          int((time.time() - 1420070400) * 1e5))
            time.sleep(1)
            st.gcs.setup_signing(self.KEY, sign_outgoing=True, allow_unsigned_callback=lambda *a: True)
            st.cam_args = ["--signing-key-file", keyfile] if give_key else []
            st.start_camera()
            return st.camera_known_to_fc(timeout=30 if give_key else 20)
        finally:
            st.close()

    def test_unsigned_camera_is_ignored(self):
        # documents the behaviour that makes the key necessary
        self.assertIsNone(self.run_with(give_key=False))

    def test_signed_camera_is_found(self):
        ci = self.run_with(give_key=True)
        self.assertIsNotNone(ci, "FC did not accept the camera even though it signs with the right key")
        self.assertTrue(ci.get_signed(), "the camera's own message was not signed")


class TriggerTest(unittest.TestCase):
    """The ways a pilot actually triggers the camera: RC switches and the classic
    trigger command, all going through the flight controller's camera backend."""

    @classmethod
    def setUpClass(cls):
        # RC7 = "Camera Record Video", RC8 = "Camera Trigger"
        cls.st = Stack(extra_params="RC7_OPTION 166\nRC8_OPTION 9\n")
        if cls.st.camera_known_to_fc() is None:
            cls.st.close()
            raise AssertionError("FC never discovered the camera")

    @classmethod
    def tearDownClass(cls):
        cls.st.close()

    def hold_rc(self, seconds, **channels):
        """Hold RC override values, e.g. ch7=1900, for a while."""
        pwm = [65535] * 8   # 65535 = leave this channel alone
        for name, value in channels.items():
            pwm[int(name[2:]) - 1] = value
        end = time.time() + seconds
        while time.time() < end:
            self.st.gcs.mav.rc_channels_override_send(self.st.fc_sysid, 1, *pwm)
            self.st.gcs.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.st.gcs.recv_match(blocking=True, timeout=0.1)

    def wait_state(self, pred, timeout=6):
        end = time.time() + timeout
        while time.time() < end:
            if pred(self.st.state.snapshot()):
                return True
            time.sleep(0.1)
        return False

    def test_01_rc_switch_records(self):
        self.hold_rc(1.0, ch7=1100)
        self.hold_rc(1.5, ch7=1900)
        self.assertTrue(self.wait_state(lambda s: s["recording"]), "RC switch high did not start recording")
        self.hold_rc(1.5, ch7=1100)
        self.assertTrue(self.wait_state(lambda s: not s["recording"]), "RC switch low did not stop recording")

    def test_02_rc_switch_takes_photo(self):
        before = self.st.state.snapshot()["snapshots"]
        self.hold_rc(1.0, ch8=1100)
        self.hold_rc(1.5, ch8=1900)
        self.assertTrue(self.wait_state(lambda s: s["snapshots"] > before), "RC camera trigger took no photo")
        self.hold_rc(0.5, ch8=1100)

    def test_03_digicam_control_takes_photo(self):
        # the command missions have long used to fire a camera
        before = self.st.state.snapshot()["snapshots"]
        self.st.gcs.mav.command_long_send(self.st.fc_sysid, 1, mav.MAV_CMD_DO_DIGICAM_CONTROL, 0,
                                          0, 0, 0, 0, 1, 0, 0)
        ack = self.st.recv("COMMAND_ACK", condition=lambda m: m.command == mav.MAV_CMD_DO_DIGICAM_CONTROL)
        self.assertIsNotNone(ack)
        self.assertEqual(ack.result, mav.MAV_RESULT_ACCEPTED)
        self.assertTrue(self.wait_state(lambda s: s["snapshots"] > before), "DO_DIGICAM_CONTROL took no photo")


if __name__ == "__main__":
    if len(sys.argv) < 2 or not os.path.isfile(os.path.join(sys.argv[1], "build/sitl/bin/arducopter")):
        sys.exit(__doc__)
    ARDUPILOT = os.path.abspath(sys.argv.pop(1))
    unittest.main(verbosity=2)
