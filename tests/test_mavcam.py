#!/usr/bin/env python3
"""End-to-end test: run mavcam against the mock streamer and talk to it over
UDP the way a flight controller would.

  python3 tests/test_mavcam.py            # both streamer types
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
FC_SYSID = 7
CAM_COMPID = 100


class Harness:
    """A fake flight controller on one end of a UDP link to mavcam."""

    def __init__(self, streamer_type, fields=None):
        self.server, self.state = mock_streamer.serve(0)
        self.state.fields.update(fields or {})
        self.media = tempfile.TemporaryDirectory()
        # Like a real autopilot, the fake FC talks first: mavcam listens, the FC sends to it.
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        self.proc = subprocess.Popen([
            os.path.join(ROOT, "mavcam"),
            "--link", f"udpin:{port}",
            "--streamer", streamer_type,
            "--http", f"127.0.0.1:{self.server.server_address[1]}",
            "--media-dir", self.media.name,
            "--model", "TestCam",
        ])
        self.fc = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=FC_SYSID,
                                             source_component=1)
        self.sent_cmds = []

    def close(self):
        self.proc.terminate()
        self.proc.wait(timeout=5)
        self.fc.close()
        self.server.shutdown()
        self.media.cleanup()

    def fc_heartbeat(self):
        self.fc.mav.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0,
                                   mav.MAV_STATE_STANDBY)

    def recv(self, mtype, timeout=3.0, condition=None):
        """Next message of the given type from the camera component."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = self.fc.recv_match(blocking=True, timeout=0.2)
            if m is None:
                continue
            if m.get_srcComponent() != CAM_COMPID:
                continue
            if m.get_type() == "COMMAND_LONG":
                self.sent_cmds.append(m)
            if m.get_type() == mtype and (condition is None or condition(m)):
                return m
        return None

    def command(self, cmd, *params, target_comp=CAM_COMPID):
        p = list(params) + [0] * (7 - len(params))
        self.fc.mav.command_long_send(FC_SYSID, target_comp, cmd, 0, *p)

    def command_ack(self, cmd, *params, **kw):
        self.command(cmd, *params, **kw)
        ack = self.recv("COMMAND_ACK", condition=lambda m: m.command == cmd)
        return None if ack is None else ack.result

    def request(self, msgid, mtype):
        self.command(mav.MAV_CMD_REQUEST_MESSAGE, msgid)
        return self.recv(mtype)


class Common:
    streamer_type = None

    @classmethod
    def setUpClass(cls):
        cls.h = Harness(cls.streamer_type)
        # discovery: the camera adopts the autopilot's system ID
        deadline = time.time() + 5
        cls.cam_hb = None
        while time.time() < deadline:
            cls.h.fc_heartbeat()
            hb = cls.h.recv("HEARTBEAT", timeout=0.5)
            if hb is not None and hb.get_srcSystem() == FC_SYSID:
                cls.cam_hb = hb
                break

    @classmethod
    def tearDownClass(cls):
        cls.h.close()

    def test_01_heartbeat_identity(self):
        hb = self.cam_hb
        self.assertIsNotNone(hb, "no camera heartbeat with the autopilot's system ID")
        self.assertEqual(hb.type, mav.MAV_TYPE_CAMERA)
        self.assertEqual(hb.autopilot, mav.MAV_AUTOPILOT_INVALID)
        self.assertEqual(hb.get_srcComponent(), CAM_COMPID)

    def test_02_camera_information(self):
        ci = self.h.request(mav.MAVLINK_MSG_ID_CAMERA_INFORMATION, "CAMERA_INFORMATION")
        self.assertIsNotNone(ci)
        self.assertEqual(bytes(ci.vendor_name).rstrip(b"\0"), b"OpenIPC")
        self.assertEqual(bytes(ci.model_name).rstrip(b"\0"), b"TestCam")
        self.assertEqual((ci.resolution_h, ci.resolution_v), (1920, 1080))
        for flag in (mav.CAMERA_CAP_FLAGS_CAPTURE_VIDEO, mav.CAMERA_CAP_FLAGS_CAPTURE_IMAGE,
                     mav.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM):
            self.assertTrue(ci.flags & flag)
        has_zoom = bool(ci.flags & mav.CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM)
        self.assertEqual(has_zoom, self.streamer_type == "waybeam")

    def test_03_video_stream_information(self):
        vi = self.h.request(mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, "VIDEO_STREAM_INFORMATION")
        self.assertIsNotNone(vi)
        self.assertEqual((vi.stream_id, vi.count), (1, 1))
        self.assertEqual((vi.resolution_h, vi.resolution_v), (1920, 1080))
        self.assertAlmostEqual(vi.framerate, 60.0)
        self.assertEqual(vi.bitrate, 8192 * 1000)
        self.assertEqual(vi.uri, "5600")
        self.assertEqual(vi.type, mav.VIDEO_STREAM_TYPE_RTPUDP)
        self.assertTrue(vi.flags & mav.VIDEO_STREAM_STATUS_FLAGS_RUNNING)

    def test_04_video_record(self):
        self.assertEqual(self.h.command_ack(mav.MAV_CMD_VIDEO_START_CAPTURE, 0, 0),
                         mav.MAV_RESULT_ACCEPTED)
        self.assertTrue(self.h.state.snapshot()["recording"])
        st = self.h.request(mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, "CAMERA_CAPTURE_STATUS")
        self.assertEqual(st.video_status, 1)

        self.assertEqual(self.h.command_ack(mav.MAV_CMD_VIDEO_STOP_CAPTURE, 0),
                         mav.MAV_RESULT_ACCEPTED)
        self.assertFalse(self.h.state.snapshot()["recording"])
        st = self.h.request(mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, "CAMERA_CAPTURE_STATUS")
        self.assertEqual(st.video_status, 0)
        self.assertEqual(st.recording_time_ms, 0)

    def test_05_record_started_outside_mavlink_is_reported(self):
        # e.g. the pilot starts recording from the camera's web page
        with self.h.state.lock:
            self.h.state.recording = True
            self.h.state.fields["records.enabled"] = True
        st = self.h.recv("CAMERA_CAPTURE_STATUS", timeout=4, condition=lambda m: m.video_status == 1)
        self.assertIsNotNone(st, "camera did not announce a recording started behind its back")
        self.assertEqual(self.h.command_ack(mav.MAV_CMD_VIDEO_STOP_CAPTURE, 0),
                         mav.MAV_RESULT_ACCEPTED)

    def test_06_single_image(self):
        before = self.h.state.snapshot()["snapshots"]
        self.h.command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1)
        img = self.h.recv("CAMERA_IMAGE_CAPTURED")
        self.assertIsNotNone(img)
        self.assertEqual(img.capture_result, 1)
        self.assertEqual(img.image_index, 1)
        self.assertTrue(os.path.isfile(img.file_url), img.file_url)
        with open(img.file_url, "rb") as f:
            self.assertEqual(f.read(), mock_streamer.JPEG)
        self.assertEqual(self.h.state.snapshot()["snapshots"], before + 1)

    def test_07_interval_capture(self):
        self.assertEqual(self.h.command_ack(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0.3, 0),
                         mav.MAV_RESULT_ACCEPTED)
        first = self.h.recv("CAMERA_IMAGE_CAPTURED")
        second = self.h.recv("CAMERA_IMAGE_CAPTURED")
        self.assertIsNotNone(second)
        self.assertEqual(second.image_index, first.image_index + 1)
        st = self.h.request(mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, "CAMERA_CAPTURE_STATUS")
        self.assertEqual(st.image_status, 2)
        self.assertAlmostEqual(st.image_interval, 0.3, places=3)

        self.assertEqual(self.h.command_ack(mav.MAV_CMD_IMAGE_STOP_CAPTURE), mav.MAV_RESULT_ACCEPTED)
        st = self.h.request(mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, "CAMERA_CAPTURE_STATUS")
        self.assertEqual(st.image_status, 0)
        self.assertGreaterEqual(st.image_count, second.image_index)
        time.sleep(0.5)
        self.assertIsNone(self.h.recv("CAMERA_IMAGE_CAPTURED", timeout=0.5),
                          "images kept coming after stop")

    def test_08_multi_image_without_interval_is_denied(self):
        self.assertEqual(self.h.command_ack(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 0),
                         mav.MAV_RESULT_DENIED)

    def test_09_storage_information(self):
        si = self.h.request(mav.MAVLINK_MSG_ID_STORAGE_INFORMATION, "STORAGE_INFORMATION")
        self.assertIsNotNone(si)
        self.assertEqual(si.status, mav.STORAGE_STATUS_READY)
        self.assertGreater(si.total_capacity, 0)
        self.assertGreater(si.available_capacity, 0)

    def test_10_message_interval(self):
        self.assertEqual(
            self.h.command_ack(mav.MAV_CMD_SET_MESSAGE_INTERVAL,
                               mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, 200000),
            mav.MAV_RESULT_ACCEPTED)
        got = [self.h.recv("CAMERA_CAPTURE_STATUS", timeout=1) for _ in range(3)]
        self.assertTrue(all(m is not None for m in got), "capture status was not streamed")
        self.assertEqual(
            self.h.command_ack(mav.MAV_CMD_SET_MESSAGE_INTERVAL,
                               mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, -1),
            mav.MAV_RESULT_ACCEPTED)

    def test_11_unsupported_command(self):
        self.assertEqual(self.h.command_ack(mav.MAV_CMD_STORAGE_FORMAT, 1, 1),
                         mav.MAV_RESULT_UNSUPPORTED)

    def test_12_ignores_commands_for_other_components(self):
        # addressed to the autopilot, not the camera
        self.assertIsNone(self.h.command_ack(mav.MAV_CMD_VIDEO_START_CAPTURE, 0, 0, target_comp=1))
        self.assertFalse(self.h.state.snapshot()["recording"])
        # broadcast, and not a camera command: stay silent
        self.h.command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, target_comp=0)
        self.assertIsNone(self.h.recv("COMMAND_ACK", timeout=1))

    def test_13_requests_position_and_geotags(self):
        # on a camera link the FC streams nothing by default, so the camera must ask
        deadline = time.time() + 8
        req = None
        while time.time() < deadline and req is None:
            self.h.fc_heartbeat()
            self.h.recv("NONE", timeout=0.5)
            for c in self.h.sent_cmds:
                if (c.command == mav.MAV_CMD_SET_MESSAGE_INTERVAL
                        and int(c.param1) == mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
                    req = c
        self.assertIsNotNone(req, "camera never asked the autopilot for GLOBAL_POSITION_INT")
        self.assertEqual((req.target_system, req.target_component), (FC_SYSID, 1))

        self.h.fc.mav.global_position_int_send(1000, -353632610, 1491652300, 584000, 10000,
                                               0, 0, 0, 0)
        time.sleep(0.2)
        self.h.command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1)
        img = self.h.recv("CAMERA_IMAGE_CAPTURED")
        self.assertEqual((img.lat, img.lon, img.alt, img.relative_alt),
                         (-353632610, 1491652300, 584000, 10000))


class WaybeamTest(Common, unittest.TestCase):
    streamer_type = "waybeam"

    def test_20_zoom(self):
        self.assertEqual(self.h.command_ack(mav.MAV_CMD_SET_CAMERA_ZOOM, mav.ZOOM_TYPE_RANGE, 100),
                         mav.MAV_RESULT_ACCEPTED)
        self.assertEqual(self.h.state.snapshot()["fields"]["video0.framing"], "zoom-4x")
        self.assertEqual(self.h.command_ack(mav.MAV_CMD_SET_CAMERA_ZOOM, mav.ZOOM_TYPE_STEP, -1),
                         mav.MAV_RESULT_ACCEPTED)
        self.assertEqual(self.h.state.snapshot()["fields"]["video0.framing"], "zoom-3x")
        cs = self.h.request(mav.MAVLINK_MSG_ID_CAMERA_SETTINGS, "CAMERA_SETTINGS")
        self.assertAlmostEqual(cs.zoomLevel, 100.0 * 5 / 6, places=2)
        self.assertEqual(self.h.command_ack(mav.MAV_CMD_SET_CAMERA_ZOOM, mav.ZOOM_TYPE_RANGE, 0),
                         mav.MAV_RESULT_ACCEPTED)
        self.assertEqual(self.h.state.snapshot()["fields"]["video0.framing"], "off")
        self.assertEqual(self.h.command_ack(mav.MAV_CMD_SET_CAMERA_ZOOM, mav.ZOOM_TYPE_RANGE, 150),
                         mav.MAV_RESULT_DENIED)

    def test_21_record_failure_is_reported(self):
        self.h.state.fail_record = True
        try:
            self.assertEqual(self.h.command_ack(mav.MAV_CMD_VIDEO_START_CAPTURE, 0, 0),
                             mav.MAV_RESULT_FAILED)
        finally:
            self.h.state.fail_record = False
        st = self.h.request(mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, "CAMERA_CAPTURE_STATUS")
        self.assertEqual(st.video_status, 0)


class MajesticTest(Common, unittest.TestCase):
    streamer_type = "majestic"

    def test_21_paused_recorder_is_reported_as_not_recording(self):
        # majestic can be recording-enabled but deliberately "stood down"
        self.assertEqual(self.h.command_ack(mav.MAV_CMD_VIDEO_START_CAPTURE, 0, 0), mav.MAV_RESULT_ACCEPTED)
        self.h.state.stood_down = True
        try:
            st = self.h.recv("CAMERA_CAPTURE_STATUS", timeout=4, condition=lambda m: m.video_status == 0)
            self.assertIsNotNone(st, "a paused recorder was still reported as recording")
        finally:
            self.h.state.stood_down = False
            self.h.command_ack(mav.MAV_CMD_VIDEO_STOP_CAPTURE, 0)

    def test_20_zoom_unsupported(self):
        self.assertEqual(self.h.command_ack(mav.MAV_CMD_SET_CAMERA_ZOOM, mav.ZOOM_TYPE_RANGE, 50),
                         mav.MAV_RESULT_UNSUPPORTED)


class AutoSizeTest(unittest.TestCase):
    """waybeam's default size is "auto"; the resolution then comes from its runtime state."""

    def test_resolution_when_size_is_auto(self):
        h = Harness("waybeam", fields={"video0.size": "auto"})
        try:
            vi = None
            deadline = time.time() + 8
            while time.time() < deadline and (vi is None or vi.resolution_h != 2560):
                h.fc_heartbeat()
                vi = h.request(mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, "VIDEO_STREAM_INFORMATION")
                time.sleep(0.3)
            self.assertIsNotNone(vi)
            self.assertEqual((vi.resolution_h, vi.resolution_v), (2560, 1440))
        finally:
            h.close()


class NoAutopilotTest(unittest.TestCase):
    """With no flight controller on the link, mavcam holds its heartbeat for a few
    seconds (so it never announces under a guessed system ID while one is booting),
    then announces as system 1 so it still works standalone."""

    def test_heartbeat_is_held_then_sent(self):
        server, _ = mock_streamer.serve(0)
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        with tempfile.TemporaryDirectory() as media:
            proc = subprocess.Popen([os.path.join(ROOT, "mavcam"), "--link", f"udpin:{port}",
                                     "--http", f"127.0.0.1:{server.server_address[1]}",
                                     "--media-dir", media])
            # a ground station, not an autopilot: enough for mavcam to learn where to send
            gcs = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=255,
                                             source_component=190)
            try:
                start = time.time()
                first = None
                while time.time() - start < 9 and first is None:
                    gcs.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    hb = gcs.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
                    if hb is not None and hb.get_srcComponent() == CAM_COMPID:
                        first = (time.time() - start, hb)
                self.assertIsNotNone(first, "camera never announced itself without an autopilot")
                waited, hb = first
                self.assertGreater(waited, 3.0, "camera announced before giving an autopilot time to appear")
                self.assertEqual(hb.get_srcSystem(), 1)
            finally:
                proc.terminate()
                proc.wait(timeout=5)
                gcs.close()
                server.shutdown()


if __name__ == "__main__":
    unittest.main(verbosity=2)
