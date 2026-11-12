#!/usr/bin/env python3
"""Tests for forwarding, COMMAND_INT, camera settings (PARAM_EXT) and MAVLink FTP.

mavcam sits between a fake flight controller and a fake ground station:

    fc (pymavlink)  <--UDP-->  mavcam  <--UDP-->  gcs (pymavlink)
"""
import logging
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET

os.environ["MAVLINK20"] = "1"
from pymavlink import mavftp, mavutil  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import mock_streamer  # noqa: E402

mav = mavutil.mavlink
FC_SYSID = 7
CAM = 100
logging.disable(logging.CRITICAL)   # mavftp is chatty


def free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def raw_value(msg, offset):
    """The 128 raw bytes of a PARAM_EXT value (pymavlink would cut it at the first NUL)."""
    payload = bytes(msg.get_msgbuf())[10:10 + msg.get_header().mlen]
    return payload.ljust(offset + 128, b"\0")[offset:offset + 128]


def ext_value(msg):
    raw = raw_value(msg, 20 if msg.get_type() == "PARAM_EXT_VALUE" else 16)
    fmt = {mav.MAV_PARAM_EXT_TYPE_UINT8: "<B", mav.MAV_PARAM_EXT_TYPE_UINT32: "<I",
           mav.MAV_PARAM_EXT_TYPE_INT32: "<i", mav.MAV_PARAM_EXT_TYPE_REAL32: "<f"}[msg.param_type]
    return struct.unpack_from(fmt, raw)[0]


class FeatureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server, cls.state = mock_streamer.serve(0)
        cls.tmp = tempfile.TemporaryDirectory()
        cls.media = os.path.join(cls.tmp.name, "media")
        os.mkdir(cls.media)
        cls.clip = os.urandom(100_000)
        with open(os.path.join(cls.media, "clip.ts"), "wb") as f:
            f.write(cls.clip)
        os.mkdir(os.path.join(cls.media, "sub"))

        # a setting backed by shell commands, standing in for e.g. radio TX power
        cls.txp_file = os.path.join(cls.tmp.name, "txpower")
        with open(cls.txp_file, "w") as f:
            f.write("20\n")
        setter = os.path.join(cls.tmp.name, "set-txpower")
        with open(setter, "w") as f:
            f.write(f"#!/bin/sh\necho \"$1\" > {cls.txp_file}\n")
        os.chmod(setter, 0o755)

        # like a real autopilot, the fake FC talks first: mavcam listens, the FC sends to it
        fc_port = free_udp_port()
        cls.fc = mavutil.mavlink_connection(f"udpout:127.0.0.1:{fc_port}", source_system=FC_SYSID,
                                            source_component=1)
        cls.gcs = mavutil.mavlink_connection("udpin:127.0.0.1:0", source_system=255, source_component=190)
        cls.ground_in = free_udp_port()
        cls.proc = subprocess.Popen([
            os.path.join(ROOT, "mavcam"),
            "--link", f"udpin:{fc_port}",
            "--ground-out", f"127.0.0.1:{cls.gcs.port.getsockname()[1]}",
            "--ground-in", str(cls.ground_in),
            "--http", f"127.0.0.1:{cls.server.server_address[1]}",
            "--media-dir", cls.media,
            "--model", "Test<Cam>",
            "--exec-param", f"TXPOWER:cat {cls.txp_file}:{setter}",
            "--ftp-rate", "100000",
        ])
        # let the camera find the autopilot and both sides learn each other's address
        deadline = time.time() + 5
        seen = False
        while time.time() < deadline and not seen:
            cls.fc.recv_match(blocking=True, timeout=0.2)
            cls.fc.mav.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
            hb = cls.gcs.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            seen = hb is not None and hb.get_srcSystem() == FC_SYSID and hb.get_srcComponent() == CAM
        if not seen:
            cls.tearDownClass()
            raise AssertionError("camera heartbeat did not reach the ground side")
        cls.gcs.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(timeout=5)
        cls.fc.close()
        cls.gcs.close()
        cls.server.shutdown()
        cls.tmp.cleanup()

    def recv(self, conn, mtype, timeout=3.0, condition=None):
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = conn.recv_match(type=mtype, blocking=True, timeout=0.2)
            if m is not None and (condition is None or condition(m)):
                return m
        return None

    def drain(self, conn):
        while conn.recv_match(blocking=False) is not None:
            pass

    # ---- forwarding ----

    def test_01_fc_telemetry_reaches_ground(self):
        self.fc.mav.attitude_send(1234, 0.1, 0.2, 0.3, 0, 0, 0)
        m = self.recv(self.gcs, "ATTITUDE")
        self.assertIsNotNone(m, "FC telemetry was not forwarded to the ground link")
        self.assertEqual((m.get_srcSystem(), m.get_srcComponent(), m.time_boot_ms), (FC_SYSID, 1, 1234))

    def test_02_messages_unknown_to_mavcam_are_forwarded(self):
        # AHRS is ArduPilot-specific: not in the common dialect mavcam is built with
        self.fc.mav.ahrs_send(1, 2, 3, 0, 0, 4, 5)
        m = self.recv(self.gcs, "AHRS")
        self.assertIsNotNone(m, "an ArduPilot-dialect message was dropped instead of forwarded")
        self.assertAlmostEqual(m.omegaIx, 1.0)

    def test_03_ground_commands_reach_fc(self):
        self.gcs.mav.command_long_send(FC_SYSID, 1, mav.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        m = self.recv(self.fc, "COMMAND_LONG", condition=lambda m: m.get_srcSystem() == 255)
        self.assertIsNotNone(m, "a ground command for the autopilot was not forwarded")
        self.assertEqual(m.command, mav.MAV_CMD_COMPONENT_ARM_DISARM)

    def test_04_camera_commands_are_not_passed_to_fc(self):
        self.drain(self.fc)
        self.gcs.mav.command_long_send(FC_SYSID, CAM, mav.MAV_CMD_REQUEST_MESSAGE, 0,
                                       mav.MAVLINK_MSG_ID_STORAGE_INFORMATION, 0, 0, 0, 0, 0, 0)
        ack = self.recv(self.gcs, "COMMAND_ACK")
        self.assertIsNotNone(ack)
        self.assertEqual((ack.get_srcComponent(), ack.result), (CAM, mav.MAV_RESULT_ACCEPTED))
        leaked = self.recv(self.fc, ["COMMAND_LONG", "COMMAND_ACK"], timeout=1,
                           condition=lambda m: m.get_srcSystem() == 255 or m.get_type() == "COMMAND_ACK")
        self.assertIsNone(leaked, f"camera-only traffic reached the FC: {leaked}")

    def test_05_separate_ground_in_port(self):
        # wfb-ng style: a different local port for the uplink
        up = mavutil.mavlink_connection(f"udpout:127.0.0.1:{self.ground_in}", source_system=254,
                                        source_component=190)
        try:
            up.mav.command_long_send(FC_SYSID, 1, mav.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)
            m = self.recv(self.fc, "COMMAND_LONG", condition=lambda m: m.get_srcSystem() == 254)
            self.assertIsNotNone(m, "uplink on --ground-in was not forwarded to the FC")
        finally:
            up.close()

    def test_06_slow_streamer_does_not_stall_forwarding(self):
        # every streamer call takes 0.7 s; telemetry must keep flowing meanwhile
        self.state.delay = 0.7
        try:
            self.drain(self.gcs)
            self.gcs.mav.command_long_send(FC_SYSID, CAM, mav.MAV_CMD_VIDEO_START_CAPTURE, 0,
                                           0, 0, 0, 0, 0, 0, 0)
            arrivals, ack = [], None
            start = time.time()
            n = 0
            while time.time() - start < 6 and (ack is None or time.time() - start < 2):
                self.fc.mav.attitude_send(n, 0, 0, 0, 0, 0, 0)
                n += 1
                end = time.time() + 0.05
                while time.time() < end:
                    m = self.gcs.recv_match(type=["ATTITUDE", "COMMAND_ACK"], blocking=True, timeout=0.02)
                    if m is None:
                        continue
                    if m.get_type() == "ATTITUDE":
                        arrivals.append(time.time())
                    elif m.command == mav.MAV_CMD_VIDEO_START_CAPTURE:
                        ack = m
            self.assertIsNotNone(ack, "record command was never acknowledged")
            self.assertEqual(ack.result, mav.MAV_RESULT_ACCEPTED)
            self.assertGreater(len(arrivals), 20)
            worst = max(b - a for a, b in zip(arrivals, arrivals[1:]))
            self.assertLess(worst, 0.3, f"forwarding stalled for {worst:.2f} s while the streamer was slow")
        finally:
            self.state.delay = 0.0
            self.gcs.mav.command_long_send(FC_SYSID, CAM, mav.MAV_CMD_VIDEO_STOP_CAPTURE, 0,
                                           0, 0, 0, 0, 0, 0, 0)
            self.recv(self.gcs, "COMMAND_ACK", timeout=8,
                      condition=lambda m: m.command == mav.MAV_CMD_VIDEO_STOP_CAPTURE)

    def test_07_continuous_zoom(self):
        def framing():
            return self.state.snapshot()["fields"]["video0.framing"]
        self.assertEqual(framing(), "off")
        self.gcs.mav.command_long_send(FC_SYSID, CAM, mav.MAV_CMD_SET_CAMERA_ZOOM, 0,
                                       mav.ZOOM_TYPE_CONTINUOUS, 1, 0, 0, 0, 0, 0)
        ack = self.recv(self.gcs, "COMMAND_ACK", condition=lambda m: m.command == mav.MAV_CMD_SET_CAMERA_ZOOM)
        self.assertEqual(ack.result, mav.MAV_RESULT_ACCEPTED)
        deadline = time.time() + 6
        while time.time() < deadline and framing() in ("off", "zoom-1.25x"):
            time.sleep(0.1)
        self.assertNotIn(framing(), ("off", "zoom-1.25x"), "zoom did not keep stepping in")
        # stop, and check it really stops
        self.gcs.mav.command_long_send(FC_SYSID, CAM, mav.MAV_CMD_SET_CAMERA_ZOOM, 0,
                                       mav.ZOOM_TYPE_CONTINUOUS, 0, 0, 0, 0, 0, 0)
        self.recv(self.gcs, "COMMAND_ACK", condition=lambda m: m.command == mav.MAV_CMD_SET_CAMERA_ZOOM)
        time.sleep(1.2)
        held = framing()
        time.sleep(1.5)
        self.assertEqual(framing(), held, "zoom kept moving after stop")
        # zoom all the way out; it must stop by itself at the wide end
        self.gcs.mav.command_long_send(FC_SYSID, CAM, mav.MAV_CMD_SET_CAMERA_ZOOM, 0,
                                       mav.ZOOM_TYPE_CONTINUOUS, -1, 0, 0, 0, 0, 0)
        deadline = time.time() + 10
        while time.time() < deadline and framing() != "off":
            time.sleep(0.1)
        self.assertEqual(framing(), "off")

    # ---- COMMAND_INT ----

    def test_10_command_int(self):
        self.gcs.mav.command_int_send(FC_SYSID, CAM, 0, mav.MAV_CMD_VIDEO_START_CAPTURE, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0)
        ack = self.recv(self.gcs, "COMMAND_ACK", condition=lambda m: m.command == mav.MAV_CMD_VIDEO_START_CAPTURE)
        self.assertIsNotNone(ack, "COMMAND_INT was not acknowledged")
        self.assertEqual(ack.result, mav.MAV_RESULT_ACCEPTED)
        self.assertTrue(self.state.snapshot()["recording"])
        self.gcs.mav.command_int_send(FC_SYSID, CAM, 0, mav.MAV_CMD_VIDEO_STOP_CAPTURE, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0)
        self.recv(self.gcs, "COMMAND_ACK", condition=lambda m: m.command == mav.MAV_CMD_VIDEO_STOP_CAPTURE)
        self.assertFalse(self.state.snapshot()["recording"])

    # ---- settings ----

    def set_param(self, name, ptype, packed):
        self.gcs.mav.param_ext_set_send(FC_SYSID, CAM, name.encode(), packed.ljust(128, b"\0"), ptype)
        return self.recv(self.gcs, "PARAM_EXT_ACK", condition=lambda m: m.param_id == name)

    def test_20_param_list(self):
        self.drain(self.gcs)
        self.gcs.mav.param_ext_request_list_send(FC_SYSID, CAM)
        got = {}
        deadline = time.time() + 5
        while time.time() < deadline:
            m = self.recv(self.gcs, "PARAM_EXT_VALUE", timeout=1)
            if m is None:
                break
            got[m.param_id] = (ext_value(m), m.param_count, m.param_index)
            if len(got) == m.param_count:
                break
        self.assertEqual(len(got), 12, sorted(got))
        self.assertEqual(sorted(i for _, _, i in got.values()), list(range(12)))
        self.assertEqual(got["BITRATE"][0], 8192)
        self.assertEqual(got["FPS"][0], 60)
        self.assertEqual(got["AWB_CT"][0], 5500)
        self.assertEqual(got["MIRROR"][0], 0)
        self.assertEqual(got["TXPOWER"][0], 20)

    def test_21_param_set_number(self):
        ack = self.set_param("BITRATE", mav.MAV_PARAM_EXT_TYPE_UINT32, struct.pack("<I", 4096))
        self.assertIsNotNone(ack)
        self.assertEqual(ack.param_result, mav.PARAM_ACK_ACCEPTED)
        self.assertEqual(ext_value(ack), 4096)
        self.assertEqual(self.state.snapshot()["fields"]["video0.bitrate"], 4096)

    def test_22_param_set_enum_bool_float_negative(self):
        self.assertEqual(self.set_param("RES", mav.MAV_PARAM_EXT_TYPE_UINT32, struct.pack("<I", 1)).param_result,
                         mav.PARAM_ACK_ACCEPTED)
        self.assertEqual(self.set_param("MIRROR", mav.MAV_PARAM_EXT_TYPE_UINT8, b"\x01").param_result,
                         mav.PARAM_ACK_ACCEPTED)
        self.assertEqual(self.set_param("GOP", mav.MAV_PARAM_EXT_TYPE_REAL32, struct.pack("<f", 0.5)).param_result,
                         mav.PARAM_ACK_ACCEPTED)
        self.assertEqual(self.set_param("QP_DELTA", mav.MAV_PARAM_EXT_TYPE_INT32, struct.pack("<i", -4)).param_result,
                         mav.PARAM_ACK_ACCEPTED)
        fields = self.state.snapshot()["fields"]
        self.assertEqual(fields["video0.size"], "1280x720")
        # and it reads back as the option that was set, in the form the streamer reports
        self.gcs.mav.param_ext_request_read_send(FC_SYSID, CAM, b"RES", -1)
        m = self.recv(self.gcs, "PARAM_EXT_VALUE", condition=lambda m: m.param_id == "RES")
        self.assertEqual(ext_value(m), 1)
        self.assertIs(fields["image.mirror"], True)
        self.assertEqual(fields["video0.gop_size"], 0.5)
        self.assertEqual(fields["video0.qp_delta"], -4)

    def test_23_param_set_rejected(self):
        before = self.state.snapshot()["fields"]["video0.fps"]
        # out of range
        ack = self.set_param("FPS", mav.MAV_PARAM_EXT_TYPE_UINT32, struct.pack("<I", 100000))
        self.assertEqual(ack.param_result, mav.PARAM_ACK_VALUE_UNSUPPORTED)
        self.assertEqual(ext_value(ack), before, "a rejected set should report the value still in force")
        # wrong type
        ack = self.set_param("FPS", mav.MAV_PARAM_EXT_TYPE_REAL32, struct.pack("<f", 30.0))
        self.assertEqual(ack.param_result, mav.PARAM_ACK_VALUE_UNSUPPORTED)
        # enum index past the end
        ack = self.set_param("RES", mav.MAV_PARAM_EXT_TYPE_UINT32, struct.pack("<I", 9))
        self.assertEqual(ack.param_result, mav.PARAM_ACK_VALUE_UNSUPPORTED)
        # unknown name
        ack = self.set_param("NOPE", mav.MAV_PARAM_EXT_TYPE_UINT32, struct.pack("<I", 1))
        self.assertEqual(ack.param_result, mav.PARAM_ACK_VALUE_UNSUPPORTED)
        self.assertEqual(self.state.snapshot()["fields"]["video0.fps"], before)

    def test_24_param_read_by_name_sees_outside_changes(self):
        with self.state.lock:
            self.state.fields["isp.awb_ct"] = 6500
        self.gcs.mav.param_ext_request_read_send(FC_SYSID, CAM, b"AWB_CT", -1)
        m = self.recv(self.gcs, "PARAM_EXT_VALUE", condition=lambda m: m.param_id == "AWB_CT")
        self.assertEqual(ext_value(m), 6500)

    def test_25_exec_param(self):
        ack = self.set_param("TXPOWER", mav.MAV_PARAM_EXT_TYPE_INT32, struct.pack("<i", 25))
        self.assertEqual(ack.param_result, mav.PARAM_ACK_ACCEPTED)
        with open(self.txp_file) as f:
            self.assertEqual(f.read().strip(), "25")
        self.gcs.mav.param_ext_request_read_send(FC_SYSID, CAM, b"TXPOWER", -1)
        m = self.recv(self.gcs, "PARAM_EXT_VALUE", condition=lambda m: m.param_id == "TXPOWER")
        self.assertEqual(ext_value(m), 25)

    # ---- definition file and FTP ----

    def ftp(self):
        return mavftp.MAVFTP(self.gcs, FC_SYSID, CAM)

    def ftp_get(self, remote):
        local = os.path.join(self.tmp.name, "download")
        if os.path.exists(local):
            os.remove(local)
        client = self.ftp()
        client.cmd_get([remote, local])
        ret = client.process_ftp_reply("OpenFileRO", timeout=30)
        self.assertEqual(ret.return_code, 0, f"get {remote}: {ret.return_code}")
        with open(local, "rb") as f:
            return f.read()

    def test_30_camera_information_points_at_definition(self):
        self.gcs.mav.command_long_send(FC_SYSID, CAM, mav.MAV_CMD_REQUEST_MESSAGE, 0,
                                       mav.MAVLINK_MSG_ID_CAMERA_INFORMATION, 0, 0, 0, 0, 0, 0)
        ci = self.recv(self.gcs, "CAMERA_INFORMATION")
        self.assertEqual(ci.cam_definition_uri, "mftp://[;comp=100]/camdef.xml")
        self.assertEqual(ci.cam_definition_version, 1)

    def test_31_definition_file_over_ftp(self):
        xml = self.ftp_get("/camdef.xml")
        root = ET.fromstring(xml)   # must be well-formed, including the escaped model name
        self.assertEqual(root.tag, "mavlinkcamera")
        self.assertEqual(root.find("definition/model").text, "Test<Cam>")
        params = {p.get("name"): p for p in root.iter("parameter")}
        self.assertEqual(len(params), 12)
        self.assertEqual(params["BITRATE"].get("type"), "uint32")
        self.assertEqual(params["MIRROR"].get("type"), "bool")
        self.assertEqual([o.get("name") for o in params["RES"].iter("option")], ["auto", "720p", "1080p"])
        self.assertEqual(params["TXPOWER"].get("type"), "int32")

    def test_31b_comp_selector_in_path_is_tolerated(self):
        self.assertTrue(self.ftp_get("[;comp=100]/camdef.xml").startswith(b"<?xml"))
        self.assertTrue(self.ftp_get("/[;comp=100]camdef.xml").startswith(b"<?xml"))

    def test_32_download_media_file(self):
        self.assertEqual(self.ftp_get("/clip.ts"), self.clip)

    def test_33_list_directory(self):
        client = self.ftp()
        ret = client.cmd_list(["/"])
        self.assertEqual(ret.return_code, 0)
        entries = {e.name: e for e in client.list_result}
        self.assertIn("camdef.xml", entries)
        self.assertIn("clip.ts", entries)
        self.assertEqual(entries["clip.ts"].size_b, len(self.clip))
        self.assertTrue(entries["sub"].is_dir)

    def test_34_cannot_escape_media_directory(self):
        secret = os.path.join(self.tmp.name, "secret")
        with open(secret, "w") as f:
            f.write("nope")
        for path in ("../secret", "/../secret", "sub/../../secret"):
            client = self.ftp()
            client.cmd_get([path, os.path.join(self.tmp.name, "stolen")])
            ret = client.process_ftp_reply("OpenFileRO", timeout=5)
            self.assertNotEqual(ret.return_code, 0, f"{path} was served")
        self.assertFalse(os.path.exists(os.path.join(self.tmp.name, "stolen")) and
                         open(os.path.join(self.tmp.name, "stolen")).read() == "nope")

    def test_35_ftp_does_not_flood_the_fc_link(self):
        self.drain(self.fc)
        self.ftp_get("/clip.ts")
        leaked = self.recv(self.fc, "FILE_TRANSFER_PROTOCOL", timeout=1)
        self.assertIsNone(leaked, "FTP data for the ground station was also sent to the FC")


if __name__ == "__main__":
    unittest.main(verbosity=2)
