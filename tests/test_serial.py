#!/usr/bin/env python3
"""The serial transport, on a pseudo-terminal throttled to 115200 baud.

    fc (pymavlink, UDP) <--> bridge thread <--> pty <--> mavcam --link /dev/ttysNNN:115200

The bridge drains mavcam's output no faster than a real 115200 UART would, so
the port backs up exactly as it would on a camera.
"""
import logging
import os
import pty
import select
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

os.environ["MAVLINK20"] = "1"
from pymavlink import mavftp, mavutil  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "tests"))
import mock_streamer  # noqa: E402
from test_features import ext_value  # noqa: E402

mav = mavutil.mavlink
FC_SYSID = 7
CAM = 100
BYTES_PER_SEC = 115200 // 10     # 8N1: ten bits on the wire per byte
logging.disable(logging.CRITICAL)


class Bridge(threading.Thread):
    """Shuttle bytes between the pty master and a UDP socket, at UART speed."""

    def __init__(self, master_fd):
        super().__init__(daemon=True)
        self.master = master_fd
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.port = self.sock.getsockname()[1]
        self.peer = None    # learned when the fake FC first speaks, as a real one would
        self.stop = False
        self.from_cam = 0

    def run(self):
        tick = 0.01
        budget = 0.0
        last = time.time()
        while not self.stop:
            now = time.time()
            budget = min(budget + (now - last) * BYTES_PER_SEC, BYTES_PER_SEC * 0.05)
            last = now
            want = [self.sock] + ([self.master] if budget >= 1 else [])
            r, _, _ = select.select(want, [], [], tick)
            if self.master in r:
                data = os.read(self.master, int(budget))
                budget -= len(data)
                self.from_cam += len(data)
                if self.peer is not None:
                    self.sock.sendto(data, self.peer)
            if self.sock in r:
                try:
                    data, self.peer = self.sock.recvfrom(4096)
                    os.write(self.master, data)
                except OSError:
                    pass


class SerialTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server, cls.state = mock_streamer.serve(0)
        cls.tmp = tempfile.TemporaryDirectory()
        cls.blob = os.urandom(60_000)
        with open(os.path.join(cls.tmp.name, "photo.jpg"), "wb") as f:
            f.write(cls.blob)
        cls.master, slave = pty.openpty()
        cls.bridge = Bridge(cls.master)
        cls.bridge.start()
        cls.fc = mavutil.mavlink_connection(f"udpout:127.0.0.1:{cls.bridge.port}", source_system=FC_SYSID,
                                            source_component=1)
        cls.proc = subprocess.Popen([
            os.path.join(ROOT, "mavcam"),
            "--link", f"{os.ttyname(slave)}:115200",
            "--http", f"127.0.0.1:{cls.server.server_address[1]}",
            "--media-dir", cls.tmp.name,
            # no rate ceiling to speak of: only the port backing up may pace the transfer
            "--ftp-rate", "1000000",
        ])
        os.close(slave)
        deadline = time.time() + 6
        ok = False
        while time.time() < deadline and not ok:
            hb = cls.fc.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
            cls.fc.mav.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
            ok = hb is not None and hb.get_srcSystem() == FC_SYSID and hb.get_srcComponent() == CAM
        if not ok:
            cls.tearDownClass()
            raise AssertionError("no camera heartbeat over the serial link")

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(timeout=5)
        cls.bridge.stop = True
        cls.fc.close()
        cls.server.shutdown()
        cls.tmp.cleanup()

    def recv(self, mtype, timeout=5.0, condition=None):
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = self.fc.recv_match(type=mtype, blocking=True, timeout=0.2)
            if m is not None and (condition is None or condition(m)):
                return m
        return None

    def test_01_request_reply(self):
        self.fc.mav.command_long_send(FC_SYSID, CAM, mav.MAV_CMD_REQUEST_MESSAGE, 0,
                                      mav.MAVLINK_MSG_ID_CAMERA_INFORMATION, 0, 0, 0, 0, 0, 0)
        ci = self.recv("CAMERA_INFORMATION")
        self.assertIsNotNone(ci)
        self.assertEqual(bytes(ci.vendor_name).rstrip(b"\0"), b"OpenIPC")

    def test_02_record(self):
        self.fc.mav.command_long_send(FC_SYSID, CAM, mav.MAV_CMD_VIDEO_START_CAPTURE, 0, 0, 0, 0, 0, 0, 0, 0)
        ack = self.recv("COMMAND_ACK", condition=lambda m: m.command == mav.MAV_CMD_VIDEO_START_CAPTURE)
        self.assertEqual(ack.result, mav.MAV_RESULT_ACCEPTED)
        self.assertTrue(self.state.snapshot()["recording"])
        self.fc.mav.command_long_send(FC_SYSID, CAM, mav.MAV_CMD_VIDEO_STOP_CAPTURE, 0, 0, 0, 0, 0, 0, 0, 0)
        self.recv("COMMAND_ACK", condition=lambda m: m.command == mav.MAV_CMD_VIDEO_STOP_CAPTURE)
        self.assertFalse(self.state.snapshot()["recording"])

    def test_03_port_is_binary_clean_inbound(self):
        # Bytes a terminal in "cooked" mode would eat or rewrite: XON/XOFF
        # (0x11, 0x13), CR/LF (0x0d, 0x0a), Ctrl-C (0x03). If the port were not
        # raw, these frames would arrive corrupted and never be acknowledged.
        for name, value in (("BITRATE", 0x1311), ("GAIN_MAX", 0x0D0A03)):
            self.fc.mav.param_ext_set_send(FC_SYSID, CAM, name.encode(),
                                           struct.pack("<I", value).ljust(128, b"\0"),
                                           mav.MAV_PARAM_EXT_TYPE_UINT32)
            ack = self.recv("PARAM_EXT_ACK", condition=lambda m: m.param_id == name)
            self.assertIsNotNone(ack, f"{name}=0x{value:x} was not acknowledged")
            self.assertEqual(ack.param_result, mav.PARAM_ACK_ACCEPTED)
            self.assertEqual(ext_value(ack), value)

    def test_04_file_transfer_at_uart_speed(self):
        # 60 kB of random data: every byte value, outbound, with the port backed up
        errors_before = self.fc.mav.total_receive_errors
        sent_before = self.bridge.from_cam
        local = os.path.join(self.tmp.name, "download")
        client = mavftp.MAVFTP(self.fc, FC_SYSID, CAM)
        client.ftp_settings.burst_read_size = 239
        start = time.time()
        client.cmd_get(["/photo.jpg", local])
        ret = client.process_ftp_reply("OpenFileRO", timeout=90)
        took = time.time() - start
        self.assertEqual(ret.return_code, 0)
        with open(local, "rb") as f:
            self.assertEqual(f.read(), self.blob)
        wire = self.bridge.from_cam - sent_before
        # nothing truncated on the wire, and nothing wastefully re-sent
        self.assertEqual(self.fc.mav.total_receive_errors, errors_before,
                         "corrupt frames arrived: something was written partially")
        self.assertLess(wire, len(self.blob) * 1.25,
                        f"{wire} bytes on the wire for a {len(self.blob)} byte file")
        # sanity check on the throttle itself: it cannot have gone faster than the UART
        self.assertGreater(took, len(self.blob) / BYTES_PER_SEC)

    def test_05_camera_stays_responsive_during_transfer(self):
        local = os.path.join(self.tmp.name, "download2")
        client = mavftp.MAVFTP(self.fc, FC_SYSID, CAM)
        client.ftp_settings.burst_read_size = 239
        client.cmd_get(["/photo.jpg", local])
        time.sleep(0.5)   # transfer under way, port saturated
        start = time.time()
        self.fc.mav.command_long_send(FC_SYSID, CAM, mav.MAV_CMD_REQUEST_MESSAGE, 0,
                                      mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, 0, 0, 0, 0, 0, 0)
        # read with the FTP client so it keeps consuming its own packets
        ack_at = None
        deadline = time.time() + 5
        while time.time() < deadline and ack_at is None:
            m = self.fc.recv_match(type=["COMMAND_ACK", "FILE_TRANSFER_PROTOCOL"], blocking=True, timeout=0.2)
            if m is not None and m.get_type() == "COMMAND_ACK":
                ack_at = time.time()
        self.assertIsNotNone(ack_at, "command was not acknowledged while a file transfer was running")
        self.assertLess(ack_at - start, 1.0, "command reply was stuck behind file-transfer data")
        client.cmd_cancel()
        time.sleep(1.5)
        while self.fc.recv_match(blocking=False) is not None:
            pass


if __name__ == "__main__":
    unittest.main(verbosity=2)
