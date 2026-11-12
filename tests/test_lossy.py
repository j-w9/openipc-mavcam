#!/usr/bin/env python3
"""The ground link behaving like a marginal radio: 10 % of packets lost in each
direction, and some delivered late and out of order.

    fc (pymavlink) <--UDP--> mavcam <--UDP--> lossy proxy <--UDP--> gcs (pymavlink)
"""
import logging
import os
import random
import socket
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
import mock_streamer  # noqa: E402

mav = mavutil.mavlink
FC_SYSID = 7
CAM = 100
LOSS = 0.10
LATE = 0.05          # fraction held back and delivered after later packets
logging.disable(logging.CRITICAL)


def free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class LossyProxy(threading.Thread):
    """Sits where the radio would be. mavcam sends to `cam_side`; the GCS talks to `gcs_side`."""

    def __init__(self):
        super().__init__(daemon=True)
        self.rng = random.Random(1234)   # same losses every run
        self.cam_side = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cam_side.bind(("127.0.0.1", 0))
        self.gcs_side = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gcs_side.bind(("127.0.0.1", 0))
        self.cam_peer = None
        self.gcs_peer = None
        self.stop = False
        self.dropped = self.passed = 0

    def relay(self, data, sock, peer):
        if peer is None or self.rng.random() < LOSS:
            self.dropped += 1
            return
        self.passed += 1
        if self.rng.random() < LATE:
            threading.Timer(0.15, lambda: sock.sendto(data, peer)).start()
        else:
            sock.sendto(data, peer)

    def run(self):
        import select
        while not self.stop:
            r, _, _ = select.select([self.cam_side, self.gcs_side], [], [], 0.1)
            if self.cam_side in r:
                data, self.cam_peer = self.cam_side.recvfrom(4096)
                self.relay(data, self.gcs_side, self.gcs_peer)
            if self.gcs_side in r:
                data, self.gcs_peer = self.gcs_side.recvfrom(4096)
                self.relay(data, self.cam_side, self.cam_peer)


class LossyLinkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server, cls.state = mock_streamer.serve(0)
        cls.tmp = tempfile.TemporaryDirectory()
        cls.blob = os.urandom(80_000)
        with open(os.path.join(cls.tmp.name, "photo.jpg"), "wb") as f:
            f.write(cls.blob)
        cls.proxy = LossyProxy()
        cls.proxy.start()
        fc_port = free_udp_port()
        cls.proc = subprocess.Popen([
            os.path.join(ROOT, "mavcam"), "--link", f"udpin:{fc_port}",
            "--ground-out", f"127.0.0.1:{cls.proxy.cam_side.getsockname()[1]}",
            "--http", f"127.0.0.1:{cls.server.server_address[1]}",
            "--media-dir", cls.tmp.name, "--ftp-rate", "60000",
        ])
        cls.fc = mavutil.mavlink_connection(f"udpout:127.0.0.1:{fc_port}", source_system=FC_SYSID,
                                            source_component=1)
        cls.gcs = mavutil.mavlink_connection(f"udpout:127.0.0.1:{cls.proxy.gcs_side.getsockname()[1]}",
                                             source_system=255, source_component=190)
        deadline = time.time() + 10
        seen = False
        while time.time() < deadline and not seen:
            cls.fc.mav.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
            cls.gcs.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            cls.fc.recv_match(blocking=False)
            hb = cls.gcs.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
            seen = hb is not None and hb.get_srcSystem() == FC_SYSID and hb.get_srcComponent() == CAM
        if not seen:
            cls.tearDownClass()
            raise AssertionError("camera heartbeat never crossed the lossy link")

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(timeout=5)
        cls.proxy.stop = True
        cls.fc.close()
        cls.gcs.close()
        cls.server.shutdown()
        cls.tmp.cleanup()

    def command_with_retries(self, cmd, *params, tries=8):
        """What a ground station does on a lossy link: resend until acknowledged."""
        p = list(params) + [0] * (7 - len(params))
        for attempt in range(tries):
            self.gcs.mav.command_long_send(FC_SYSID, CAM, cmd, attempt, *p)
            end = time.time() + 1.0
            while time.time() < end:
                m = self.gcs.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.2)
                if m is not None and m.command == cmd:
                    return m, attempt + 1
        return None, tries

    def test_01_commands_survive_with_retries(self):
        ack, tries = self.command_with_retries(mav.MAV_CMD_VIDEO_START_CAPTURE, 0, 0)
        self.assertIsNotNone(ack, "no acknowledgement in 8 attempts")
        self.assertEqual(ack.result, mav.MAV_RESULT_ACCEPTED)
        self.assertTrue(self.state.snapshot()["recording"])
        # a repeated start (because the first ACK was lost) must not restart the recording
        for _ in range(3):
            self.command_with_retries(mav.MAV_CMD_VIDEO_START_CAPTURE, 0, 0)
        self.assertEqual(self.state.snapshot()["record_starts"], 1,
                         "resent start commands restarted the recording")
        ack, _ = self.command_with_retries(mav.MAV_CMD_VIDEO_STOP_CAPTURE, 0)
        self.assertIsNotNone(ack)
        self.assertFalse(self.state.snapshot()["recording"])

    def test_02_file_transfer_completes_intact(self):
        local = os.path.join(self.tmp.name, "download")
        client = mavftp.MAVFTP(self.gcs, FC_SYSID, CAM)
        client.ftp_settings.burst_read_size = 239
        client.cmd_get(["/photo.jpg", local])
        ret = client.process_ftp_reply("OpenFileRO", timeout=120)
        self.assertEqual(ret.return_code, 0, f"transfer failed: {ret.return_code}")
        with open(local, "rb") as f:
            self.assertEqual(f.read(), self.blob)
        self.assertGreater(self.proxy.dropped, 20, "the proxy dropped too little to mean anything")

    def test_03_forwarding_keeps_working(self):
        got = 0
        for n in range(100):
            self.fc.mav.attitude_send(n, 0, 0, 0, 0, 0, 0)
            m = self.gcs.recv_match(type="ATTITUDE", blocking=True, timeout=0.05)
            got += m is not None
        while self.gcs.recv_match(type="ATTITUDE", blocking=True, timeout=0.3) is not None:
            got += 1
        # about 90 % should arrive; none should be duplicated
        self.assertGreater(got, 75)
        self.assertLessEqual(got, 100)


if __name__ == "__main__":
    unittest.main(verbosity=2)
