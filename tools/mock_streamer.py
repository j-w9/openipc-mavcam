#!/usr/bin/env python3
"""Stand-in for an OpenIPC streamer's HTTP API, for developing mavcam off-camera.

Serves the subset of the waybeam and majestic APIs that mavcam calls.
  python3 tools/mock_streamer.py [--port 8080]
GET /_state returns the mock's internal state as JSON (not a real endpoint).
"""
import argparse
import json
import os
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlsplit

# smallest valid-looking JPEG: SOI, a comment segment, EOI
JPEG = b"\xff\xd8\xff\xfe\x00\x10mock-streamer\x00\xff\xd9"


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.fields = {
            "video0.size": "1920x1080",
            "video0.fps": 60,
            "video0.bitrate": 8192,
            "video0.codec": "h265",
            "video0.framing": "off",
            "video0.gop_size": 1.0,
            "video0.gopSize": 1.0,
            "video0.qp_delta": 0,
            "fpv.roi_qp": 0,
            "isp.gain_max": 0,
            "isp.awb_mode": "auto",
            "isp.awb_ct": 5500,
            "image.mirror": False,
            "image.flip": False,
            "records.enabled": False,
        }
        self.recording = False
        self.record_starts = 0
        self.snapshots = 0
        self.fail_record = False
        self.delay = 0.0          # seconds to stall every request, to imitate a slow streamer
        self.hostile = False      # answer with malformed, oversized or truncated replies
        self.stood_down = False

    def snapshot(self):
        with self.lock:
            return {
                "fields": dict(self.fields),
                "recording": self.recording,
                "record_starts": self.record_starts,
                "snapshots": self.snapshots,
            }


def parse_value(text):
    if text in ("true", "false"):
        return text == "true"
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text


class Handler(BaseHTTPRequestHandler):
    state = None
    protocol_version = "HTTP/1.0"

    def log_message(self, fmt, *args):
        pass

    def reply(self, code, body, ctype="application/json; charset=UTF-8"):
        if not isinstance(body, bytes):
            body = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def ok(self, data):
        self.reply(200, {"ok": True, "data": data})

    def err(self, code, name, message):
        self.reply(code, {"ok": False, "error": {"code": name, "message": message}})

    def hostile_reply(self):
        """One of a set of replies a well-behaved streamer would never give."""
        kind = random.randrange(9)
        if kind == 0:
            return self.reply(200, b"")
        if kind == 1:
            return self.reply(200, os.urandom(random.randrange(1, 4000)))
        if kind == 2:
            return self.reply(200, b'{"ok":true,"data":{"field":"x","value":' + b"9" * 5000 + b"}}")
        if kind == 3:
            return self.reply(200, b'{"ok":true,"data":{"value":"' + b"A" * 70000 + b'"}}')
        if kind == 4:
            return self.reply(200, b'{"ok":true,"data":{"active":tru')
        if kind == 5:
            return self.reply(500, b"<html>" + b"x" * 100000)
        if kind == 6:
            return self.reply(200, b"\xff\xd8" + os.urandom(200000), "image/jpeg")
        if kind == 7:
            return self.reply(200, b'{"value":-1e999,"w":-5,"h":99999999999,"active":true,"elapsed_ms":-1}')
        self.wfile.write(b"HTTP/1.0 200 OK\r\nContent-Length: 100000\r\n\r\nshort")   # lies about its length
        self.close_connection = True

    def do_GET(self):
        st = self.state
        url = urlsplit(self.path)
        path, query = url.path, unquote(url.query)

        if st.delay:
            time.sleep(st.delay)

        if st.hostile:
            return self.hostile_reply()

        if path == "/metrics/records":
            with st.lock:
                text = ("# HELP records_enabled 1 when records.enabled is on, 0 when recording is switched off\n"
                        f"records_enabled {int(st.recording)}\n"
                        f"records_stood_down {int(st.stood_down)}\n"
                        "records_state 0\n")
            return self.reply(200, text.encode(), "text/plain")

        if path == "/_state":
            return self.reply(200, st.snapshot())

        if path in ("/api/v1/snapshot.jpg", "/image.jpg"):
            with st.lock:
                st.snapshots += 1
            return self.reply(200, JPEG, "image/jpeg")

        if path == "/api/v1/config":
            with st.lock:
                return self.ok({"config": {"video0": {"size": st.fields["video0.size"]}},
                                "runtime": {"active_precrop": {"x": 0, "y": 240, "w": 2560, "h": 1440},
                                            "vpe_taps": {"port0": ["main", "jpeg"], "port1": None}}})

        if path == "/api/v1/get":
            # waybeam: ?<field>   majestic: ?key=<field>
            field = query[4:] if query.startswith("key=") else query
            with st.lock:
                if field not in st.fields:
                    return self.err(404, "not_found", "unknown config field")
                return self.ok({"field": field, "value": st.fields[field]})

        if path == "/api/v1/set":
            with st.lock:
                for pair in filter(None, query.split("&")):
                    key, _, value = pair.partition("=")
                    if key not in st.fields:
                        return self.err(404, "not_found", "unknown config field")
                    st.fields[key] = parse_value(value)
                    if key == "video0.size":
                        # waybeam stores a size as numbers and always reports it as WxH
                        st.fields[key] = {"720p": "1280x720", "1080p": "1920x1080"}.get(value, value)
                    if key == "records.enabled":
                        if st.fields[key] and not st.recording:
                            st.record_starts += 1
                        st.recording = bool(st.fields[key])
            return self.ok({})

        if path in ("/api/v1/record/start", "/api/v1/record/stop"):
            start = path.endswith("start")
            with st.lock:
                if st.fail_record:
                    return self.err(500, "internal_error", "mock failure")
                if start and not st.recording:
                    st.record_starts += 1
                st.recording = start
            return self.ok({"action": "start" if start else "stop"})

        if path == "/api/v1/record/status":
            with st.lock:
                return self.ok({"active": st.recording, "format": "ts",
                                "elapsed_ms": 1234 if st.recording else 0,
                                "stop_reason": "none" if st.recording else "manual"})

        self.err(404, "not_found", "unknown route")


def serve(port=0, state=None):
    """Start the mock in a background thread. Returns (server, state)."""
    state = state or State()
    handler = type("BoundHandler", (Handler,), {"state": state})
    server = ThreadingHTTPServer(("127.0.0.1", port), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, state


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()
    srv, _ = serve(args.port)
    print(f"mock streamer on http://127.0.0.1:{srv.server_address[1]}")
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
