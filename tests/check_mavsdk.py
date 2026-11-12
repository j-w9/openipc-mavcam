#!/usr/bin/env python3
"""Drive mavcam with MAVSDK's camera plugin: an independent implementation of
the camera protocol, including the definition-file download and parser.

    python3 tests/check_mavsdk.py /path/to/ardupilot [--http-definition]

Layout: MAVSDK <-UDP-> mavcam <-UDP-> ArduCopter SITL, streamer mocked.
Needs the gRPC MAVSDK binding:  pip install 'mavsdk<4'
Prints one line per step and exits non-zero if any step failed.
"""
import asyncio
import os
import socket
import subprocess
import sys
import tempfile
import warnings

warnings.simplefilter("ignore")
if os.environ.get("MAVSDK_DEBUG"):
    import logging
    logging.basicConfig(level=logging.INFO, stream=sys.stdout)
from mavsdk import System  # noqa: E402
from mavsdk import camera as cam  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import mock_streamer  # noqa: E402

results = []


def free_port(kind):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


async def first(agen, timeout, pred=lambda x: True):
    async def run():
        async for item in agen:
            if pred(item):
                return item
    return await asyncio.wait_for(run(), timeout)


async def step(name, coro):
    try:
        detail = await coro
        results.append((name, True, detail))
        print(f"ok    {name}: {detail}")
    except Exception as e:  # noqa: BLE001 - report every kind of failure
        results.append((name, False, repr(e)))
        print(f"FAIL  {name}: {e!r}")


async def main(ardupilot, via_http):
    httpd = None
    tmp = tempfile.TemporaryDirectory()
    server, state = mock_streamer.serve(0)
    parm = os.path.join(tmp.name, "p.parm")
    with open(parm, "w") as f:
        f.write("SERIAL2_PROTOCOL 2\nCAM1_TYPE 6\n")
    cam_port = free_port(socket.SOCK_DGRAM)
    gcs_port = free_port(socket.SOCK_DGRAM)
    defaults = os.path.join(ardupilot, "Tools/autotest/default_params/copter.parm")
    sitl = subprocess.Popen([
        os.path.join(ardupilot, "build/sitl/bin/arducopter"), "--model", "quad", "--speedup", "1", "-w",
        "--defaults", f"{defaults},{parm}", "--serial0", f"tcp:{free_port(socket.SOCK_STREAM)}",
        "--serial2", f"udpclient:127.0.0.1:{cam_port}",
    ], cwd=tmp.name, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    cam_args = [
        os.path.join(ROOT, "mavcam"), "--link", f"udpin:{cam_port}",
        "--ground-out", f"127.0.0.1:{gcs_port}",
        "--http", f"127.0.0.1:{server.server_address[1]}", "--media-dir", tmp.name,
        "--model", "SDKCam", "--ftp-rate", "100000",
    ]
    if via_http:
        # MAVSDK 3.x asks the *autopilot* for mftp:// definition files, so it cannot fetch one
        # from a separate camera component. Serve a copy over HTTP to exercise the rest.
        web = os.path.join(tmp.name, "web")
        os.mkdir(web)
        with open(os.path.join(web, "camdef.xml"), "wb") as f:
            f.write(subprocess.run(cam_args + ["--dump-camdef"], check=True, capture_output=True).stdout)
        http_port = free_port(socket.SOCK_STREAM)
        httpd = subprocess.Popen([sys.executable, "-m", "http.server", str(http_port), "--bind", "127.0.0.1",
                                  "--directory", web], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        cam_args += ["--camdef-uri", f"http://127.0.0.1:{http_port}/camdef.xml"]
    mavcam = subprocess.Popen(cam_args)
    try:
        drone = System()
        await drone.connect(system_address=f"udpin://0.0.0.0:{gcs_port}")
        await step("vehicle discovered through mavcam",
                   first(drone.core.connection_state(), 40, lambda s: s.is_connected))

        async def camera_found():
            lst = await first(drone.camera.camera_list(), 40, lambda c: len(c.cameras) > 0)
            c = lst.cameras[0]
            assert c.vendor_name == "OpenIPC" and c.model_name == "SDKCam", (c.vendor_name, c.model_name)
            return f"component {c.component_id}, {c.vendor_name} {c.model_name}, {c.horizontal_resolution_px}x{c.vertical_resolution_px}"
        await step("camera discovered", camera_found())
        comp = 100

        async def stream():
            v = await first(drone.camera.video_stream_info(), 15)
            s = v.video_stream_info.settings
            assert s.uri == "5600", s.uri
            return f"uri {s.uri}, {s.horizontal_resolution_pix}x{s.vertical_resolution_pix} @ {s.frame_rate_hz} Hz, {s.bit_rate_b_s} b/s"
        await step("video stream information", stream())

        async def storage():
            s = await first(drone.camera.storage(), 15)
            assert s.storage.total_storage_mib > 0
            return f"{s.storage.storage_status}, {s.storage.available_storage_mib:.0f} MiB free"
        await step("storage", storage())

        async def options():
            # MAVSDK only produces these after fetching and parsing the definition file over MAVLink FTP
            o = await first(drone.camera.possible_setting_options(), 40, lambda u: len(u.setting_options) > 0)
            ids = sorted(s.setting_id for s in o.setting_options)
            assert "BITRATE" in ids and "RES" in ids, ids
            res = [s for s in o.setting_options if s.setting_id == "RES"][0]
            return f"{len(ids)} settings {ids}; RES options {[x.option_description for x in res.options]}"
        await step("definition file fetched and parsed (settings offered)", options())

        async def current():
            c = await first(drone.camera.current_settings(), 20, lambda u: len(u.current_settings) > 0)
            vals = {s.setting_id: s.option.option_id for s in c.current_settings}
            return f"{vals}"
        await step("current settings", current())

        async def set_bitrate():
            await asyncio.wait_for(drone.camera.set_setting(
                comp, cam.Setting("BITRATE", "", cam.Option("5000", ""), True)), 15)
            got = state.snapshot()["fields"]["video0.bitrate"]
            assert got == 5000, got
            return "streamer bitrate is now 5000"
        await step("set a range setting (BITRATE)", set_bitrate())

        async def set_res():
            await asyncio.wait_for(drone.camera.set_setting(
                comp, cam.Setting("RES", "", cam.Option("2", ""), False)), 15)
            got = state.snapshot()["fields"]["video0.size"]
            assert got == "1920x1080", got
            return "streamer size is now 1920x1080"
        await step("set an option setting (RES)", set_res())

        async def video():
            await asyncio.wait_for(drone.camera.start_video(comp), 15)
            assert state.snapshot()["recording"]
            await asyncio.wait_for(drone.camera.stop_video(comp), 15)
            assert not state.snapshot()["recording"]
            return "streamer started and stopped recording"
        await step("start / stop video", video())

        async def photo():
            task = asyncio.ensure_future(first(drone.camera.capture_info(), 20))
            await asyncio.sleep(0.5)
            await asyncio.wait_for(drone.camera.take_photo(comp), 15)
            info = await task
            assert info.is_success, info
            return f"photo {info.index} at {info.file_url}"
        await step("take photo", photo())

        async def zoom():
            await asyncio.wait_for(drone.camera.zoom_range(comp, 100.0), 15)
            got = state.snapshot()["fields"]["video0.framing"]
            assert got == "zoom-4x", got
            return "streamer framing is zoom-4x"
        await step("zoom range", zoom())
    finally:
        for p in (mavcam, sitl) + ((httpd,) if httpd else ()):
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        server.shutdown()
        tmp.cleanup()

    failed = [n for n, ok, _ in results if not ok]
    print(f"\n{len(results) - len(failed)} of {len(results)} steps passed")
    return 1 if failed else 0


if __name__ == "__main__":
    via_http = "--http-definition" in sys.argv
    if via_http:
        sys.argv.remove("--http-definition")
    if len(sys.argv) != 2 or not os.path.isfile(os.path.join(sys.argv[1], "build/sitl/bin/arducopter")):
        sys.exit(__doc__)
    code = asyncio.run(main(os.path.abspath(sys.argv[1]), via_http))
    os._exit(code)   # mavsdk_server threads can keep the interpreter alive
