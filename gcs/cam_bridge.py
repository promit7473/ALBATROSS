#!/usr/bin/env python3
"""Gazebo camera -> MJPEG bridge (Wayland-proof; no screen scraping, no ROS).

The old feed screen-scraped the desktop with ffmpeg x11grab, which returns an
all-black frame under a Wayland session (XWayland is rootless -- the X root the
grab reads is never composited with what's on screen). This instead subscribes
to the drone's onboard camera image topic straight off gz-transport and serves
it as multipart MJPEG on :8091. It renders on the GPU inside the sim, so it
works identically under Wayland, Xorg, or fully headless.

server.py proxies /video.mjpeg here, so the web UI is unchanged.
"""
import os
# gz.msgs10 was generated for an older protobuf; force the pure-python runtime
# so it imports against the system protobuf 7.x (image data is one memcpy, so
# the pure-python parser is plenty fast for a video feed).
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
import gz.transport13 as gzt

PORT = int(os.environ.get("CAM_BRIDGE_PORT", "8091"))
TOPIC = os.environ.get("CAM_TOPIC", "")      # empty => auto-discover
OUT_W = int(os.environ.get("CAM_OUT_W", "640"))   # downscale for the web feed
JPEG_Q = int(os.environ.get("CAM_JPEG_Q", "70"))
FPS = float(os.environ.get("CAM_FPS", "15"))

_lock = threading.Lock()
_latest = {"jpeg": None, "ts": 0.0}


def _placeholder(text):
    img = np.full((360, 640, 3), 30, np.uint8)
    cv2.putText(img, text, (28, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (180, 180, 180), 2, cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
    return buf.tobytes() if ok else None


_WAITING = _placeholder("Waiting for Gazebo camera...")


def on_image(msg):
    try:
        h, w = msg.height, msg.width
        if h == 0 or w == 0:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        ch = arr.size // (h * w)          # infer 1/3/4 channels from payload size
        if ch < 1:
            return
        arr = arr[: h * w * ch].reshape(h, w, ch)
        if ch == 3:
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif ch == 4:
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        else:
            bgr = cv2.cvtColor(arr[:, :, 0], cv2.COLOR_GRAY2BGR)
        if OUT_W and w > OUT_W:
            bgr = cv2.resize(bgr, (OUT_W, int(h * OUT_W / w)))
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
        if ok:
            with _lock:
                _latest["jpeg"] = buf.tobytes()
                _latest["ts"] = time.time()
    except Exception as e:
        print(f"[cam_bridge] decode error: {e}", flush=True)


def discover_topic():
    """Poll `gz topic -l` until the drone's camera image topic appears."""
    while True:
        try:
            out = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                                 text=True, timeout=5).stdout
        except Exception:
            out = ""
        cands = [t for t in out.split()
                 if t.endswith("/image") and "depth" not in t.lower()]
        if cands:
            return sorted(cands, key=len)[0]   # shortest = the main camera
        time.sleep(1.0)


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/healthz":
            with _lock:
                fresh = _latest["jpeg"] is not None and (time.time() - _latest["ts"] < 2.0)
            body = b'{"ok": true, "fresh": %s}' % (b"true" if fresh else b"false")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if not self.path.startswith("/video.mjpeg"):
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace;boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        period = 1.0 / FPS
        try:
            while True:
                with _lock:
                    j = _latest["jpeg"]
                if not j:
                    j = _WAITING
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(j)}\r\n\r\n".encode())
                self.wfile.write(j)
                self.wfile.write(b"\r\n")
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    node = gzt.Node()
    topic = TOPIC or discover_topic()
    print(f"[cam_bridge] subscribing to {topic}", flush=True)
    if not node.subscribe(Image, topic, on_image):
        print(f"[cam_bridge] FAILED to subscribe to {topic}", flush=True)
    print(f"[cam_bridge] MJPEG on http://0.0.0.0:{PORT}/video.mjpeg", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()


if __name__ == "__main__":
    main()
