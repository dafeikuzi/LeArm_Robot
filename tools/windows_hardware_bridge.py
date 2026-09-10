#!/usr/bin/env python3
"""Expose a Windows-owned LeArm serial port and camera to WSL over TCP."""

from __future__ import annotations

import argparse
import logging
import os
import signal
import socket
import struct
import threading
import time

if os.name == "nt":
    os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")

import cv2
try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:  # Allows protocol-only tests outside the Windows venv.
    serial = None
    list_ports = None


LOGGER = logging.getLogger("learm_windows_bridge")
CAMERA_HEADER = struct.Struct("!4sBBHHQQI")
CAMERA_MAGIC = b"LIMG"
CAMERA_VERSION = 1
CAMERA_REQUEST = b"GET1"
MAX_JPEG_BYTES = 2 * 1024 * 1024


def find_serial_port(requested: str) -> str:
    if list_ports is None:
        raise RuntimeError("pyserial is required; start the bridge through the PowerShell script")
    if requested.lower() != "auto":
        return requested
    matches = [
        item.device for item in list_ports.comports()
        if item.vid == 0x1A86 and item.pid == 0x7523
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"expected exactly one CH340 (VID:PID 1a86:7523), found {matches or 'none'}; "
            "use --serial-port COMx"
        )
    return matches[0]


def open_serial(requested: str) -> serial.Serial:
    if serial is None:
        raise RuntimeError("pyserial is required; start the bridge through the PowerShell script")
    device = find_serial_port(requested)
    connection = serial.Serial(
        port=None,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
        write_timeout=1.0,
    )
    connection.dtr = True
    connection.rts = False
    connection.port = device
    connection.open()
    connection.dtr = True
    connection.rts = False
    time.sleep(0.25)
    connection.reset_input_buffer()
    connection.reset_output_buffer()
    LOGGER.info("serial ready: %s at 115200 baud (DTR=high, RTS=low)", device)
    return connection


def make_server(bind_host: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((bind_host, port))
    server.listen(2)
    server.settimeout(1.0)
    return server


class SerialBridge:
    def __init__(self, bind_host: str, port: int, serial_port: str, stop: threading.Event) -> None:
        self.bind_host = bind_host
        self.port = port
        self.serial_port = serial_port
        self.stop = stop
        self.connection: serial.Serial | None = None

    def _discard_connection(self) -> None:
        connection, self.connection = self.connection, None
        if connection is not None:
            try:
                connection.close()
            except (OSError, serial.SerialException):
                pass

    def _open_until_ready(self) -> serial.Serial | None:
        while not self.stop.is_set():
            try:
                self.connection = open_serial(self.serial_port)
                return self.connection
            except (OSError, serial.SerialException, RuntimeError) as exc:
                LOGGER.warning("serial unavailable: %s; retrying in 2 seconds", exc)
                self.stop.wait(2.0)
        return None

    def _serial_to_tcp(
        self, client: socket.socket, connection: serial.Serial, failed: threading.Event
    ) -> None:
        try:
            while not self.stop.is_set() and not failed.is_set():
                data = connection.read(256)
                if data:
                    client.sendall(data)
        except serial.SerialException as exc:
            LOGGER.warning("serial receive bridge stopped: %s", exc)
            self._discard_connection()
            failed.set()
        except OSError as exc:
            LOGGER.info("serial TCP receive stopped: %s", exc)
            failed.set()

    def _serve_client(self, client: socket.socket) -> None:
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        client.settimeout(1.0)
        connection = self.connection
        if connection is None:
            return
        failed = threading.Event()
        reader = threading.Thread(
            target=self._serial_to_tcp, args=(client, connection, failed), daemon=True)
        reader.start()
        try:
            while not self.stop.is_set() and not failed.is_set():
                try:
                    data = client.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                connection.write(data)
                connection.flush()
        except serial.SerialException as exc:
            LOGGER.warning("serial transmit bridge stopped: %s", exc)
            self._discard_connection()
            failed.set()
        except OSError as exc:
            LOGGER.info("serial TCP transmit stopped: %s", exc)
            failed.set()
        finally:
            failed.set()
            reader.join(timeout=1.0)

    def run(self) -> None:
        with make_server(self.bind_host, self.port) as server:
            LOGGER.info("serial TCP listening on %s:%d", self.bind_host, self.port)
            while not self.stop.is_set():
                if self.connection is None or not self.connection.is_open:
                    if self._open_until_ready() is None:
                        break
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                LOGGER.info("serial client connected: %s:%d", *address)
                try:
                    with client:
                        self._serve_client(client)
                finally:
                    LOGGER.info("serial client disconnected")
                    if self.connection is not None and not self.connection.is_open:
                        self.connection = None
            self._discard_connection()


class LatestCameraFrame:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.sequence = 0
        self.captured_ns = 0
        self.width = 0
        self.height = 0
        self.jpeg = b""

    def update(self, frame, quality: int) -> None:
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return
        jpeg = encoded.tobytes()
        if len(jpeg) > MAX_JPEG_BYTES:
            LOGGER.warning("dropping oversized JPEG frame: %d bytes", len(jpeg))
            return
        with self.lock:
            self.sequence += 1
            self.captured_ns = time.time_ns()
            self.height, self.width = frame.shape[:2]
            self.jpeg = jpeg

    def snapshot(self):
        with self.lock:
            return self.sequence, self.captured_ns, self.width, self.height, self.jpeg

    def clear(self) -> None:
        with self.lock:
            self.captured_ns = 0
            self.width = 0
            self.height = 0
            self.jpeg = b""


class CameraBridge:
    def __init__(self, bind_host: str, port: int, camera_index: int, camera_backend: str, fps: int,
                 jpeg_quality: int, stop: threading.Event) -> None:
        self.bind_host = bind_host
        self.port = port
        self.camera_index = camera_index
        self.camera_backend = camera_backend
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.stop = stop
        self.latest = LatestCameraFrame()

    def backend_candidates(self):
        backends = {
            "msmf": ("MSMF", cv2.CAP_MSMF),
            "dshow": ("DirectShow", cv2.CAP_DSHOW),
        }
        if self.camera_backend == "auto":
            return [backends["msmf"], backends["dshow"]]
        return [backends[self.camera_backend]]

    def capture_loop(self) -> None:
        candidates = self.backend_candidates()
        backend_index = 0
        while not self.stop.is_set():
            backend_name, backend = candidates[backend_index]
            capture = cv2.VideoCapture(self.camera_index, backend)
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            capture.set(cv2.CAP_PROP_FPS, self.fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                capture.release()
                self.latest.clear()
                LOGGER.warning(
                    "camera %d unavailable through %s; trying the next backend in 2 seconds",
                    self.camera_index, backend_name,
                )
                backend_index = (backend_index + 1) % len(candidates)
                self.stop.wait(2.0)
                continue
            LOGGER.info("camera opened: index=%d through %s", self.camera_index, backend_name)
            frame_received = False
            try:
                while not self.stop.is_set():
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        self.latest.clear()
                        LOGGER.warning(
                            "camera frame read failed through %s; trying the next backend in 2 seconds",
                            backend_name,
                        )
                        break
                    if not frame_received:
                        frame_received = True
                        LOGGER.info(
                            "camera ready: index=%d through %s, %dx%d at %d FPS",
                            self.camera_index, backend_name, frame.shape[1], frame.shape[0], self.fps,
                        )
                    self.latest.update(frame, self.jpeg_quality)
            finally:
                capture.release()
            backend_index = (backend_index + 1) % len(candidates)
            self.stop.wait(2.0)

    def _serve_client(self, client: socket.socket, address) -> None:
        client.settimeout(2.0)
        LOGGER.info("camera client connected: %s:%d", *address)
        try:
            while not self.stop.is_set():
                request = bytearray()
                while len(request) < len(CAMERA_REQUEST):
                    chunk = client.recv(len(CAMERA_REQUEST) - len(request))
                    if not chunk:
                        return
                    request.extend(chunk)
                if bytes(request) != CAMERA_REQUEST:
                    raise ValueError("invalid camera request")
                sequence, captured_ns, width, height, jpeg = self.latest.snapshot()
                status = 0 if jpeg else 1
                payload = jpeg if status == 0 else b""
                header = CAMERA_HEADER.pack(
                    CAMERA_MAGIC, CAMERA_VERSION, status, width, height,
                    sequence, captured_ns, len(payload),
                )
                client.sendall(header + payload)
        except (OSError, ValueError) as exc:
            LOGGER.info("camera client disconnected: %s", exc)

    def run(self) -> None:
        threading.Thread(target=self.capture_loop, daemon=True).start()
        with make_server(self.bind_host, self.port) as server:
            LOGGER.info("camera TCP listening on %s:%d", self.bind_host, self.port)
            while not self.stop.is_set():
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                with client:
                    self._serve_client(client, address)


def list_devices() -> None:
    if list_ports is None:
        raise RuntimeError("pyserial is required; start the bridge through the PowerShell script")
    print("Serial ports:")
    for item in list_ports.comports():
        vid_pid = f"{item.vid:04x}:{item.pid:04x}" if item.vid is not None else "unknown"
        print(f"  {item.device}: {vid_pid} {item.description}")
    print("Camera indices:")
    for backend_name, backend in (("MSMF", cv2.CAP_MSMF), ("DirectShow", cv2.CAP_DSHOW)):
        for index in range(5):
            capture = cv2.VideoCapture(index, backend)
            if capture.isOpened():
                ok, frame = capture.read()
                shape = frame.shape if ok and frame is not None else "no frame"
                print(f"  {backend_name} index {index}: {shape}")
            capture.release()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial-port", default="auto")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--camera-backend", choices=("auto", "msmf", "dshow"), default="auto")
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--serial-tcp-port", type=int, default=8766)
    parser.add_argument("--camera-tcp-port", type=int, default=8767)
    parser.add_argument("--camera-fps", type=int, default=15)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--list-devices", action="store_true")
    args = parser.parse_args()
    for name in ("serial_tcp_port", "camera_tcp_port"):
        if not 1 <= getattr(args, name) <= 65535:
            parser.error(f"--{name.replace('_', '-')} must be between 1 and 65535")
    if args.camera_fps <= 0:
        parser.error("--camera-fps must be greater than 0")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be between 1 and 100")
    return args


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    if args.list_devices:
        list_devices()
        return 0
    if serial is None:
        raise RuntimeError("pyserial is required; start the bridge through the PowerShell script")
    stop = threading.Event()

    def request_stop(*_args) -> None:
        if not stop.is_set():
            LOGGER.info("shutdown requested; closing bridge services")
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    workers = [
        threading.Thread(
            target=SerialBridge(args.bind_host, args.serial_tcp_port, args.serial_port, stop).run,
            name="serial-bridge",
            daemon=True,
        ),
        threading.Thread(
            target=CameraBridge(
                args.bind_host, args.camera_tcp_port, args.camera_index, args.camera_backend,
                args.camera_fps, args.jpeg_quality, stop,
            ).run,
            name="camera-bridge",
            daemon=True,
        ),
    ]
    for worker in workers:
        worker.start()
    while not stop.wait(0.2) and any(worker.is_alive() for worker in workers):
        pass
    stop.set()
    for worker in workers:
        worker.join(timeout=3.0)
    still_running = [worker.name for worker in workers if worker.is_alive()]
    if still_running:
        LOGGER.warning("forcing process exit with blocked worker(s): %s", ", ".join(still_running))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
