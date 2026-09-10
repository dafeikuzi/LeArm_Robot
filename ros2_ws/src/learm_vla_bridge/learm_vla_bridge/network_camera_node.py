import socket
import struct
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


CAMERA_HEADER = struct.Struct('!4sBBHHQQI')
CAMERA_MAGIC = b'LIMG'
CAMERA_VERSION = 1
CAMERA_REQUEST = b'GET1'


class NetworkCameraClient:
    def __init__(self, host, port, timeout_s, max_payload_bytes):
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.max_payload_bytes = max_payload_bytes
        self.socket = None

    def connect(self):
        self.close()
        connection = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        connection.settimeout(self.timeout_s)
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.socket = connection

    def close(self):
        if self.socket is not None:
            try:
                self.socket.close()
            finally:
                self.socket = None

    def _read_exact(self, size):
        data = bytearray()
        while len(data) < size:
            chunk = self.socket.recv(size - len(data))
            if not chunk:
                raise ConnectionError('camera TCP connection closed')
            data.extend(chunk)
        return bytes(data)

    def read(self):
        if self.socket is None:
            self.connect()
        try:
            self.socket.sendall(CAMERA_REQUEST)
            header = self._read_exact(CAMERA_HEADER.size)
            magic, version, status, width, height, sequence, captured_ns, length = \
                CAMERA_HEADER.unpack(header)
            if magic != CAMERA_MAGIC or version != CAMERA_VERSION:
                raise ValueError('invalid network camera protocol header')
            if length > self.max_payload_bytes:
                raise ValueError(f'network camera payload is too large: {length}')
            if status != 0 or length == 0:
                raise RuntimeError(f'Windows camera has no frame (status={status})')
            jpeg = self._read_exact(length)
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError('cannot decode network camera JPEG')
            if frame.shape[1] != width or frame.shape[0] != height:
                raise ValueError('network camera dimensions do not match JPEG')
            return sequence, captured_ns, frame
        except Exception:
            self.close()
            raise


class NetworkCameraNode(Node):
    def __init__(self):
        super().__init__('learm_network_camera')
        host = self.declare_parameter('host', '127.0.0.1').value
        port = int(self.declare_parameter('port', 8767).value)
        timeout_ms = int(self.declare_parameter('request_timeout_ms', 1000).value)
        max_payload = int(self.declare_parameter('max_payload_bytes', 2 * 1024 * 1024).value)
        publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 15.0).value)
        self.reconnect_interval_s = float(
            self.declare_parameter('reconnect_interval_s', 1.0).value)
        self.frame_id = self.declare_parameter('frame_id', 'learm_camera').value
        self.bridge = CvBridge()
        self.client = NetworkCameraClient(host, port, max(timeout_ms, 1) / 1000.0, max_payload)
        self.publisher = self.create_publisher(Image, '~/image_raw', 10)
        self.last_sequence = None
        self.last_attempt = 0.0
        self.last_warning = 0.0
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 0.1), self._request_frame)
        self.get_logger().info(f'Network camera configured for {host}:{port}')

    def _request_frame(self):
        now = time.monotonic()
        if self.client.socket is None and now - self.last_attempt < self.reconnect_interval_s:
            return
        self.last_attempt = now
        try:
            sequence, _captured_ns, frame = self.client.read()
        except Exception as exc:
            if now - self.last_warning >= self.reconnect_interval_s:
                self.last_warning = now
                self.get_logger().warn(f'Network camera unavailable: {exc}')
            return
        if sequence == self.last_sequence:
            return
        self.last_sequence = sequence
        message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)

    def destroy_node(self):
        self.client.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NetworkCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
