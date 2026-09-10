#!/usr/bin/env python3
"""Persistent ROS 2 service client used by the browser control panel."""

import json
import queue
import sys
import threading

import rclpy
from learm_driver.srv import GetArmStatus, MoveJoints, MovePose, SetGripper
from rclpy.node import Node
from std_srvs.srv import Trigger


class ServiceWorker(Node):
    def __init__(self):
        super().__init__('learm_web_service_worker')
        self.service_clients = {
            '/learm_driver/move_joints': self.create_client(MoveJoints, '/learm_driver/move_joints'),
            '/learm_driver/move_pose': self.create_client(MovePose, '/learm_driver/move_pose'),
            '/learm_driver/set_gripper': self.create_client(SetGripper, '/learm_driver/set_gripper'),
            '/learm_driver/get_status': self.create_client(GetArmStatus, '/learm_driver/get_status'),
            '/learm_driver/emergency_stop': self.create_client(Trigger, '/learm_driver/emergency_stop'),
            '/learm_driver/clear_estop': self.create_client(Trigger, '/learm_driver/clear_estop'),
        }
        self.service_types = {
            '/learm_driver/move_joints': MoveJoints,
            '/learm_driver/move_pose': MovePose,
            '/learm_driver/set_gripper': SetGripper,
            '/learm_driver/get_status': GetArmStatus,
            '/learm_driver/emergency_stop': Trigger,
            '/learm_driver/clear_estop': Trigger,
        }

    def call(self, service_name, values):
        client = self.service_clients.get(service_name)
        if client is None:
            return {'ok': False, 'message': f'不支持的 ROS 服务: {service_name}'}
        if not client.wait_for_service(timeout_sec=35.0):
            return {'ok': False, 'message': f'ROS 服务不可用: {service_name}'}

        request = self.service_types[service_name].Request()
        if service_name == '/learm_driver/move_joints':
            request.joint_names = list(values.get('joint_names', []))
            request.positions_rad = [float(value) for value in values.get('positions_rad', [])]
            request.duration_ms = int(values.get('duration_ms', 500))
        elif service_name == '/learm_driver/move_pose':
            request.opening = float(values.get('opening', 0.0))
            request.positions_rad = [float(value) for value in values.get('positions_rad', [])]
            request.duration_ms = int(values.get('duration_ms', 500))
        elif service_name == '/learm_driver/set_gripper':
            request.opening = float(values.get('opening', 0.0))
            request.duration_ms = int(values.get('duration_ms', 500))

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=35.0)
        if not future.done():
            future.cancel()
            return {'ok': False, 'message': f'ROS 服务调用超时: {service_name}'}
        try:
            response = future.result()
        except Exception as exc:
            return {'ok': False, 'message': f'ROS 服务调用失败: {exc}'}

        ok = bool(getattr(response, 'success', True))
        message = str(getattr(response, 'message', response))
        if service_name == '/learm_driver/get_status' and ok:
            current = list(getattr(response, 'current_pulse_us', []))
            target = list(getattr(response, 'target_pulse_us', []))
            message = (
                f'{message}; moving={bool(response.moving)}, estop={bool(response.estop_active)}, '
                f'current={current}, target={target}'
            )
        return {'ok': ok, 'message': message}


def main():
    rclpy.init()
    node = ServiceWorker()
    requests = queue.Queue()

    def read_requests():
        try:
            for line in sys.stdin:
                try:
                    requests.put(json.loads(line))
                except json.JSONDecodeError as exc:
                    print(json.dumps({'id': None, 'ok': False, 'message': f'请求 JSON 无效: {exc}'}, ensure_ascii=False), flush=True)
        finally:
            requests.put(None)

    threading.Thread(target=read_requests, daemon=True).start()
    try:
        while rclpy.ok():
            try:
                item = requests.get(timeout=0.05)
            except queue.Empty:
                continue
            if item is None:
                break
            try:
                result = node.call(item.get('service', ''), item.get('request', {}))
            except Exception as exc:
                result = {'ok': False, 'message': f'ROS 客户端异常: {exc}'}
            result['id'] = item.get('id')
            print(json.dumps(result, ensure_ascii=False), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
