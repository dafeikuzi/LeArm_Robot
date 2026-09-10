#!/usr/bin/env python3
"""Browser control panel for the LeArm VLA ROS collection workflow."""

import html
import json
import math
import os
import shlex
import signal
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


WORKSPACE = os.environ.get('LEARM_WORKSPACE', '/root/LeArm_Robot/ros2_ws')
DEFAULT_DATASET = '/root/LeArm_Robot/datasets/learm_vla/raw'
HOST = os.environ.get('LEARM_WEB_HOST', '0.0.0.0')
PORT = int(os.environ.get('LEARM_WEB_PORT', '8765'))


class CollectionController:
    def __init__(self):
        self.lock = threading.RLock()
        self.processes = {}
        self.logs = []
        self.recording = False
        self.worker_lock = threading.RLock()
        self.worker_pending = {}
        self.worker_counter = 0
        self.ros_worker = None

    def log(self, message):
        with self.lock:
            self.logs.append(message.rstrip())
            self.logs = self.logs[-300:]

    def _command(self, args):
        setup = os.path.join(WORKSPACE, 'install', 'setup.bash')
        return 'source /opt/ros/jazzy/setup.bash && source {} && exec {}'.format(
            shlex.quote(setup), shlex.join(args))

    def start(self, name, args):
        with self.lock:
            current = self.processes.get(name)
            if current and current.poll() is None:
                return False, f'{name} 已在运行'
            command = self._command(args)
            self.log(f'$ {command}')
            process = subprocess.Popen(
                ['/bin/bash', '-lc', command],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            self.processes[name] = process
            threading.Thread(target=self._read_output, args=(name, process), daemon=True).start()
            return True, f'{name} 已启动 (PID {process.pid})'

    def _read_output(self, name, process):
        try:
            for line in process.stdout:
                self.log(f'[{name}] {line}')
        finally:
            code = process.wait()
            self.log(f'[{name}] 已退出，返回码 {code}')
            with self.lock:
                if self.processes.get(name) is process:
                    self.processes.pop(name, None)
                if name == 'VLA 记录节点':
                    self.recording = False

    def stop(self, name):
        with self.lock:
            process = self.processes.get(name)
        if not process or process.poll() is not None:
            if name == '机械臂驱动':
                pids = self._external_driver_pids()
                if pids:
                    try:
                        os.killpg(os.getpgid(pids[0]), signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    return True, '已请求停止外部机械臂驱动'
            return False, f'{name} 未运行'
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        return True, f'已请求停止 {name}'

    def driver_running(self):
        with self.lock:
            process = self.processes.get('机械臂驱动')
            if process is not None and process.poll() is None:
                return True
        return bool(self._external_driver_pids())

    @staticmethod
    def _external_driver_pids():
        pids = []
        for entry in Path('/proc').glob('[0-9]*'):
            try:
                command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            except (FileNotFoundError, PermissionError):
                continue
            if 'learm_driver_node' in command:
                try:
                    pids.append(int(entry.name))
                except ValueError:
                    pass
        return pids

    def _ensure_ros_worker(self):
        with self.worker_lock:
            if self.ros_worker is not None and self.ros_worker.poll() is None:
                return self.ros_worker
            worker_path = os.path.join(os.path.dirname(__file__), 'vla_ros_worker.py')
            command = self._command(['python3', worker_path])
            self.log(f'$ {command}')
            process = subprocess.Popen(
                ['/bin/bash', '-lc', command],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            self.ros_worker = process
            threading.Thread(target=self._read_ros_worker_output, args=(process,), daemon=True).start()
            threading.Thread(target=self._read_ros_worker_error, args=(process,), daemon=True).start()
            return process

    def _read_ros_worker_output(self, process):
        try:
            for line in process.stdout:
                try:
                    response = json.loads(line)
                except json.JSONDecodeError:
                    self.log(f'[ROS 控制客户端] {line}')
                    continue
                request_id = response.get('id')
                with self.worker_lock:
                    pending = self.worker_pending.pop(request_id, None)
                if pending is not None:
                    pending['response'] = response
                    pending['event'].set()
        finally:
            with self.worker_lock:
                if self.ros_worker is process:
                    self.ros_worker = None
                pending_requests = list(self.worker_pending.values())
                self.worker_pending.clear()
            for pending in pending_requests:
                pending['response'] = {'ok': False, 'message': 'ROS 控制客户端已退出'}
                pending['event'].set()

    def _read_ros_worker_error(self, process):
        for line in process.stderr:
            self.log(f'[ROS 控制客户端] {line}')

    def _call_ros_worker(self, service_name, request, timeout=40.0):
        process = self._ensure_ros_worker()
        with self.worker_lock:
            self.worker_counter += 1
            request_id = str(self.worker_counter)
            pending = {'event': threading.Event(), 'response': None}
            self.worker_pending[request_id] = pending
            try:
                process.stdin.write(json.dumps({
                    'id': request_id,
                    'service': service_name,
                    'request': request,
                }, ensure_ascii=False) + '\n')
                process.stdin.flush()
            except (BrokenPipeError, OSError) as exc:
                self.worker_pending.pop(request_id, None)
                return False, f'ROS 控制客户端写入失败: {exc}'
        if not pending['event'].wait(timeout):
            with self.worker_lock:
                self.worker_pending.pop(request_id, None)
            return False, 'ROS 控制服务调用超时'
        response = pending['response'] or {'ok': False, 'message': 'ROS 控制客户端无响应'}
        return bool(response.get('ok')), str(response.get('message', ''))

    def driver_service(self, service_name, service_type, request):
        del service_type
        success, message = self._call_ros_worker(service_name, request)
        self.log(f'[机械臂控制] {message}')
        return success, message

    def service(self, action):
        command = self._command([
            'ros2', 'service', 'call', f'/learm_vla_recorder/{action}',
            'std_srvs/srv/Trigger', '{}',
        ])
        self.log(f'$ {command}')
        try:
            result = subprocess.run(
                ['/bin/bash', '-lc', command], capture_output=True, text=True, timeout=15)
        except subprocess.TimeoutExpired:
            return False, '记录服务调用超时'
        output = (result.stdout + result.stderr).strip()
        for line in output.splitlines():
            self.log(f'[记录服务] {line}')
        success = result.returncode == 0 and ('success: true' in output.lower() or 'success=true' in output.lower())
        if success:
            with self.lock:
                self.recording = action == 'start_episode'
        return success, output[-500:] or f'返回码 {result.returncode}'

    def status(self):
        with self.lock:
            processes = {name: process.poll() is None for name, process in self.processes.items()}
            if '机械臂驱动' not in processes and self._external_driver_pids():
                processes['机械臂驱动'] = True
            return {'processes': processes, 'recording': self.recording, 'logs': self.logs[-80:]}


controller = CollectionController()


def default_windows_host():
    try:
        result = subprocess.run(
            ['ip', 'route', 'show', 'default'], capture_output=True, text=True,
            timeout=2, check=False,
        )
        fields = result.stdout.split()
        if 'via' in fields:
            return fields[fields.index('via') + 1]
    except (OSError, subprocess.SubprocessError, IndexError):
        pass
    return '127.0.0.1'


def page():
    return """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LeArm VLA 数据采集</title>
<style>
body{font:15px system-ui,sans-serif;background:#f4f6f8;color:#18222d;margin:0}.wrap{max-width:980px;margin:28px auto;padding:0 18px}
h1{font-size:25px;margin:0 0 18px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}.panel{background:#fff;border:1px solid #d8dee5;border-radius:8px;padding:18px;box-shadow:0 2px 8px #0000000b}
label{display:block;font-weight:600;margin:10px 0 5px}input{box-sizing:border-box;width:100%;padding:9px;border:1px solid #b8c2cc;border-radius:5px;font:inherit}.buttons{display:flex;flex-wrap:wrap;gap:8px;margin-top:15px}button{border:0;border-radius:5px;padding:9px 12px;background:#1769aa;color:#fff;font-weight:600;cursor:pointer}button.secondary{background:#52606d}button.danger{background:#b42318}button:disabled{opacity:.5;cursor:default}.status{white-space:pre-wrap;background:#eef2f5;border-radius:5px;padding:10px;min-height:38px}.logs{height:350px;overflow:auto;background:#10161d;color:#d9e2ec;padding:12px;border-radius:5px;font:12px ui-monospace,monospace;white-space:pre-wrap}.hint{color:#52606d;font-size:13px}.control-panel{grid-column:1/-1}.arm-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px 20px}.arm-control{display:grid;grid-template-columns:78px minmax(0,1fr) 64px;gap:8px;align-items:center}.arm-control input[type=range]{width:100%;padding:0;border:0}.arm-control output{text-align:right;color:#52606d}.arm-options{display:grid;grid-template-columns:1fr 180px;gap:16px;align-items:end}@media(max-width:760px){.grid{grid-template-columns:1fr}.arm-grid{grid-template-columns:1fr}.arm-options{grid-template-columns:1fr}.control-panel{grid-column:auto}}
</style></head><body><main class="wrap"><h1>LeArm VLA 数据采集</h1><div class="grid">
<section class="panel"><h2>设备与数据</h2><label>ROS 2 工作区</label><input id="workspace" value="/root/LeArm_Robot/ros2_ws"><label>Windows 主机</label><input id="windowsHost" value="__DEFAULT_WINDOWS_HOST__"><label>串口 TCP 端口</label><input id="serialTcpPort" type="number" min="1" max="65535" value="8766"><label>摄像头 TCP 端口</label><input id="cameraTcpPort" type="number" min="1" max="65535" value="8767"><label>任务描述</label><input id="task" value="抓取棕色小狗"><label>数据目录</label><input id="dataset" value="/root/LeArm_Robot/datasets/learm_vla/raw"><div class="buttons"><button onclick="startDriver()">启动机械臂驱动</button><button onclick="startVla()">启动观察/记录节点</button><button class="secondary" onclick="stop('VLA 记录节点')">停止观察节点</button><button class="secondary" onclick="stop('机械臂驱动')">停止机械臂驱动</button></div></section>
<section class="panel"><h2>Episode 控制</h2><p class="hint">先启动机械臂驱动和观察/记录节点，确认状态正常后再开始记录。</p><div class="buttons"><button id="startEp" onclick="episode('start_episode')">开始记录</button><button id="stopEp" class="secondary" onclick="episode('stop_episode')">成功结束</button><button id="abortEp" class="danger" onclick="episode('abort_episode')">放弃记录</button></div><h3>状态</h3><div id="status" class="status">正在连接...</div></section>
<section class="panel control-panel"><h2>机械臂实时控制</h2><p class="hint">启动机械臂驱动后滑动关节和夹爪，目标会按 50 ms 节流发送；松开滑块会立即补发最终目标。</p><div class="arm-grid">
<label class="arm-control"><span>joint_2</span><input id="joint2" type="range" min="-90" max="90" step="0.5" value="0"><output id="joint2Out">0.0 deg</output></label>
<label class="arm-control"><span>joint_3</span><input id="joint3" type="range" min="-90" max="90" step="0.5" value="-40"><output id="joint3Out">-40.0 deg</output></label>
<label class="arm-control"><span>joint_4</span><input id="joint4" type="range" min="-90" max="90" step="0.5" value="-40"><output id="joint4Out">-40.0 deg</output></label>
<label class="arm-control"><span>joint_5</span><input id="joint5" type="range" min="-90" max="90" step="0.5" value="0"><output id="joint5Out">0.0 deg</output></label>
<label class="arm-control"><span>joint_6</span><input id="joint6" type="range" min="-90" max="90" step="0.5" value="0"><output id="joint6Out">0.0 deg</output></label>
<label class="arm-control"><span>夹爪</span><input id="gripper" type="range" min="0" max="100" step="1" value="0"><output id="gripperOut">0 %</output></label>
</div><div class="arm-options"><div><label>动作时间 (ms)</label><input id="duration" type="number" min="20" max="30000" step="10" value="500"></div><div class="buttons"><button onclick="movePose()">发送当前姿态</button><button class="secondary" onclick="resetPose()">复位到零位</button><button class="secondary" onclick="setGripper()">仅夹爪</button><button class="secondary" onclick="getArmStatus()">读取状态</button><button class="danger" onclick="driverCommand('emergency_stop')">软件急停</button><button class="secondary" onclick="driverCommand('clear_estop')">解除急停</button></div></div><div id="armStatus" class="status" style="margin-top:12px">尚未读取机械臂状态。</div></section>
</div><section class="panel" style="margin-top:16px"><h2>实时日志</h2><div id="logs" class="logs"></div></section></main>
<script>
const $=id=>document.getElementById(id); const val=id=>$(id).value.trim();
async function post(action,extra={}){const fields=Object.fromEntries(['workspace','windowsHost','serialTcpPort','cameraTcpPort','task','dataset'].map(x=>[x,val(x)]));const r=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...fields,...extra})});if(!r.ok)throw new Error('HTTP '+r.status);return r.json()}
async function request(action,extra={},showAlert=true){try{const r=await post(action,extra);const message=r.message||'操作完成';if(['move_joints','move_pose','set_gripper','get_arm_status','emergency_stop','clear_estop'].includes(action))$('armStatus').textContent=message;if(!r.ok&&showAlert)alert(message);return r}catch(error){const message='请求失败：'+error.message;$('armStatus').textContent=message;if(showAlert)alert(message);return {ok:false,message}}}
async function startDriver(){const r=await request('start_driver');if(r.ok)alert(r.message)} async function startVla(){const r=await request('start_vla');if(r.ok)alert(r.message)} async function stop(name){const r=await request(name==='机械臂驱动'?'stop_driver':'stop_vla');if(r.ok)alert(r.message)}
async function episode(action){await request(action)}
function controlPayload(){return{positions_deg:['joint2','joint3','joint4','joint5','joint6'].map(id=>Number($(id).value)),opening:Number($('gripper').value)/100,duration_ms:Number($('duration').value)}}
async function movePose(){await request('move_pose',controlPayload())} async function setGripper(){await request('set_gripper',{opening:Number($('gripper').value)/100,duration_ms:Number($('duration').value)})} async function getArmStatus(){await request('get_arm_status')} async function driverCommand(action){await request(action)}
const LIVE_SEND_INTERVAL_MS=50;const jointMap={joint2:'joint_2',joint3:'joint_3',joint4:'joint_4',joint5:'joint_5',joint6:'joint_6'};let pendingJoints=new Set(),pendingGripper=false,liveRequestInFlight=false,lastLiveSendTime=0,flushAfterResponse=false,liveTimer=null;
function durationValue(){const value=Number($('duration').value);return Number.isFinite(value)?Math.max(20,Math.min(30000,Math.round(value))):500}
function scheduleLive(){if(liveTimer===null)liveTimer=setTimeout(()=>{liveTimer=null;dispatchLiveCommand(false)},LIVE_SEND_INTERVAL_MS)}
function queueJoint(id){pendingJoints.add(id);scheduleLive()} function queueGripper(){pendingGripper=true;scheduleLive()}
function flushLive(){flushAfterResponse=true;if(liveTimer!==null){clearTimeout(liveTimer);liveTimer=null}dispatchLiveCommand(true)}
async function dispatchLiveCommand(force=false){if(liveRequestInFlight)return;if(!force&&Date.now()-lastLiveSendTime<LIVE_SEND_INTERVAL_MS){scheduleLive();return}if(pendingJoints.size){const ids=Array.from(pendingJoints);pendingJoints.clear();liveRequestInFlight=true;lastLiveSendTime=Date.now();await request('move_joints',{joint_names:ids.map(id=>jointMap[id]),positions_deg:ids.map(id=>Number($(id).value)),duration_ms:durationValue()},false);liveRequestInFlight=false;if(flushAfterResponse){flushAfterResponse=false;dispatchLiveCommand(true)}else dispatchLiveCommand(false);return}if(pendingGripper){pendingGripper=false;liveRequestInFlight=true;lastLiveSendTime=Date.now();await request('set_gripper',{opening:Number($('gripper').value)/100,duration_ms:durationValue()},false);liveRequestInFlight=false;if(flushAfterResponse){flushAfterResponse=false;dispatchLiveCommand(true)}else dispatchLiveCommand(false);return}}
function resetPose(){[['joint2',0],['joint3',-40],['joint4',-40],['joint5',0],['joint6',0],['gripper',0]].forEach(([id,value])=>{$(id).value=value;$(id).dispatchEvent(new Event('input'))});pendingJoints.clear();pendingGripper=false;movePose()}
[['joint2','joint2Out'],['joint3','joint3Out'],['joint4','joint4Out'],['joint5','joint5Out'],['joint6','joint6Out']].forEach(([id,out])=>{const input=$(id);input.addEventListener('input',()=>{$(out).value=Number(input.value).toFixed(1)+' deg';queueJoint(id)});input.addEventListener('change',flushLive);input.addEventListener('pointerup',flushLive)});$('gripper').addEventListener('input',()=>{$('gripperOut').value=$('gripper').value+' %';queueGripper()});$('gripper').addEventListener('change',flushLive);$('gripper').addEventListener('pointerup',flushLive);
async function refresh(){try{const r=await fetch('/api/status');const s=await r.json();let p=Object.entries(s.processes).map(([k,v])=>k+': '+(v?'运行中':'已停止')).join('\\n');$('status').textContent=(p||'没有启动的节点')+'\\n记录状态: '+(s.recording?'正在记录':'未记录');$('logs').textContent=s.logs.join('\\n');$('logs').scrollTop=$('logs').scrollHeight;$('startEp').disabled=!s.processes['VLA 记录节点']||s.recording;$('stopEp').disabled=!s.recording;$('abortEp').disabled=!s.recording}catch(error){$('status').textContent='无法连接采集服务：'+error.message}} setInterval(refresh,1000);refresh();
</script></body></html>""".replace('__DEFAULT_WINDOWS_HOST__', default_windows_host())


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        return

    def send_json(self, value, code=200):
        data = json.dumps(value, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if urlparse(self.path).path == '/api/status':
            self.send_json(controller.status())
            return
        data = page().encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        if urlparse(self.path).path != '/api/action':
            self.send_json({'ok': False, 'message': 'unknown endpoint'}, 404)
            return
        try:
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length) or '{}')
            action = body.get('action', '')
            windows_host = str(body.get('windowsHost', default_windows_host())).strip()
            try:
                serial_tcp_port = int(body.get('serialTcpPort', 8766))
                camera_tcp_port = int(body.get('cameraTcpPort', 8767))
            except (TypeError, ValueError):
                self.send_json({'ok': False, 'message': 'TCP 端口必须是数字'}, 400)
                return
            if not windows_host or not (1 <= serial_tcp_port <= 65535 and 1 <= camera_tcp_port <= 65535):
                self.send_json({'ok': False, 'message': 'Windows 主机或 TCP 端口无效'}, 400)
                return
            if action == 'start_driver':
                if controller.driver_running():
                    result = (False, '机械臂驱动已经在运行')
                else:
                    result = controller.start('机械臂驱动', [
                        'ros2', 'launch', 'learm_driver', 'learm_bringup.launch.py',
                        'serial_transport:=tcp', f'tcp_host:={windows_host}',
                        f'tcp_port:={serial_tcp_port}',
                    ])
            elif action == 'start_vla':
                result = controller.start('VLA 记录节点', [
                    'ros2', 'launch', 'learm_vla_bridge', 'recording.launch.py',
                    'camera_source:=topic', f'windows_host:={windows_host}',
                    f'camera_tcp_port:={camera_tcp_port}',
                    f"dataset_root:={body.get('dataset', DEFAULT_DATASET)}",
                    f"task:={body.get('task', '')}",
                ])
            elif action == 'stop_driver':
                result = controller.stop('机械臂驱动')
            elif action == 'stop_vla':
                result = controller.stop('VLA 记录节点')
            elif action in ('start_episode', 'stop_episode', 'abort_episode'):
                result = controller.service(action)
            elif action in ('move_joints', 'move_pose', 'set_gripper', 'get_arm_status', 'emergency_stop', 'clear_estop'):
                if not controller.driver_running():
                    result = (False, '请先启动机械臂驱动')
                elif action == 'move_joints':
                    names = body.get('joint_names')
                    positions = body.get('positions_deg')
                    if (not isinstance(names, list) or not isinstance(positions, list)
                            or len(names) != len(positions) or not 1 <= len(names) <= 5
                            or any(name not in {'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'} for name in names)
                            or len(set(names)) != len(names)):
                        result = (False, '关节命令必须包含不重复的 joint_2 到 joint_6')
                    else:
                        try:
                            positions = [float(value) for value in positions]
                            duration = int(body.get('duration_ms', 500))
                        except (TypeError, ValueError):
                            result = (False, '关节参数必须是数字')
                        else:
                            if (not all(math.isfinite(value) and -90.0 <= value <= 90.0 for value in positions)
                                    or not 20 <= duration <= 30000):
                                result = (False, '关节角度范围为 -90 到 90 度，时长为 20 到 30000 ms')
                            else:
                                result = controller.driver_service(
                                    '/learm_driver/move_joints',
                                    'learm_driver/srv/MoveJoints',
                                    {
                                        'joint_names': names,
                                        'positions_rad': [math.radians(value) for value in positions],
                                        'duration_ms': duration,
                                    },
                                )
                elif action == 'move_pose':
                    positions = body.get('positions_deg')
                    if not isinstance(positions, list) or len(positions) != 5:
                        result = (False, '需要提供 joint_2 到 joint_6 的 5 个角度')
                    else:
                        try:
                            positions = [float(value) for value in positions]
                            opening = float(body.get('opening', 0.0))
                            duration = int(body.get('duration_ms', 500))
                        except (TypeError, ValueError):
                            result = (False, '姿态参数必须是数字')
                        else:
                            if (not all(math.isfinite(value) and -90.0 <= value <= 90.0 for value in positions)
                                    or not math.isfinite(opening) or not 0.0 <= opening <= 1.0
                                    or not 20 <= duration <= 30000):
                                result = (False, '角度范围为 -90 到 90 度，夹爪为 0 到 100%，时长为 20 到 30000 ms')
                            else:
                                result = controller.driver_service(
                                    '/learm_driver/move_pose',
                                    'learm_driver/srv/MovePose',
                                    {
                                        'opening': opening,
                                        'positions_rad': [math.radians(value) for value in positions],
                                        'duration_ms': duration,
                                    },
                                )
                elif action == 'set_gripper':
                    try:
                        opening = float(body.get('opening', 0.0))
                        duration = int(body.get('duration_ms', 500))
                    except (TypeError, ValueError):
                        result = (False, '夹爪参数必须是数字')
                    else:
                        if (not math.isfinite(opening) or not 0.0 <= opening <= 1.0
                                or not 20 <= duration <= 30000):
                            result = (False, '夹爪范围为 0 到 100%，时长为 20 到 30000 ms')
                        else:
                            result = controller.driver_service(
                                '/learm_driver/set_gripper',
                                'learm_driver/srv/SetGripper',
                                {'opening': opening, 'duration_ms': duration},
                            )
                elif action == 'get_arm_status':
                    result = controller.driver_service(
                        '/learm_driver/get_status', 'learm_driver/srv/GetArmStatus', {})
                else:
                    service_name = '/learm_driver/' + action
                    result = controller.driver_service(service_name, 'std_srvs/srv/Trigger', {})
            else:
                result = (False, '未知操作')
            self.send_json({'ok': result[0], 'message': result[1]})
        except Exception as exc:
            self.send_json({'ok': False, 'message': html.escape(str(exc))}, 400)


if __name__ == '__main__':
    print(f'LeArm VLA web GUI: http://{HOST}:{PORT}')
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
