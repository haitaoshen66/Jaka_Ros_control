#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VR手柄数据WebSocket服务器 - 适配Oculus数据格式
运行在获取VR数据的主机上
"""

"""
上下：x
前后：y
左右：z
"""
import asyncio
import websockets
import json
import threading
import time
import numpy as np
from typing import Dict, Any, Optional
import logging
import math
from scipy.spatial.transform import Rotation as R  # 新增
import csv  # 新增

# 导入你的原始代码
from ppadb.client import Client as AdbClient
import os
import sys
import socket

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# # 新增：添加文件日志处理器，将日志写入move.log
# file_handler = logging.FileHandler(os.path.join(os.path.dirname(__file__), 'move.log'), encoding='utf-8')
# file_handler.setLevel(logging.INFO)
# formatter = logging.Formatter('%(levelname)s:%(name)s:%(message)s')
# file_handler.setFormatter(formatter)
# if not any(isinstance(h, logging.FileHandler) and h.baseFilename == file_handler.baseFilename for h in logger.handlers):
#     logger.addHandler(file_handler)
class FPSCounter:
    def __init__(self):
        current_time = time.time()
        self.start_time_for_display = current_time
        self.last_time = current_time
        self.x = 5  # displays the frame rate every X second
        self.time_between_calls = []
        self.elements_for_mean = 50

    def getAndPrintFPS(self, print_fps=True):
        current_time = time.time()
        self.time_between_calls.append(1.0/(current_time - self.last_time + 1e-9))
        if len(self.time_between_calls) > self.elements_for_mean:
            self.time_between_calls.pop(0)
        self.last_time = current_time
        frequency = np.mean(self.time_between_calls)
        if (current_time - self.start_time_for_display) > self.x and print_fps:
            print("Frequency: {}Hz".format(int(frequency)))
            self.start_time_for_display = current_time
        return frequency
    
def parse_buttons(text):
    split_text = text.split(',')
    buttons = {}
    if 'R' in split_text: # right hand if available
        split_text.remove('R') # remove marker
        buttons.update({'A': False,
                        'B': False,
                        'RThU': False, # indicates that right thumb is up from the rest position
                        'RJ': False, # joystick pressed
                        'RG': False, # boolean value for trigger on the grip (delivered by SDK)
                        'RTr': False # boolean value for trigger on the index finger (delivered by SDK)
                        })
        # besides following keys are provided:
        # 'rightJS' / 'leftJS' - (x, y) position of joystick. x, y both in range (-1.0, 1.0)
        # 'rightGrip' / 'leftGrip' - float value for trigger on the grip in range (0.0, 1.0)
        # 'rightTrig' / 'leftTrig' - float value for trigger on the index finger in range (0.0, 1.0)

    if 'L' in split_text: # left hand accordingly
        split_text.remove('L') # remove marker
        buttons.update({'X': False, 'Y': False, 'LThU': False, 'LJ': False, 'LG': False, 'LTr': False})
    for key in buttons.keys():
        if key in list(split_text):
            buttons[key] = True
            split_text.remove(key)
    for elem in split_text:
        split_elem = elem.split(' ')
        if len(split_elem) < 2:
            continue
        key = split_elem[0]
        value = tuple([float(x) for x in split_elem[1:]])
        buttons[key] = value
    return buttons

def send_command_and_receive_response(command, host, port):
    try:
        # Create a TCP/IP socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            # Connect to the server
            sock.connect((host, port))
            
            # Send the command
            sock.sendall(command.encode('utf-8'))
            
            # Receive the response
            response = sock.recv(4096).decode('utf-8')
            
            # Parse the JSON response
            response_json = json.loads(response)
            return response_json
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

class VRDataEncoder(json.JSONEncoder):
    """自定义JSON编码器，处理numpy数组"""

    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, np.float32):
            return float(obj)
        elif isinstance(obj, np.int32):
            return int(obj)
        elif isinstance(obj, np.float64):
            return float(obj)
        elif isinstance(obj, np.bool_):
            return bool(obj)
        return super().default(obj)


def eprint(*args, **kwargs):
    RED = "\033[1;31m"
    sys.stderr.write(RED)
    # print(*args, file=sys.stderr, **kwargs)
    RESET = "\033[0;0m"
    sys.stderr.write(RESET)


class OculusReader:
    """集成你的原始OculusReader类"""

    def __init__(
        self,
        ip_address=None,
        port=5555,
        APK_name="com.rail.oculus.teleop",
        print_FPS=False,
        run=True,
    ):
        self.running = False
        self.last_transforms = {}
        self.last_buttons = {}
        self._lock = threading.Lock()
        self.tag = "wE9ryARX"

        self.ip_address = ip_address
        self.port = port
        self.APK_name = APK_name
        self.print_FPS = print_FPS
        if self.print_FPS:
            self.fps_counter = FPSCounter()
        self.device = self.get_device()
        self.install(verbose=False)
        if run:
            self.run()

    def __del__(self):
        self.stop()

    def run(self):
        self.running = True
        self.device.shell(
            'am start -n "com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity" -a android.intent.action.MAIN -c android.intent.category.LAUNCHER'
        )
        self.thread = threading.Thread(
            target=self.device.shell, args=("logcat -T 0", self.read_logcat_by_line)
        )
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join()

    def get_network_device(self, client, retry=0):
        try:
            client.remote_connect(self.ip_address, self.port)
        except RuntimeError:
            os.system("adb devices")
            client.remote_connect(self.ip_address, self.port)
        device = client.device(self.ip_address + ":" + str(self.port))

        if device is None:
            if retry == 1:
                os.system("adb tcpip " + str(self.port))
            if retry == 2:
                eprint(
                    "Make sure that device is running and is available at the IP address specified as the OculusReader argument `ip_address`."
                )
                eprint("Currently provided IP address:", self.ip_address)
                eprint("Run `adb shell ip route` to verify the IP address.")
                exit(1)
            else:
                self.get_network_device(client=client, retry=retry + 1)
        return device

    def get_usb_device(self, client):
        try:
            devices = client.devices()
        except RuntimeError:
            os.system("adb devices")
            devices = client.devices()
        for device in devices:
            if device.serial.count(".") < 3:
                return device
        eprint(
            "Device not found. Make sure that device is running and is connected over USB"
        )
        eprint("Run `adb devices` to verify that the device is visible.")
        exit(1)

    def get_device(self):
        client = AdbClient(host="127.0.0.1", port=5037)
        if self.ip_address is not None:
            return self.get_network_device(client)
        else:
            return self.get_usb_device(client)

    def install(self, APK_path=None, verbose=True, reinstall=False):
        try:
            installed = self.device.is_installed(self.APK_name)
            if not installed or reinstall:
                if APK_path is None:
                    APK_path = os.path.join(
                        os.path.dirname(os.path.realpath(__file__)),
                        "APK",
                        "teleop-debug.apk",
                    )
                success = self.device.install(APK_path, test=True, reinstall=reinstall)
                installed = self.device.is_installed(self.APK_name)
                if installed and success:
                    print("APK installed successfully.")
                else:
                    eprint("APK install failed.")
            elif verbose:
                print("APK is already installed.")
        except RuntimeError:
            eprint("Device is visible but could not be accessed.")
            eprint(
                "Run `adb devices` to verify that the device is visible and accessible."
            )
            eprint(
                'If you see "no permissions" next to the device serial, please put on the Oculus Quest and allow the access.'
            )
            exit(1)

    def uninstall(self, verbose=True):
        try:
            installed = self.device.is_installed(self.APK_name)
            if installed:
                success = self.device.uninstall(self.APK_name)
                installed = self.device.is_installed(self.APK_name)
                if not installed and success:
                    print("APK uninstall finished.")
                else:
                    eprint("APK uninstall failed")
            elif verbose:
                print("APK is not installed.")
        except RuntimeError:
            eprint("Device is visible but could not be accessed.")
            exit(1)

    @staticmethod
    def process_data(string):
        try:
            transforms_string, buttons_string = string.split("&")
        except ValueError:
            return None, None
        split_transform_strings = transforms_string.split("|")
        transforms = {}
        for pair_string in split_transform_strings:
            transform = np.empty((4, 4))
            pair = pair_string.split(":")
            if len(pair) != 2:
                continue
            left_right_char = pair[0]
            transform_string = pair[1]
            values = transform_string.split(" ")
            c = 0
            r = 0
            count = 0
            for value in values:
                if not value:
                    continue
                transform[r][c] = float(value)
                c += 1
                if c >= 4:
                    c = 0
                    r += 1
                count += 1
            if count == 16:
                transforms[left_right_char] = transform
        buttons = parse_buttons(buttons_string)
        return transforms, buttons

    def extract_data(self, line):
        output = ""
        if self.tag in line:
            try:
                output += line.split(self.tag + ": ")[1]
            except ValueError:
                pass
        return output

    def get_transformations_and_buttons(self):
        with self._lock:
            return self.last_transforms, self.last_buttons

    def read_logcat_by_line(self, connection):
        file_obj = connection.socket.makefile()
        while self.running:
            try:
                line = file_obj.readline().strip()
                data = self.extract_data(line)
                if data:
                    transforms, buttons = OculusReader.process_data(data)
                    with self._lock:
                        self.last_transforms, self.last_buttons = transforms, buttons
                    if self.print_FPS:
                        self.fps_counter.getAndPrintFPS()
            except UnicodeDecodeError:
                pass
        file_obj.close()
        connection.close()


class VRWebSocketServer:
    def __init__(self, host="0.0.0.0", port=8765, oculus_ip=None):
        self.host = host
        self.port = port
        self.oculus_ip = oculus_ip

        # WebSocket相关
        self.clients = set()
        self.server = None
        self.running = False

        # VR数据相关
        self.oculus_reader = None
        self.data_thread = None
        self.current_data = {
            "right_hand_delta": None,  # 右手相对位移和欧拉角变化量 [dx, dy, dz, droll, dpitch, dyaw]
            "timestamp": 0,
            "status": "disconnected",
            "frame_count": 0,
        }
        self.data_lock = threading.Lock()
        self.right_hand_total = np.array([0,0,0,0.154,-0.396,0.211])

        # 性能监控
        self.fps_counter = FPSCounter()
        self.last_broadcast_time = time.time()
        self.data_rate = 0

        # 新增：初始化csv文件，写入表头
        csv_path = os.path.join(os.path.dirname(__file__), 'right_hand_xyz.csv')
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['frame', 'roll', 'pitch', 'yaw', 'x', 'y', 'z'])

    def calMatrix(self, matrix,init_matrix):
        matrix = np.array(matrix)
        # 定义初始状态旋转矩阵（标准姿态）
        initial_rotation = np.array(init_matrix)
        # 提取当前3*3旋转矩阵
        current_rotation = matrix[:3, :3]
        # 计算相对旋转矩阵（当前旋转相对于初始旋转）
        # R_relative = R_current * R_initial^(-1)
        relative_rotation = np.matmul(current_rotation, np.linalg.inv(initial_rotation))

        # 提取原始位置 (单位: m)
        position_m = matrix[:3, 3]
        
        # 转换为mm
        position_mm = position_m * 1000.0
        
        # vr->jaka
        transform_matrix = np.array([
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])
        # 使用矩阵乘法进行坐标系转换
        transformed_position = np.matmul(transform_matrix, position_mm)
        robot_relative_rotation = np.matmul(np.matmul(transform_matrix, relative_rotation), transform_matrix.T)
        # ----- 新增：机器人夹爪姿态处理 -----
        
        # 创建机器人夹爪的初始姿态矩阵 (欧拉角xyz顺序：π/2, 0, π/2)
        # 计算旋转矩阵的三个基本旋转
        init_rx,init_ry,init_rz=-0.78,-1.57,-2.36
        cx, sx = np.cos(init_rx), np.sin(init_rx) 
        cy, sy = np.cos(init_ry), np.sin(init_ry)   
        cz, sz = np.cos(init_rz), np.sin(init_rz)
        
        # 构建旋转矩阵 (按xyz顺序)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        
        # 组合旋转矩阵：Rxyz = Rz * Ry * Rx
        robot_initial_rotation = np.matmul(np.matmul(Rz, Ry), Rx)
        
        # 将相对旋转矩阵应用到机器人夹爪的初始姿态上
        robot_final_rotation = np.matmul(robot_relative_rotation, robot_initial_rotation)
        
        # 从最终旋转矩阵中提取欧拉角
        robot_roll = math.atan2(robot_final_rotation[2, 1], robot_final_rotation[2, 2])
        robot_pitch = math.atan2(-robot_final_rotation[2, 0], 
                             math.sqrt(robot_final_rotation[2, 1]**2 + robot_final_rotation[2, 2]**2))
        robot_yaw = math.atan2(robot_final_rotation[1, 0], robot_final_rotation[0, 0])

        # 转换为度数
        robot_roll_deg = math.degrees(robot_roll)
        robot_pitch_deg = math.degrees(robot_pitch)
        robot_yaw_deg = math.degrees(robot_yaw)

        return robot_roll_deg,robot_pitch_deg,robot_yaw_deg




    def format_oculus_data(self, transforms, buttons):
        """格式化Oculus数据为右手相对位移和旋转变化量"""
        formatted_data = {
            "right_hand_delta": None,  # 右手相对位移和欧拉角变化量 [dx, dy, dz, droll, dpitch, dyaw]
            "right_hand_total": None,  # 新增
            "timestamp": time.time(),
        }
        # 只处理右手数据
        if transforms and 'r' in transforms:
            transform_matrix = transforms['r']

            if transform_matrix is not None:
                # 提取位置 [x, y, z]
                position = transform_matrix[:3, 3]
                # 提取旋转矩阵
                rotation_matrix = transform_matrix[:3, :3]
                r = R.from_matrix(rotation_matrix)
                roll, pitch, yaw = r.as_euler('xyz', degrees=True)
                # droll, dpitch, dyaw = self.calMatrix(transform_matrix,)
                # 记录当前帧
                if not hasattr(self, '_prev_right'):  # 首帧
                    self._prev_right = (position.copy(), rotation_matrix.copy(), roll, pitch, yaw)
                    delta = np.array([0,0,0,0,0,0])
                else:
                    prev_pos, prev_rot, prev_roll, prev_pitch, prev_yaw = self._prev_right
                    # 位置变化量
                    dx, dy, dz = position - prev_pos
                
                    droll, dpitch, dyaw = self.calMatrix(transform_matrix,prev_rot)
                    cx = -dz
                    cy = -dx
                    cz = dy
                    # fx = -cz
                    # fy = cx
                    # fz = -cy
                    delta = np.array([0, 0, 0, cx*0.5, cy*0.5, cz*0.5])
                    self._prev_right = (position.copy(), rotation_matrix.copy(), roll, pitch, yaw)
                # 累加
                self.right_hand_total += delta
                self.right_hand_total[0] = droll
                self.right_hand_total[1] = dpitch
                self.right_hand_total[2] = dyaw
                formatted_data["right_hand_delta"] = delta.tolist()
                formatted_data["right_hand_total"] = self.right_hand_total.tolist()
                # print(formatted_data["right_hand_total"])
        # if buttons and 'rightJS' in buttons:
        #     y = buttons['rightJS'][1]
        #     self.vrun(y)
        # if buttons and 'leftJS' in buttons:
        #     x = buttons['leftJS'][0]
        #     self.rrun(x)
        return formatted_data

    def vrun(self, y):
        v = 0
        r = 0
        vrun = 0.5
        if y > 0:
            v=vrun
        else: v=-vrun
        param_str = f"angular_velocity={r}&linear_velocity={v}"
        self.sendRequest(param_str)
    
    def rrun(self, x):
        v = 0
        r = 0
        vrun = 0.5
        rrun = 0.3
        if x > 0:
            r = rrun
        else: r = -rrun
        param_str = f"angular_velocity={r}&linear_velocity={v}"
        self.sendRequest(param_str)

    def sendRequest(self, param_str):
        if param_str:
            cmd = COMMAND + "?" + param_str + "&uuid=123456"
            response = send_command_and_receive_response(cmd, HOST, PORT)
            if response:
                print("Received response:")
                print(json.dumps(response, indent=4))
            else:
                print("Failed to get a valid response.")
            

    async def register_client(self, websocket):
        """注册新的客户端连接"""
        self.clients.add(websocket)
        client_ip = websocket.remote_address[0]
        logger.info(f"客户端连接: {client_ip}")

        try:
            # 发送连接确认消息
            welcome_msg = {
                "type": "connection",
                "status": "connected",
                "message": "VR数据服务器连接成功",
                "server_info": {
                    "data_format": "right_hand_delta",
                    "data_structure": "[dx, dy, dz, droll, dpitch, dyaw]",
                    "update_rate": "100Hz",
                },
                "timestamp": time.time(),
            }
            await websocket.send(json.dumps(welcome_msg, cls=VRDataEncoder))

            # 保持连接
            async for message in websocket:
                try:
                    client_msg = json.loads(message)
                    if client_msg.get("type") == "ping":
                        pong_msg = {
                            "type": "pong",
                            "timestamp": time.time(),
                            "data_rate": self.data_rate,
                        }
                        await websocket.send(json.dumps(pong_msg, cls=VRDataEncoder))
                    elif client_msg.get("type") == "get_current_data":
                        # 客户端请求当前数据
                        with self.data_lock:
                            current_data_copy = self.current_data.copy()
                        current_data_copy["type"] = "current_data_response"
                        await websocket.send(
                            json.dumps(current_data_copy, cls=VRDataEncoder)
                        )

                except json.JSONDecodeError:
                    logger.warning(f"收到无效JSON消息: {message}")

        except websockets.exceptions.ConnectionClosed:
            logger.info(f"客户端断开连接: {client_ip}")
        except Exception as e:
            logger.error(f"客户端连接错误: {e}")
        finally:
            self.clients.discard(websocket)

    async def broadcast_data(self, data):
        """向所有连接的客户端广播数据"""
        if not self.clients:
            return

        try:
            json_data = json.dumps(data, cls=VRDataEncoder)
        except Exception as e:
            logger.error(f"数据序列化失败: {e}")
            return

        # 并发发送给所有客户端
        disconnected_clients = set()
        send_tasks = []

        for client in self.clients.copy():

            async def send_to_client(ws, data):
                try:
                    await ws.send(data)
                    return True
                except websockets.exceptions.ConnectionClosed:
                    disconnected_clients.add(ws)
                    return False
                except Exception as e:
                    logger.error(f"发送数据失败: {e}")
                    disconnected_clients.add(ws)
                    return False

            send_tasks.append(send_to_client(client, json_data))

        # 等待所有发送完成
        if send_tasks:
            await asyncio.gather(*send_tasks, return_exceptions=True)

        # 移除断开的客户端
        self.clients -= disconnected_clients

    def start_oculus_reader(self):
        """启动Oculus数据读取"""
        try:
            self.oculus_reader = OculusReader(
                ip_address=self.oculus_ip, print_FPS=False, run=True
            )
            logger.info("Oculus Reader启动成功")

            with self.data_lock:
                self.current_data["status"] = "connected"

        except Exception as e:
            logger.error(f"Oculus Reader启动失败: {e}")
            with self.data_lock:
                self.current_data["status"] = "error"

    def data_collection_loop(self):
        """数据收集循环"""
        logger.info("开始VR数据收集循环")
        frame_count = 0

        while self.running:
            try:
                if self.oculus_reader:
                    # 获取原始VR数据
                    transforms, buttons = (
                        self.oculus_reader.get_transformations_and_buttons()
                    )
                    # 格式化数据
                    formatted_data = self.format_oculus_data(transforms, buttons)
                    formatted_data["frame_count"] = frame_count
                    formatted_data["status"] = "active"
                    # 更新数据
                    with self.data_lock:
                        self.current_data.update(formatted_data)
                    # logger.info(f"Updated right hand delta: {self.current_data['right_hand_delta']}")
                    logger.info(f"Updated right hand total: {self.current_data['right_hand_total'][3:]}")
                    # 追加写入csv，不再覆盖
                    xyz = self.current_data.get('right_hand_total', [0,0,0,0,0,0])[:]
                    csv_path = os.path.join(os.path.dirname(__file__), 'right_hand_xyz.csv')
                    with open(csv_path, 'a', newline='') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow([frame_count] + list(xyz))

                    frame_count += 1
                    # 性能计数
                    if hasattr(self, "fps_counter"):
                        self.fps_counter.getAndPrintFPS()
                time.sleep(0.3)  # 100Hz更新频率
            except Exception as e:
                logger.error(f"数据收集错误: {e}")
                time.sleep(0.5)

    async def data_broadcast_loop(self):
        """数据广播循环"""
        logger.info("开始WebSocket数据广播循环")
        broadcast_count = 0
        start_time = time.time()
        logger.info(f"self.running: {self.running}")
        while self.running:
            try:
                # 获取当前数据
                with self.data_lock:
                    data_to_send = self.current_data.copy()

                # 添加消息类型和广播信息
                data_to_send.update(
                    {
                        "type": "vr_data",
                        "broadcast_count": broadcast_count,
                        "client_count": len(self.clients),
                        "server_timestamp": time.time(),
                    }
                )

                # 广播给所有客户端
                if self.clients:
                    await self.broadcast_data(data_to_send)

                broadcast_count += 1

                # 计算数据传输率
                current_time = time.time()
                if current_time - start_time >= 1.0:
                    self.data_rate = broadcast_count / (current_time - start_time)
                    broadcast_count = 0
                    start_time = current_time

                await asyncio.sleep(1)  # 50Hz广播频率

            except Exception as e:
                logger.error(f"数据广播错误: {e}")
                await asyncio.sleep(0.03)

    async def start_server(self):
        """启动WebSocket服务器"""
        logger.info(f"启动WebSocket服务器: {self.host}:{self.port}")

        self.running = True

        # 启动Oculus数据读取
        self.start_oculus_reader()

        # 启动数据收集线程
        self.data_thread = threading.Thread(target=self.data_collection_loop)
        self.data_thread.daemon = True
        self.data_thread.start()

        # 启动WebSocket服务器
        self.server = await websockets.serve(
            self.register_client,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=10,
            max_size=10**6,  # 1MB最大消息大小
            compression=None,  # 禁用压缩以减少延迟
        )

        logger.info(f"WebSocket服务器运行在 ws://{self.host}:{self.port}")
        logger.info("数据格式：Oculus VR手柄数据")

        # 启动数据广播循环
        await self.data_broadcast_loop()

    async def stop_server(self):
        """停止服务器"""
        logger.info("正在停止WebSocket服务器...")

        self.running = False

        # 停止Oculus Reader
        if self.oculus_reader:
            self.oculus_reader.stop()

        # 等待数据线程结束
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=5)

        # 关闭所有客户端连接
        if self.clients:
            await asyncio.gather(
                *[client.close() for client in self.clients], return_exceptions=True
            )

        # 停止服务器
        if self.server:
            self.server.close()
            await self.server.wait_closed()

        logger.info("WebSocket服务器已停止")


async def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(
        description="VR手柄数据WebSocket服务器 - Oculus版本"
    )
    parser.add_argument("--host", default="0.0.0.0", help="服务器IP地址")
    parser.add_argument("--port", type=int, default=8765, help="服务器端口")
    parser.add_argument("--oculus-ip", help="Oculus设备IP地址（USB连接时不需要）")

    args = parser.parse_args()

    # 创建并启动服务器
    server = VRWebSocketServer(host=args.host, port=args.port, oculus_ip=args.oculus_ip)

    try:
        await server.start_server()
    except KeyboardInterrupt:
        logger.info("收到停止信号")
    except Exception as e:
        logger.error(f"服务器错误: {e}")
    finally:
        await server.stop_server()


if __name__ == "__main__":
    HOST = "192.168.10.10"  # Replace with the server's IP address
    PORT = 31001        # Replace with the server's port number
    COMMAND = "/api/joy_control"
    asyncio.run(main())
