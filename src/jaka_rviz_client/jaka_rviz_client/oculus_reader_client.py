#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import threading
import time
import os
from ppadb.client import Client as AdbClient
import sys
import rclpy
from rclpy.node import Node
from jaka_msgs.srv import GetIK
import time
import numpy as np
import asyncio
import math
import json
import socket
from jaka_msgs.srv import ServoMoveEnable,ServoMove
import requests
from geometry_msgs.msg import TwistStamped
# import jkrc


# 控制底盘
HOST = "192.168.10.10"  # Replace with the server's IP address
PORT = 31001        # Replace with the server's port number
COMMAND = "/api/joy_control"

# 控制升降
API_BASE_URL = "http://192.168.10.90:5000/api/extaxis"

ENABLE_URL = f"{API_BASE_URL}/enable"
RESET_URL = f"{API_BASE_URL}/reset"
MOVETO_URL = f"{API_BASE_URL}/moveto"
STATUS_URL = f"{API_BASE_URL}/status"
# 升降控制参数
LIFT_STEP = 10.0  # 每次按键移动的距离（mm）
MIN_HEIGHT = 0.0   # 最小高度
MAX_HEIGHT = 300.0 # 最大高度

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
    
    
def eprint(*args, **kwargs):
    RED = "\033[1;31m"  
    sys.stderr.write(RED)
    print(*args, file=sys.stderr, **kwargs)
    RESET = "\033[0;0m"
    sys.stderr.write(RESET)

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GetIK, 'jaka_rviz_driver/get_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetIK.Request()

    def send_request(self, cartesian_pose):
        self.req.cartesian_pose = cartesian_pose
        return self.cli.call_async(self.req)

class JakaServoClientAsync(Node):

    def __init__(self):
        super().__init__('jaka_servo_p_clientpy')
        self.servo_move_enable_client  = self.create_client(ServoMoveEnable, '/jaka_driver/servo_move_enable')
        self.servo_p_client = self.create_client(ServoMove, '/jaka_driver/servo_p')
        self.gripper_init_client = self.create_client(ServoMoveEnable, '/jaka_driver/gripper_init')
        self.gripper_control_client = self.create_client(ServoMoveEnable, '/jaka_driver/gripper_control')
        while not self.servo_move_enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.gripper_init_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('夹爪初始化服务不可用，继续等待...')
            
        while not self.gripper_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('夹爪控制服务不可用，继续等待...')
        self.req = ServoMove.Request()
        
        
    def init_gripper(self):
        self.get_logger().info('正在初始化夹爪...')
        request = ServoMoveEnable.Request()
        request.enable = True  # 参数值不重要，服务会忽略它
        return self.gripper_init_client.call_async(request)
    
    # 添加夹爪控制方法
    def control_gripper(self, open_gripper):
        request = ServoMoveEnable.Request()
        request.enable = open_gripper  # True表示打开，False表示关闭
        return self.gripper_control_client.call_async(request)

    def enable_servo_mode(self):
        enable_request = ServoMoveEnable.Request()
        enable_request.enable = True
        return self.servo_move_enable_client.call_async(enable_request)

    def send_request(self,pose_delta):# pose_delta[]
        self.req.pose = pose_delta
        return self.servo_p_client.call_async(self.req)
    
class OculusDataRosThread(threading.Thread):
    def __init__(self, oculus_reader, ros_client, interval=0.0):
        super().__init__()
        self.oculus_reader = oculus_reader
        self.ros_client = ros_client
        self.interval = interval
        self.running = True
        self.right_hand_total = np.array([0,0,0,0.154,-0.396,0.211])
        self.prev_right = None
        self.init_matrix = None
        self._service_busy = False
        self.loop = None
        self.gripper_initialized = False
        self.gripper_state = None
        self.save_current_pose = True
        self.current_tool_pose = None
        self.tool_pos_subscription = self.ros_client.create_subscription(
            TwistStamped,
            '/jaka_driver/tool_position',
            self.tool_position_callback,
            5)
        
        self.robot_initial_rotation = None 
        self.prev_robot_matrix = None
        ang_x=2.61
        ang_y=-0.456
        ang_z=0.91
        self.init_rx = ang_x
        self.init_ry = ang_y
        self.init_rz = ang_z
        
        # 更新机器人初始旋转矩阵
        cx, sx = np.cos(self.init_rx), np.sin(self.init_rx) 
        cy, sy = np.cos(self.init_ry), np.sin(self.init_ry)   
        cz, sz = np.cos(self.init_rz), np.sin(self.init_rz)
        
        # 构建旋转矩阵 (按xyz顺序)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        
        # 组合旋转矩阵：Rxyz = Rz * Ry * Rx
        self.robot_initial_rotation = np.matmul(np.matmul(Rz, Ry), Rx)
        self.prev_robot_matrix = self.robot_initial_rotation.copy()
        # 添加线程锁和事件
        self._transforms_lock = threading.Lock()
        self._buttons_lock = threading.Lock()
        self._other_lock = threading.Lock()
        
        self._transforms_event = threading.Event()
        self._buttons_event = threading.Event()
        self._other_event = threading.Event()

        # 创建两个独立的线程
        self.transforms_thread = threading.Thread(target=self._process_transforms_loop)
        self.buttons_thread = threading.Thread(target=self._process_buttons_loop)
        self.otherButton_thread = threading.Thread(target=self._process_otherButton_loop)
        
        # 数据缓存
        self._latest_transforms = None
        self._latest_buttons = None
        self._latest_other = None

        self._transforms_ready = False
        self._buttons_ready = False
        self._other_ready = False

        # 添加transforms处理控制标志
        self._transforms_enabled = True
    def tool_position_callback(self, msg):
        """处理机器人末端位姿数据"""
        self.current_tool_pose = msg
        print(f"current_tool_pose :{self.current_tool_pose }")
        # 如果需要保存当前位姿
        if self.save_current_pose and self.current_tool_pose:
            self.save_current_pose = False
            
            # 获取角度值并转换为弧度
            ang_x = math.radians(self.current_tool_pose.twist.angular.x)
            ang_y = math.radians(self.current_tool_pose.twist.angular.y)
            ang_z = math.radians(self.current_tool_pose.twist.angular.z)
            
            print(f"已获取机器人末端位姿角度 (弧度): rx={ang_x:.4f}, ry={ang_y:.4f}, rz={ang_z:.4f}")
            
            # 更新机器人初始角度
            self.init_rx = ang_x
            self.init_ry = ang_y
            self.init_rz = ang_z
            
            # 更新机器人初始旋转矩阵
            cx, sx = np.cos(self.init_rx), np.sin(self.init_rx) 
            cy, sy = np.cos(self.init_ry), np.sin(self.init_ry)   
            cz, sz = np.cos(self.init_rz), np.sin(self.init_rz)
            
            # 构建旋转矩阵 (按xyz顺序)
            Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
            Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
            
            # 组合旋转矩阵：Rxyz = Rz * Ry * Rx
            self.robot_initial_rotation = np.matmul(np.matmul(Rz, Ry), Rx)
            self.prev_robot_matrix = self.robot_initial_rotation.copy()
            
            print("✅ 已更新机器人初始旋转矩阵")

    async def call_ros_service(self, cartesian_pose):
        """调用ROS2服务"""
        if self.ros_client is None:
            print("ROS2客户端未初始化")
            return 
        if self._service_busy:
            return
            
        self._service_busy = True
        try:
            # 分离XYZ和角度变化
            xyz_delta = cartesian_pose[:3]
            angle_delta = cartesian_pose[3:]
            
            # 处理XYZ变化
            max_xyz = max(abs(x) for x in xyz_delta)
            max_angle = max(abs(a) for a in angle_delta)

            n_xyz = 1
            if max_xyz > 5.0:
                n_xyz = int(np.ceil(max_xyz / 5.0))
            n_angle = 1
            if max_angle > 0.2:
                n_angle = int(np.ceil(max_angle / 0.2))
            
            n_delta = max(n_xyz, n_angle)
            split_xyz_delta = [x / n_delta for x in xyz_delta]
            split_angle_delta = [a / n_delta for a in angle_delta]

            for i in range(n_delta):
                xyz_request = split_xyz_delta + split_angle_delta
                future = self.ros_client.send_request(list(xyz_request))
                rclpy.spin_until_future_complete(self.ros_client, future)
                response = future.result()
                
            self.ros_client.get_logger().info(f'完成请求处理，原始姿态: {cartesian_pose}')
        except Exception as e:
            print(f"ROS2服务调用失败: {e}")
        finally:
            self._service_busy = False
    
    def _process_transforms_loop(self):
        """处理transforms的独立线程循环 - 串行处理，跳过中间数据"""
        while self.running:
            # 检查transforms处理是否被禁用
            if not self._transforms_enabled:
                time.sleep(0.01)
                continue
                
            current_transforms = None
            print("当前transforms数据:", self._latest_transforms)
            # print(self._transforms_ready)
            # 获取最新的transforms数据
            with self._transforms_lock:
                if self._transforms_ready and self._latest_transforms and 'r' in self._latest_transforms:
                    current_transforms = self._latest_transforms.copy()
                    # 标记数据已处理，等待下一次更新
                    self._transforms_ready = False
            # print(current_transforms)

            # 处理获取到的transforms数据
            if current_transforms:
                transform_matrix = current_transforms['r']
                if transform_matrix is not None:
                    position = transform_matrix[:3, 3]
                    rotation_matrix = transform_matrix[:3, :3]
                    
                    if self.prev_right is None:
                        self.prev_right = (position.copy(), rotation_matrix.copy(), -0.785, -1.57, -2.36)
                        self.init_matrix = rotation_matrix.copy()  # 保存为实例变量
                        abs_roll, abs_pitch, abs_yaw = 0.0, 0.0, 0.0
                        delta_roll, delta_pitch, delta_yaw = 0.0, 0.0, 0.0
                    else:
                        prev_pos, prev_rot, prev_roll, prev_pitch, prev_yaw = self.prev_right
                        dx, dy, dz = position - prev_pos
                        current_robot_matrix,delta_roll,delta_pitch,delta_yaw,abs_roll, abs_pitch, abs_yaw = self.calMatrix(transform_matrix)
                        cx = -dz*1000*1.2
                        cy = -dx*1000*1.2
                        cz = dy*1000*1.2
                        self.prev_robot_matrix = current_robot_matrix.copy()
                        delta_ = np.array([ cx, cy, cz,self.normalize_angle(delta_yaw), -self.normalize_angle(delta_pitch),self.normalize_angle(delta_roll)])
                        self.prev_right = (position.copy(), rotation_matrix.copy(), abs_roll, abs_pitch, abs_yaw)
                        cartesian_pose = [float(x) for x in delta_]
                        print(f"Transforms处理: {cartesian_pose}")
                        
                        # 创建新的事件循环来处理异步调用
                        try:
                            loop = asyncio.new_event_loop()
                            asyncio.set_event_loop(loop)
                            start = time.time()
                            loop.run_until_complete(self.call_ros_service(cartesian_pose))
                            end = time.time()
                            print(f"Transforms处理时间: {end - start:.4f}秒")
                        except Exception as e:
                            print(f"Transforms处理异常: {e}")
                        finally:
                            loop.close()
                    # else :
                    #     print("❌ prev_robot_matrix未初始化，无法处理transforms数据")
                    #     continue
            time.sleep(0.01)  # 短暂休眠避免过度占用CPU
    
    def _process_buttons_loop(self):
        """处理buttons的独立线程循环"""
        # self.enable_lift()
        if not self.gripper_initialized:
            future = self.ros_client.init_gripper()
            while not future.done():
                time.sleep(0.01)
            result = future.result()
            print(f"✅ 夹爪初始化完成: {result.message}")
            self.gripper_initialized = True

        while self.running:
            with self._buttons_lock:
                if self._buttons_ready and self._latest_buttons:
                    buttons = self._latest_buttons
                    # print(buttons)
                    # 处理右摇杆
                    if 'rightTrig' in buttons:
                        trigger_value = buttons['rightTrig'][0]
                        # 打印当前trigger值，便于调试
                        print(f"右手Trigger值: {trigger_value}")
                        
                        # 根据trigger值控制夹爪
                        if trigger_value > 0.5 and self.gripper_state != False:
                            future = self.ros_client.control_gripper(False)
                            # 等待服务调用完成
                            while not future.done():
                                time.sleep(0.01)
                            result = future.result()
                            self.gripper_state = False

                        
                        elif trigger_value <= 0.5  and self.gripper_state != True:
                            future = self.ros_client.control_gripper(True)
                            # 等待服务调用完成
                            while not future.done():
                                time.sleep(0.01)
                            result = future.result()
                            self.gripper_state = True
                    # if 'rightJS' in buttons:
                    #     x = buttons['rightJS'][0]
                    #     y = buttons['rightJS'][1]
                    #     if abs(x) > abs(y):
                    #         self.rrun(x)
                    #     elif abs(y) > abs(x):
                    #         self.vrun(y)
                    # if 'A' in buttons and buttons['A'] is True:
                    #     print("检测到A按钮，停止transforms处理")
                    #     self._transforms_enabled = False
                    #     self.prev_right = None
                    #     self.init_matrix = None
                    #     self._latest_transforms = None
                    #     self._transforms_ready = True
                    #     self.save_current_pose = True
                    # if 'B' in buttons and buttons['B'] is True:
                    #     print("检测到B按钮，重新校准transforms处理")
                    #     # 重置prev_right和init_matrix
                    #     self.prev_right = None
                    #     self.init_matrix = None
                    #     self._transforms_ready = False
                    #     self._transforms_enabled = True
                    
                    # if 'leftTrig' in buttons and buttons['leftTrig'][0] > 0.0:
                    #     self.move_up()
                    # if 'leftGrip' in buttons and buttons['leftGrip'][0] > 0.0:
                    #     self.move_down()
            time.sleep(0.01)  # 短暂休眠避免过度占用CPU
    
    def _process_otherButton_loop(self):
        """处理buttons的独立线程循环"""
        self.enable_lift()
        # if not self.gripper_initialized:
        #     future = self.ros_client.init_gripper()
        #     while not future.done():
        #         time.sleep(0.01)
        #     result = future.result()
        #     print(f"✅ 夹爪初始化完成: {result.message}")
        #     self.gripper_initialized = True

        while self.running:
            with self._other_lock:
                if self._other_ready and self._latest_other:
                    buttons = self._latest_other
                    print(buttons)
                    # # 处理右摇杆
                    # if 'rightTrig' in buttons:
                    #     trigger_value = buttons['rightTrig'][0]
                    #     # 打印当前trigger值，便于调试
                    #     print(f"右手Trigger值: {trigger_value}")
                        
                    #     # 根据trigger值控制夹爪
                    #     if trigger_value > 0.5 and self.gripper_state != False:
                    #         future = self.ros_client.control_gripper(False)
                    #         # 等待服务调用完成
                    #         while not future.done():
                    #             time.sleep(0.01)
                    #         result = future.result()
                    #         self.gripper_state = False

                        
                    #     elif trigger_value <= 0.5  and self.gripper_state != True:
                    #         future = self.ros_client.control_gripper(True)
                    #         # 等待服务调用完成
                    #         while not future.done():
                    #             time.sleep(0.01)
                    #         result = future.result()
                    #         self.gripper_state = True
                    if 'rightJS' in buttons:
                        x = buttons['rightJS'][0]
                        y = buttons['rightJS'][1]
                        if abs(x) > abs(y):
                            self.rrun(x)
                        elif abs(y) > abs(x):
                            self.vrun(y)
                    if 'A' in buttons and buttons['A'] is True:
                        print("检测到A按钮，停止transforms处理")
                        self._transforms_enabled = False
                        self.prev_right = None
                        self.init_matrix = None
                        self._latest_transforms = None
                        self._transforms_ready = True
                        self.save_current_pose = True
                    if 'B' in buttons and buttons['B'] is True:
                        print("检测到B按钮，重新校准transforms处理")
                        # 重置prev_right和init_matrix
                        self.prev_right = None
                        self.init_matrix = None
                        self._transforms_ready = False
                        self._transforms_enabled = True
                    
                    if 'leftTrig' in buttons and buttons['leftTrig'][0] > 0.0:
                        self.move_up()
                    if 'leftGrip' in buttons and buttons['leftGrip'][0] > 0.0:
                        self.move_down()
            time.sleep(0.01)  # 短暂休眠避免过度占用CPU

    def calMatrix(self,matrix):
        matrix = np.array(matrix)
        # 定义初始状态旋转矩阵（标准姿态）
        initial_rotation = np.array(self.init_matrix)
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
        
    
        
        # 将相对旋转矩阵应用到机器人夹爪的初始姿态上
        robot_final_rotation = np.matmul(robot_relative_rotation, self.robot_initial_rotation)
        delta_robot_rotation = np.matmul(np.linalg.inv(self.prev_robot_matrix), robot_final_rotation)
        delta_robot_roll = math.atan2(delta_robot_rotation[2, 1], delta_robot_rotation[2, 2])
        delta_robot_pitch = math.atan2(-delta_robot_rotation[2, 0], 
                                math.sqrt(delta_robot_rotation[2, 1]**2 + delta_robot_rotation[2, 2]**2))
        delta_robot_yaw = math.atan2(delta_robot_rotation[1, 0], delta_robot_rotation[0, 0])

        # 从最终旋转矩阵中提取欧拉角
        abs_robot_roll = math.atan2(robot_final_rotation[2, 1], robot_final_rotation[2, 2])
        abs_robot_pitch = math.atan2(-robot_final_rotation[2, 0], 
                                math.sqrt(robot_final_rotation[2, 1]**2 + robot_final_rotation[2, 2]**2))
        abs_robot_yaw = math.atan2(robot_final_rotation[1, 0], robot_final_rotation[0, 0])

        # 转换为度数
        # robot_roll_deg = math.degrees(robot_roll)
        # robot_pitch_deg = math.degrees(robot_pitch)
        # robot_yaw_deg = math.degrees(robot_yaw)

        return robot_final_rotation,delta_robot_roll,delta_robot_pitch,delta_robot_yaw,abs_robot_roll,abs_robot_pitch,abs_robot_yaw

    def enable_lift(self):
        """启用升降关节"""
        print("🔧 启用关节使能...")
        resp = requests.post(ENABLE_URL, json={"enable": 1})
        if resp.status_code == 200:
            self.is_enabled = True
            print("✅ 使能完成")
            self.get_current_height()
            return True
        else:
            print(f"❌ 启用失败: {resp.status_code}")
            return False
    
    def disable_lift(self):
        """关闭升降关节"""
        print("🛑 关闭使能...")
        resp = requests.post(ENABLE_URL, json={"enable": 0})
        if resp.status_code == 200:
            self.is_enabled = False
            print("✅ 已关闭使能")
        else:
            print(f"❌ 关闭使能失败: {resp.status_code}")
    
    def get_current_height(self):
        """获取当前升降位置"""
        resp = requests.get(STATUS_URL)
        if resp.status_code == 200:
            state = resp.json()
            self.current_height = state[0]["pos"]
            return self.current_height
        else:
            print(f"❌ 获取状态失败: {resp.status_code}")
            return None
    
    def move_to_height(self, target_height):
        """移动到指定高度"""
        if not self.is_enabled:
            print("❌ 关节未启用，请先启用")
            return False
            
        # 限制高度范围
        target_height = max(MIN_HEIGHT, min(MAX_HEIGHT, target_height))
        
        print(f"🚀 移动至高度: {target_height} mm...")
        move_command = {
            "pos": [target_height, 0, 0, 0],  # 仅控制升降（关节1），其余维持0
            "vel": 100,
            "acc": 100
        }
        resp = requests.post(MOVETO_URL, json=move_command)
        if resp.status_code == 200:
            self.current_height = target_height
            print(f"✅ 已移动到 {target_height} mm")
            return True
        else:
            print(f"❌ 运动失败: {resp.status_code}")
            return False
    
    def move_up(self):
        """上升"""
        new_height = self.current_height + LIFT_STEP
        print(f"当前高度: {self.current_height} mm, 目标高度: {new_height} mm")
        if new_height <= MAX_HEIGHT:
            self.move_to_height(new_height)
        else:
            print(f"⚠️ 已达到最大高度 {MAX_HEIGHT} mm")
    
    def move_down(self):
        """下降"""
        new_height = self.current_height - LIFT_STEP
        if new_height >= MIN_HEIGHT:
            self.move_to_height(new_height)
        else:
            print(f"⚠️ 已达到最小高度 {MIN_HEIGHT} mm")

    def vrun(self, y):
        v = 0
        r = 0
        vrun = 0.5
        if y == 0:
            return
        if y > 0:
            v=vrun
        elif y < 0: 
            v=-vrun
        param_str = f"angular_velocity={r}&linear_velocity={v}"
        self.sendRequest(param_str)
    
    def rrun(self, x):
        v = 0
        r = 0
        vrun = 0.5
        rrun = 0.3
        if x == 0:
            return
        if x > 0:
            r = -rrun
        elif x < 0:
            r = rrun
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
    
    def normalize_angle(self, angle):
        """将角度规范到[-pi, pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    async def process_data_async(self):
        """异步处理数据循环 - 只负责数据分发"""
        while self.running:
            transforms, buttons = self.oculus_reader.get_transformations_and_buttons()
            # 更新transforms数据 - 只有在未处理且启用时才更新
            with self._transforms_lock:
                if not self._transforms_ready and self._transforms_enabled:  # 只有在未处理且启用时才更新数据
                    self._latest_transforms = transforms
                    if transforms!= None and 'r' in transforms:
                        self._transforms_ready = True
                
            # 更新buttons数据
            with self._buttons_lock:
                self._latest_buttons = buttons
                self._buttons_ready = True

            with self._other_lock:
                self._latest_other = buttons
                self._other_ready = True

            await asyncio.sleep(self.interval)

    def run(self):
        """创建并运行事件循环"""
        # 启动两个处理线程
        self.transforms_thread.start()
        self.buttons_thread.start()
        self.otherButton_thread.start()
        
        # 创建并运行主事件循环
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.process_data_async())
        except Exception as e:
            print(f"事件循环异常: {e}")
        finally:
            self.loop.close()
    
    def stop(self):
        """停止所有线程"""
        self.running = False
        if hasattr(self, 'transforms_thread'):
            self.transforms_thread.join()
        if hasattr(self, 'buttons_thread'):
            self.buttons_thread.join()
        if hasattr(self, 'otherButton_thread'):
            self.otherButton_thread.join()

class OculusReader:
    def __init__(self,
            ip_address=None,
            port = 5555,
            APK_name='com.rail.oculus.teleop',
            print_FPS=False,
            run=True
        ):
        self.running = False
        self.last_transforms = {}
        self.last_buttons = {}
        self._lock = threading.Lock()
        self.tag = 'wE9ryARX'

        self.connection = None
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
        self.device.shell('am start -n "com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity" -a android.intent.action.MAIN -c android.intent.category.LAUNCHER')
        # self.device.shell('am startservice -n "com.rail.oculus.teleop/com.rail.oculus.teleop.MainService" -a android.intent.action.MAIN -c android.intent.category.DEFAULT')
        self.thread = threading.Thread(target=self.device.shell, args=("logcat -T 0", self.read_logcat_by_line))
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'connection') and self.connection:
            self.connection.close()
        if hasattr(self, 'thread'):
            self.thread.join()

    def get_network_device(self, client, retry=0):
        try:
            client.remote_connect(self.ip_address, self.port)
        except RuntimeError:
            os.system('adb devices')
            client.remote_connect(self.ip_address, self.port)
        device = client.device(self.ip_address + ':' + str(self.port))

        if device is None:
            if retry==1:
                os.system('adb tcpip ' + str(self.port))
            if retry==2:
                eprint('Make sure that device is running and is available at the IP address specified as the OculusReader argument `ip_address`.')
                eprint('Currently provided IP address:', self.ip_address)
                eprint('Run `adb shell ip route` to verify the IP address.')
                exit(1)
            else:
                self.get_network_device(client=client, retry=retry+1)
        return device

    def get_usb_device(self, client):
        try:
            devices = client.devices()
        except RuntimeError:
            os.system('adb devices')
            devices = client.devices()
        for device in devices:
            if device.serial.count('.') < 3:
                return device
        eprint('Device not found. Make sure that device is running and is connected over USB')
        eprint('Run `adb devices` to verify that the device is visible.')
        exit(1)

    def get_device(self):
        # Default is "127.0.0.1" and 5037
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
                    APK_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'APK', 'teleop-debug.apk')
                success = self.device.install(APK_path, test=True, reinstall=reinstall)
                installed = self.device.is_installed(self.APK_name)
                if installed and success:
                    print('APK installed successfully.')
                else:
                    eprint('APK install failed.')
            elif verbose:
                print('APK is already installed.')
        except RuntimeError:
            eprint('Device is visible but could not be accessed.')
            eprint('Run `adb devices` to verify that the device is visible and accessible.')
            eprint('If you see "no permissions" next to the device serial, please put on the Oculus Quest and allow the access.')
            exit(1)

    def uninstall(self, verbose=True):
        try:
            installed = self.device.is_installed(self.APK_name)
            if installed:
                success = self.device.uninstall(self.APK_name)
                installed = self.device.is_installed(self.APK_name)
                if not installed and success:
                    print('APK uninstall finished.')
                    print('Please verify if the app disappeared from the list as described in "UNINSTALL.md".')
                    print('For the resolution of this issue, please follow https://github.com/Swind/pure-python-adb/issues/71.')
                else:
                    eprint('APK uninstall failed')
            elif verbose:
                print('APK is not installed.')
        except RuntimeError:
            eprint('Device is visible but could not be accessed.')
            eprint('Run `adb devices` to verify that the device is visible and accessible.')
            eprint('If you see "no permissions" next to the device serial, please put on the Oculus Quest and allow the access.')
            exit(1)

    @staticmethod
    def process_data(string):
        try:
            transforms_string, buttons_string = string.split('&')
        except ValueError:
            return None, None
        split_transform_strings = transforms_string.split('|')
        transforms = {}
        for pair_string in split_transform_strings:
            transform = np.empty((4,4))
            pair = pair_string.split(':')
            if len(pair) != 2:
                continue
            left_right_char = pair[0] # is r or l
            transform_string = pair[1]
            values = transform_string.split(' ')
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
        output = ''
        if self.tag in line:
            try:
                output += line.split(self.tag + ': ')[1]
            except ValueError:
                pass
        return output

    def get_transformations_and_buttons(self):
        with self._lock:
            return self.last_transforms, self.last_buttons

    def read_logcat_by_line(self, connection):
        self.connection = connection
        file_obj = connection.socket.makefile()
        while self.running:
            try:
                line = file_obj.readline().strip()
                if not line:
                    if not self.running:
                        break
                    else:
                        continue
                data = self.extract_data(line)
                if data:
                    transforms, buttons = OculusReader.process_data(data)
                    with self._lock:
                        self.last_transforms, self.last_buttons = transforms, buttons
                    if self.print_FPS:
                        self.fps_counter.getAndPrintFPS()
            except (UnicodeDecodeError, IOError):
                pass
        file_obj.close()
        connection.close()

def main():
    rclpy.init()
    # robot = jkrc.RC("192.168.10.90")#返回一个机器人对象
    # robot.login()  #登录
    # robot.power_on()
    # robot.enable_robot()
    ros_client = JakaServoClientAsync()
    future = ros_client.enable_servo_mode()
    rclpy.spin_until_future_complete(ros_client, future)

    response = future.result()
    ros_client.get_logger().info('servo mode enable %s' % (response.message))

    oculus_reader = OculusReader()
    data_thread = OculusDataRosThread(oculus_reader, ros_client)
    data_thread.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nProgram interrupted. Stopping...")
        data_thread.stop()  # 这会停止所有子线程
        data_thread.join()
        oculus_reader.stop()
        ros_client.destroy_node()
        rclpy.shutdown()
        print("Stopped.")


if __name__ == '__main__':
    main()
 