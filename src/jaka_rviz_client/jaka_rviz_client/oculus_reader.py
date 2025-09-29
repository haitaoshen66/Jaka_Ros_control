#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from FPS_counter import FPSCounter
from buttons_parser import parse_buttons
import numpy as np
import threading
import time
import os
from ppadb.client import Client as AdbClient
import sys
import csv
import math

def eprint(*args, **kwargs):
    RED = "\033[1;31m"  
    sys.stderr.write(RED)
    print(*args, file=sys.stderr, **kwargs)
    RESET = "\033[0;0m"
    sys.stderr.write(RESET)

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

def calMatrix(matrix,init_matrix):
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

    return robot_roll,robot_pitch,robot_yaw

def normalize_angle(angle):
    """将角度规范到[-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def main():
    oculus_reader = OculusReader()
    prev_right = None
    right_hand_total = np.array([0,0,0,0.154,-0.396,0.211])
    frame_count = 0
    init_matrix = None
    try:
        while True:
            time.sleep(0.02)
            # print(oculus_reader.get_transformations_and_buttons())
            transforms, buttons = (
                oculus_reader.get_transformations_and_buttons()
            )
            delta = np.array([0,0,0,0,0,0])
            if transforms and 'r' in transforms:
                transform_matrix = transforms['r']
                if transform_matrix is not None:
                    # 提取位置 [x, y, z]
                    position = transform_matrix[:3, 3]
                    # 提取旋转矩阵
                    rotation_matrix = transform_matrix[:3, :3]
                    # 记录当前帧
                    if prev_right is None:  # 首帧
                        prev_right = (position.copy(), rotation_matrix.copy(), -0.785, -1.57, -2.36)
                        init_matrix = rotation_matrix.copy()
                        delta = np.array([0,0,0,0,0,0])
                        droll, dpitch, dyaw = 0.0, 0.0, 0.0
                    else:
                        prev_pos, prev_rot, prev_roll, prev_pitch, prev_yaw = prev_right
                        # 位置变化量
                        dx, dy, dz = position - prev_pos
                    
                        droll, dpitch, dyaw = calMatrix(transform_matrix, init_matrix)
                        # droll = normalize_angle(droll)
                        # dpitch = normalize_angle(dpitch)
                        # dyaw = normalize_angle(dyaw)
                        cx = -dz
                        cy = -dx
                        cz = dy
                        # fx = -cz
                        # fy = cx
                        # fz = -cy
                        delta = np.array([normalize_angle(droll-prev_roll), normalize_angle(dpitch-prev_pitch), normalize_angle(dyaw-prev_yaw), cx*0.5*1000, cy*0.5*1000, cz*0.5*1000])
                        prev_right = (position.copy(), init_matrix, droll, dpitch, dyaw)
                    # 累加
                    right_hand_total += delta
                    right_hand_total[0] = droll
                    right_hand_total[1] = dpitch
                    right_hand_total[2] = dyaw
            
            # 追加写入csv，不再覆盖
            xyz = delta.tolist()
            current_xyz = [0, 0, 0] + xyz[3:]
            csv_path = os.path.join(os.path.dirname(__file__), 'right_hand_xyz.csv')
            with open(csv_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([frame_count] + list(current_xyz))
            frame_count += 1
            for angle_idx in range(3):  # 处理roll, pitch, yaw三个角度
                angle_change = xyz[angle_idx]
                if abs(angle_change) > 0.2:
                    n = int(np.ceil(abs(angle_change) / 0.2))
                    step_size = angle_change / n
                    
                    for i in range(n):
                        current_xyz = [0.0, 0.0, 0.0] + [0.0, 0.0, 0.0]  # 前三个角度初始化为0
                        current_xyz[angle_idx] = step_size  # 只设置当前角度
                        with open(csv_path, 'a', newline='') as csvfile:
                            writer = csv.writer(csvfile)
                            writer.writerow([frame_count] + list(current_xyz))
                        frame_count += 1
                else:
                    current_xyz = [0.0, 0.0, 0.0] + [0.0, 0.0, 0.0]  # 前三个角度初始化为0
                    current_xyz[angle_idx] = angle_change  # 设置当前角度  
                    with open(csv_path, 'a', newline='') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow([frame_count] + list(current_xyz))
                    frame_count += 1
    except KeyboardInterrupt:
        print("\n程序已停止（Ctrl+C）")
        oculus_reader.stop()
    except Exception as e:
        print(f"\n程序异常终止：{e}")
        oculus_reader.stop()
        # 可选：打印详细traceback
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    csv_path = os.path.join(os.path.dirname(__file__), 'right_hand_xyz.csv')
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['frame', 'roll', 'pitch', 'yaw', 'x', 'y', 'z'])
    main()
 