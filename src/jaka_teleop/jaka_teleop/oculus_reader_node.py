#!/usr/bin/env python3
# oculus_reader_node.py
# 作用：读取Oculus数据，并发布至两个Topic：/oculus/buttons 和 /oculus/transforms

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
import threading
import time
from ppadb.client import Client as AdbClient
import threading
import os
import sys

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

class OculusReader:
    def __init__(self,
            ip_address=None,
            port = 5555,
            APK_name='com.rail.oculus.teleop',
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
            except UnicodeDecodeError:
                pass
        file_obj.close()
        connection.close()



class OculusReaderNode(Node):
    def __init__(self):
        super().__init__("oculus_reader_node")
        self.publisher_buttons = self.create_publisher(String, "/oculus/buttons", 10)
        self.publisher_transforms = self.create_publisher(
            String, "/oculus/transforms", 10
        )
        # self.reseting_sub = self.create_subscription(Bool, "/teleop/reset_pose", self.reset_callback, 1)
        self.oculus_reader = OculusReader()
        self.running = True
        # print(self.running)
        self.reset_mode = False
        # 后台线程定时读取数据
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    # def reset_callback(self, msg):
    #     # self.get_logger().info("reset_callback called")
    #     if self.reset_mode == msg.data:
    #         return
    #     # self.get_logger().info(f"reset_callback: reset_mode = {self.reset_mode} -> {msg.data}")
    #     self.reset_mode = msg.data

    def loop(self):
        rate = 20.0  # Hz，提升频率更灵敏
        while self.running:
            transforms, buttons = self.oculus_reader.get_transformations_and_buttons()
            # print(transforms)
            # 将numpy matrix转成list（防止JSON无法序列化）
            transforms_serializable = {}
            if transforms:
                for key, mat in transforms.items():
                    if mat is not None and mat.shape == (4, 4):
                        transforms_serializable[key] = mat.flatten().tolist()

            msg_buttons = String()
            msg_transforms = String()
            try:
                msg_buttons.data = json.dumps(buttons, ensure_ascii=False)
                msg_transforms.data = json.dumps(transforms_serializable)
            except Exception as e:
                self.get_logger().warn(f"序列化失败: {e}")
                continue
            # print("msg_transforms", msg_transforms.data)
            self.publisher_buttons.publish(msg_buttons)
            # if self.reset_mode:
            #     # 如果处于重置模式，跳过数据读取
            #     time.sleep(1.0 / rate)
            #     continue    
            # else:
            self.publisher_transforms.publish(msg_transforms)
            time.sleep(1.0 / rate)

    def destroy_node(self):
        self.running = False
        self.thread.join()
        self.oculus_reader.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OculusReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
