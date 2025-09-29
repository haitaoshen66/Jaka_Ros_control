#!/usr/bin/env python3
# oculus_reader.py
# 提供 OculusReader 类与 parse_buttons 工具函数

import numpy as np
import threading
import os
from ppadb.client import Client as AdbClient
import json

TAG = "wE9ryARX"  # 日志Tag


def parse_buttons(text):
    split_text = text.split(",")
    buttons = {}
    if "R" in split_text:
        split_text.remove("R")
        buttons.update(
            {
                "A": False,
                "B": False,
                "X": False,
                "Y": False,
                "RThU": False,
                "RJ": False,
                "RG": False,
                "RTr": False,
            }
        )
    if "L" in split_text:
        split_text.remove("L")
        buttons.update(
            {
                "A": False,
                "B": False,
                "X": False,
                "Y": False,
                "LThU": False,
                "LJ": False,
                "LG": False,
                "LTr": False,
            }
        )
    for key in buttons.keys():
        if key in split_text:
            buttons[key] = True
            split_text.remove(key)
    for elem in split_text:
        split_elem = elem.split(" ")
        if len(split_elem) < 2:
            continue
        key = split_elem[0]
        value = tuple([float(x) for x in split_elem[1:]])
        buttons[key] = value
    return buttons


class OculusReader:
    def __init__(
        self, ip_address=None, port=5555, APK_name="com.rail.oculus.teleop", run=True
    ):
        self.ip_address = ip_address
        self.port = port
        self.APK_name = APK_name
        self.running = False
        self.last_transforms = {}
        self.last_buttons = {}
        self._lock = threading.Lock()
        self.device = self.get_device()
        self.install()
        if run:
            self.run()

    def get_device(self):
        client = AdbClient(host="127.0.0.1", port=5037)
        if self.ip_address:
            try:
                client.remote_connect(self.ip_address, self.port)
            except RuntimeError:
                os.system("adb devices")
                client.remote_connect(self.ip_address, self.port)
            return client.device(f"{self.ip_address}:{self.port}")
        else:
            return client.devices()[0]  # 默认第一个USB设备

    def install(self):
        if not self.device.is_installed(self.APK_name):
            apk_path = os.path.join(
                os.path.dirname(__file__), "APK", "teleop-debug.apk"
            )
            self.device.install(apk_path, reinstall=True)

    def run(self):
        self.running = True
        self.device.shell(f'am start -n "{self.APK_name}/{self.APK_name}.MainActivity"')
        self.thread = threading.Thread(target=self._read_logcat_loop)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join()

    def _read_logcat_loop(self):
        connection = self.device.shell("logcat -T 0")
        file_obj = connection.socket.makefile()
        while self.running:
            try:
                line = file_obj.readline().strip()
                if TAG not in line:
                    continue
                payload = line.split(TAG + ": ")[1]
                transforms, buttons = self.process_data(payload)
                with self._lock:
                    self.last_transforms = transforms
                    self.last_buttons = buttons
            except Exception:
                continue
        file_obj.close()

    def get_transformations_and_buttons(self):
        with self._lock:
            return self.last_transforms.copy(), self.last_buttons.copy()

    @staticmethod
    def process_data(string):
        try:
            transforms_string, buttons_string = string.split("&")
        except ValueError:
            return {}, {}
        split_transform_strings = transforms_string.split("|")
        transforms = {}
        for pair_string in split_transform_strings:
            transform = np.zeros((4, 4))
            pair = pair_string.split(":")
            if len(pair) != 2:
                continue
            key, values = pair[0], pair[1].split(" ")
            try:
                float_values = [float(v) for v in values if v]
                if len(float_values) == 16:
                    transform = np.array(float_values).reshape(4, 4)
                    transforms[key] = transform
            except:
                continue
        buttons = parse_buttons(buttons_string)
        return transforms, buttons
