#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VR手柄数据WebSocket客户端（简化版）
用于接收socket_reader服务器(right_hand_total)数据并打印
"""

import asyncio

import json
import time
import logging
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from jaka_msgs.srv import GetIK
import sys
print(sys.executable)
import websockets

# 设置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("VRClient")

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GetIK, 'jaka_rviz_driver/get_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetIK.Request()

    def send_request(self,cartesian_pose):
        self.req.cartesian_pose = cartesian_pose
        return self.cli.call_async(self.req)

class VRDataClient:
    def __init__(self, server_url: str):
        """初始化VR数据客户端
        Args:
            server_url: WebSocket服务器URL，格式如 "ws://192.168.1.100:8765"
        """
        self.server_url = server_url
        self.websocket = None
        self.connected = False
        self.last_data = None
        self.data_received_count = 0
        self.last_ping_time = 0
        self.ping_interval = 5  # 每5秒发送一次ping
        self.connection_retry_delay = 2  # 重连延迟(秒)
        self.max_retries = 5  # 最大重试次数
        # ros client端
        self.ros_client = None

    def init_ros_client(self):
        if self.ros_client is None:
            try:
                self.ros_client = MinimalClientAsync()
                print("ROS2客户端初始化成功")
            except Exception as e:
                print(f"ROS2客户端初始化失败: {e}")
                self.ros_client = None

    async def connect(self) -> bool:
        retry_count = 0
        while retry_count < self.max_retries:
            try:
                logger.info(f"正在连接到服务器: {self.server_url}")
                self.websocket = await websockets.connect(
                    self.server_url,
                    ping_interval=None,  # 我们自己管理ping
                    max_size=10**6,  # 1MB最大消息大小
                )
                self.connected = True
                logger.info("连接成功!")
                self.init_ros_client()
                return True
            except (websockets.exceptions.ConnectionClosed, 
                    websockets.exceptions.WebSocketException,
                    ConnectionRefusedError) as e:
                retry_count += 1
                logger.error(f"连接失败 ({retry_count}/{self.max_retries}): {e}")
                if retry_count < self.max_retries:
                    logger.info(f"{self.connection_retry_delay}秒后重试...")
                    await asyncio.sleep(self.connection_retry_delay)
                else:
                    logger.error(f"达到最大重试次数，无法连接到服务器")
                    return False
            except Exception as e:
                logger.error(f"连接出现未知错误: {e}")
                return False
        return False

    async def disconnect(self):
        if self.websocket and self.connected:
            try:
                await self.websocket.close()
                logger.info("已断开连接")
            except Exception as e:
                logger.error(f"断开连接时出错: {e}")
            finally:
                self.connected = False
                self.websocket = None

    async def send_ping(self):
        if not self.connected or not self.websocket:
            return
        try:
            ping_msg = {
                "type": "ping",
                "timestamp": time.time()
            }
            await self.websocket.send(json.dumps(ping_msg))
            self.last_ping_time = time.time()
            logger.debug("Ping已发送")
        except Exception as e:
            logger.error(f"发送ping失败: {e}")
            self.connected = False

    def pretty_print_data(self, data):
        # 适配socket_reader.py服务器的数据格式
        if 'right_hand_total' in data:
            print("\n" + "="*50)
            print(f"帧序号: {data.get('frame_count', 'N/A')}")
            print(f"时间戳: {data.get('timestamp', 'N/A')}")
            print(f"状态: {data.get('status', 'N/A')}")
            delta = data.get('right_hand_total')
            if delta:
                print(f"右手相对位移和旋转变化量: droll={delta[0]:.4f}, dpitch={delta[1]:.4f}, dyaw={delta[2]:.4f}, "
                      f"dx={delta[3]:.2f}, dy={delta[4]:.2f}, dz={delta[5]:.2f}")
            else:
                print("未收到右手数据")
        elif 'type' in data and data['type'] == 'connection':
            print("\n" + "="*50)
            print(f"连接状态: {data.get('status', 'unknown')}")
            print(f"消息: {data.get('message', '')}")
            server_info = data.get('server_info', {})
            if server_info:
                print("\n服务器信息:")
                print(f"  数据格式: {server_info.get('data_format', 'unknown')}")
                print(f"  更新频率: {server_info.get('update_rate', 'unknown')}")
                print(f"  数据结构: {server_info.get('data_structure', 'unknown')}")
        elif 'type' in data and data['type'] == 'pong':
            print(f"\n服务器响应 Pong - 延迟: {(time.time() - data.get('timestamp', 0))*1000:.1f}ms")
            print(f"数据传输率: {data.get('data_rate', 0):.2f} Hz")
        else:
            print("收到未知数据:", data)

    async def call_ros_service(self, cartesian_pose):
        """调用ROS2服务"""
        if self.ros_client is None:
            logger.error("ROS2客户端未初始化")
            return 
        try:
            future = self.ros_client.send_request(cartesian_pose)
            rclpy.spin_until_future_complete(self.ros_client, future)
            response = future.result()
            print(f"ROS2服务调用结果: {response.message}")
        except Exception as e:
            print(f"ROS2服务调用失败: {e}")

    async def receive_data(self):
        if not self.connected or not self.websocket:
            return
        try:
            raw_data = await self.websocket.recv()
            try:
                data = json.loads(raw_data)
                self.last_data = data
                self.data_received_count += 1
                self.pretty_print_data(data)
                delta = data.get('right_hand_total')
                # 新增：每2秒执行一次命令
                if delta and isinstance(delta, (list, tuple)) and len(delta) >= 6:
                    now = time.time()
                    if not hasattr(self, "_last_cmd_time") or now - self._last_cmd_time > 2:
                        self._last_cmd_time = now
                        cartesian_pose = [float(f"{v:.4f}") for v in delta[:6]]
                        print(f"调用ROS2服务，cartesian_pose: {cartesian_pose}")
                        # 异步调用ROS2服务
                        # asyncio.create_task(self.call_ros_service(cartesian_pose))

            except json.JSONDecodeError as e:
                logger.error(f"解析JSON失败: {e}")
        except websockets.exceptions.ConnectionClosed:
            logger.warning("连接已关闭")
            self.connected = False
        except Exception as e:
            logger.error(f"接收数据时出错: {e}")
            self.connected = False

    async def run(self):
        if not await self.connect():
            return
        try:
            last_ping_time = time.time()
            while self.connected:
                current_time = time.time()
                if current_time - last_ping_time >= self.ping_interval:
                    await self.send_ping()
                    last_ping_time = current_time
                try:
                    await asyncio.wait_for(self.receive_data(), timeout=1.5)
                except asyncio.TimeoutError:
                    pass
                if not self.connected:
                    logger.info("连接断开，尝试重新连接...")
                    if await self.connect():
                        last_ping_time = time.time()
                    else:
                        break
        except KeyboardInterrupt:
            logger.info("接收到停止信号")
        except Exception as e:
            logger.error(f"客户端错误: {e}")
        finally:
            await self.disconnect()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="VR手柄数据WebSocket客户端（简化版）")
    parser.add_argument("--host", default="10.46.187.70", help="服务器主机名")
    parser.add_argument("--port", type=int, default=8765, help="服务器端口")
    args = parser.parse_args()
    
    # 初始化ROS2
    try:
        rclpy.init(args=None)
        logger.info("ROS2初始化成功")
    except Exception as e:
        logger.error(f"ROS2初始化失败: {e}")
        return
    
    server_url = f"ws://{args.host}:{args.port}"
    client = VRDataClient(server_url)
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(client.run())
    except KeyboardInterrupt:
        logger.info("程序被用户中断")
    except Exception as e:
        logger.error(f"发生错误: {e}")
    finally:
        try:
            rclpy.shutdown()
            logger.info("ROS2已关闭")
        except Exception as e:
            logger.error(f"关闭ROS2时出错: {e}")
        logger.info("程序退出")


if __name__ == "__main__":
    main()