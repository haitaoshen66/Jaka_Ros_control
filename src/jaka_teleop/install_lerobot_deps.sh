#!/bin/bash
# install_lerobot_deps.sh
# 安装Lerobot数据记录所需的Python依赖

echo "🔧 安装Lerobot数据记录依赖..."

# 安装Python包
pip3 install h5py opencv-python numpy pathlib

# 检查ROS2包
echo "📦 检查ROS2依赖..."
ros2 pkg list | grep cv_bridge || echo "⚠️ 需要安装 cv_bridge: sudo apt install ros-humble-cv-bridge"
ros2 pkg list | grep sensor_msgs || echo "⚠️ 需要安装 sensor_msgs: sudo apt install ros-humble-sensor-msgs"

echo "✅ 依赖安装完成！"

echo ""
echo "📝 使用说明："
echo "1. 构建包: colcon build --packages-select jaka_teleop"
echo "2. 启动系统: ros2 launch jaka_teleop teleop_launch.py"
echo "3. 开始录制: ros2 service call /lerobot/toggle_recording std_srvs/srv/SetBool '{data: true}'"
echo "4. 停止录制: ros2 service call /lerobot/toggle_recording std_srvs/srv/SetBool '{data: false}'"
echo "5. 或使用VR手柄Y键控制录制"
