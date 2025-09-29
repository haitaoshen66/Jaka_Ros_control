#!/bin/bash
# install_camera_deps.sh
# 安装摄像头相关依赖

echo "📦 开始安装摄像头依赖..."

# 系统依赖
echo "🔧 安装系统依赖..."
sudo apt update
sudo apt install -y \
    python3-opencv \
    python3-yaml \
    v4l-utils \
    cheese

# Python依赖
echo "🐍 安装Python依赖..."
pip3 install --user \
    opencv-python \
    PyYAML \
    numpy

# 可选：RealSense支持
# read -p "❓ 是否安装RealSense支持? (y/N): " install_realsense
# if [[ $install_realsense =~ ^[Yy]$ ]]; then
#     echo "📷 安装RealSense依赖..."
    
#     # 添加Intel RealSense仓库
#     sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
#     sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
    
#     # 安装RealSense库
#     sudo apt install -y \
#         librealsense2-dkms \
#         librealsense2-utils \
#         librealsense2-dev \
#         librealsense2-dbg
    
#     # 安装Python绑定
#     pip3 install --user pyrealsense2
    
#     echo "✅ RealSense安装完成"
# fi

# # 设置摄像头权限
# echo "🔐 配置摄像头权限..."
# sudo usermod -a -G video $USER
# sudo usermod -a -G dialout $USER

# # 创建udev规则（可选，用于RealSense）
# if [[ $install_realsense =~ ^[Yy]$ ]]; then
#     echo "📝 配置RealSense udev规则..."
#     sudo cp /etc/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/ 2>/dev/null || true
#     sudo udevadm control --reload-rules && sudo udevadm trigger
# fi

echo ""
echo "✅ 摄像头依赖安装完成！"
echo ""
echo "📋 使用说明："
echo "1. 重新登录或重启以应用用户组更改"
echo "2. 使用 'v4l2-ctl --list-devices' 查看可用摄像头"
echo "3. 使用 'cheese' 测试摄像头功能"
echo "4. 编辑 config/camera_config.yaml 配置摄像头"
echo ""

# 测试摄像头
echo "🧪 检测摄像头设备..."
if command -v v4l2-ctl &> /dev/null; then
    echo "可用摄像头设备："
    v4l2-ctl --list-devices
else
    echo "请安装 v4l-utils 以查看摄像头设备"
fi

echo ""
echo "🚀 现在可以运行 camera_node 了！"
echo "   ros2 run jaka_teleop camera_node"
