# Camera Node 使用说明

## 概述

`camera_node` 是一个专门的摄像头管理模块，负责统一管理多路摄像头输入，为 `lerobot_recorder_node` 提供标准化的图像数据。

## 功能特性

- 🎥 **多摄像头支持**: 同时管理多个USB、网络或RealSense摄像头
- 🔄 **动态配置**: 运行时添加/移除摄像头
- 📸 **图像处理**: 内置图像增强、调整大小、滤波等功能
- 📡 **ROS2集成**: 标准ROS2图像话题发布
- ⚙️ **灵活配置**: YAML配置文件支持

## 支持的摄像头类型

### 1. USB摄像头
```yaml
main_camera:
  type: usb
  device_id: 0          # /dev/video0
  width: 1280
  height: 720
  fps: 30
```

### 2. 网络摄像头
```yaml
ip_camera:
  type: ip
  url: "http://192.168.1.100:8080/video"
  width: 640
  height: 480
  fps: 20
```

### 3. RealSense摄像头
```yaml
realsense_camera:
  type: realsense
  width: 640
  height: 480
  fps: 30
  enable_depth: true    # 启用深度数据
```

## 配置文件

配置文件位置: `config/camera_config.yaml`

### 完整配置示例
```yaml
cameras:
  # 主摄像头
  main_camera:
    type: usb
    device_id: 0
    width: 1280
    height: 720
    fps: 30
    resize: [640, 480]  # 输出尺寸调整
    enhance: true       # 图像增强
    contrast: 1.2       # 对比度
    brightness: 10      # 亮度
    blur: false         # 高斯模糊
    
  # 辅助摄像头
  auxiliary_camera:
    type: usb
    device_id: 1
    width: 640
    height: 480
    fps: 15

global:
  publish_rate: 30.0    # 发布频率(Hz)
  image_quality: 80     # JPEG压缩质量
```

## 使用方法

### 1. 启动camera_node

#### 方式一：单独启动
```bash
ros2 run jaka_teleop camera_node --ros-args -p config_file:=config/camera_config.yaml
```

#### 方式二：通过launch文件启动
```bash
ros2 launch jaka_teleop teleop_launch.py
```

### 2. 动态控制摄像头

使用 `camera_controller.py` 工具：

```bash
# 列出当前摄像头
python3 camera_controller.py list

# 查看摄像头状态
python3 camera_controller.py status

# 添加新摄像头
python3 camera_controller.py add --name new_camera --device-id 2

# 移除摄像头
python3 camera_controller.py remove --name new_camera
```

### 3. 查看图像话题

```bash
# 列出所有摄像头话题
ros2 topic list | grep camera

# 查看特定摄像头的图像
ros2 run rqt_image_view rqt_image_view /camera/main_camera/image_raw
```

## ROS2话题接口

### 发布的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/camera/{camera_name}/image_raw` | sensor_msgs/Image | 原始图像数据 |
| `/camera/{camera_name}/camera_info` | sensor_msgs/CameraInfo | 摄像头标定信息 |
| `/camera/status` | std_msgs/String | 摄像头状态(JSON格式) |

### 订阅的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/camera/control` | std_msgs/String | 控制命令(JSON格式) |

### 控制命令格式

#### 添加摄像头
```json
{
  "action": "add_camera",
  "name": "camera_name",
  "config": {
    "type": "usb",
    "device_id": 0,
    "width": 640,
    "height": 480,
    "fps": 30
  }
}
```

#### 移除摄像头
```json
{
  "action": "remove_camera",
  "name": "camera_name"
}
```

#### 列出摄像头
```json
{
  "action": "list_cameras"
}
```

### 状态消息格式
```json
{
  "timestamp": 1642678901.23,
  "cameras": {
    "main_camera": {
      "active": true,
      "last_update": 1642678901.22
    },
    "auxiliary_camera": {
      "active": false,
      "last_update": 0
    }
  }
}
```

## 与Lerobot记录器的集成

`camera_node` 与 `lerobot_recorder_node` 无缝集成：

1. **自动发现**: recorder会自动订阅所有camera_node发布的图像话题
2. **动态更新**: 新增摄像头时recorder会自动开始记录
3. **数据同步**: 所有图像数据都会包含时间戳信息

## 故障排除

### 常见问题

1. **摄像头无法打开**
   ```bash
   # 检查设备权限
   ls -la /dev/video*
   
   # 添加用户到video组
   sudo usermod -a -G video $USER
   ```

2. **RealSense摄像头不工作**
   ```bash
   # 安装RealSense库
   pip install pyrealsense2
   
   # 检查RealSense设备
   rs-enumerate-devices
   ```

3. **图像话题没有数据**
   ```bash
   # 检查节点状态
   ros2 node info /camera_node
   
   # 查看话题频率
   ros2 topic hz /camera/main_camera/image_raw
   ```

### 调试命令

```bash
# 查看camera_node日志
ros2 run jaka_teleop camera_node --ros-args --log-level DEBUG

# 监控摄像头状态
ros2 topic echo /camera/status

# 检查图像质量
ros2 run image_view image_view image:=/camera/main_camera/image_raw
```

## 性能优化

### 降低CPU使用率
- 减少发布频率：`publish_rate: 15.0`
- 降低图像分辨率：`resize: [320, 240]`
- 减少摄像头数量

### 减少网络带宽
- 降低JPEG质量：`image_quality: 60`
- 使用图像压缩传输

## 扩展开发

### 添加新的摄像头类型

1. 在 `CameraManager.add_camera()` 中添加新类型处理
2. 实现对应的 `_setup_xxx()` 方法
3. 在配置文件中添加相应参数

### 自定义图像处理

在 `_process_frame()` 方法中添加：
- 图像滤波
- 色彩空间转换
- 特征检测
- 图像标注

## 许可证

Apache License 2.0
