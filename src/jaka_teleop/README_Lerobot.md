# JAKA Teleop Lerobot数据记录模块

本模块为JAKA遥操作系统添加了Lerobot格式的数据记录功能，可以记录VR遥操作过程中的所有数据用于机器学习训练。

## 📦 新增模块

### 1. `lerobot_recorder_node.py`
- **功能**: 记录所有传感器数据和控制指令
- **数据格式**: HDF5 (Lerobot兼容)
- **记录内容**:
  - VR手柄位置、姿态、按钮状态
  - 机器人关节位置、速度
  - 机器人末端位姿
  - 底盘控制指令
  - 摄像头图像（如果有）

### 2. `lerobot_control_node.py`
- **功能**: 提供录制控制接口
- **控制方式**:
  - ROS2服务调用
  - VR手柄Y键开始/停止
- **状态监控**: 实时显示录制状态

### 3. `convert_to_lerobot.py`
- **功能**: 数据格式转换工具
- **用途**: 将录制数据转换为标准Lerobot格式

## 🚀 安装与使用

### 1. 安装依赖
```bash
cd /home/sht/DIJA/jaka_ros2/src/jaka_teleop
chmod +x install_lerobot_deps.sh
./install_lerobot_deps.sh
```

### 2. 构建包
```bash
cd /home/sht/DIJA/jaka_ros2
colcon build --packages-select jaka_teleop
source install/setup.bash
```

### 3. 启动系统
```bash
ros2 launch jaka_teleop teleop_launch.py
```

### 4. 控制录制

#### 方法1: 使用服务调用
```bash
# 开始录制
ros2 service call /lerobot/toggle_recording std_srvs/srv/SetBool "{data: true}"

# 停止录制
ros2 service call /lerobot/toggle_recording std_srvs/srv/SetBool "{data: false}"
```

#### 方法2: 使用VR手柄
- 按下 **Y键** 开始录制
- 再次按下 **Y键** 停止录制

### 5. 查看录制数据
```bash
# 数据保存位置（默认）
ls /tmp/lerobot_datasets/

# 每个episode包含：
# ├── episode_0001_20250912_143022/
# │   ├── data.hdf5        # 原始数据
# │   └── metadata.json    # 元数据
```

### 6. 转换为标准Lerobot格式
```bash
cd /home/sht/DIJA/jaka_ros2/src/jaka_teleop
python3 convert_to_lerobot.py /tmp/lerobot_datasets /tmp/lerobot_converted
```

## 📊 数据结构

### HDF5数据格式
```
data.hdf5
├── action/
│   ├── vr_position      # VR手柄位置 (T, 3)
│   ├── vr_orientation   # VR手柄姿态 (T, 4)
│   └── vr_buttons       # VR按钮状态 (T, N)
└── observation/
    ├── joint_position   # 关节位置 (T, 6)
    ├── joint_velocity   # 关节速度 (T, 6)
    ├── end_effector_position    # 末端位置 (T, 3)
    ├── end_effector_orientation # 末端姿态 (T, 3)
    └── camera_xxx       # 摄像头图像 (T, H, W, C)
```

### 元数据格式
```json
{
  "episode_name": "jaka_teleop_0001_20250912_143022",
  "num_frames": 1500,
  "fps": 30.0,
  "created_at": "2025-09-12T14:30:22",
  "data_keys": {
    "action": ["vr_position", "vr_orientation", "vr_buttons"],
    "observation": ["joint_position", "joint_velocity", ...]
  }
}
```

## ⚙️ 配置参数

可在启动文件中修改以下参数：

```python
Node(
    package='jaka_teleop',
    executable='lerobot_recorder_node',
    parameters=[
        {'output_dir': '/your/custom/path'},  # 数据保存路径
        {'episode_name': 'custom_name'},      # Episode名称前缀
        {'fps': 30.0}                         # 记录频率
    ]
)
```

## 🔧 添加摄像头支持

如果系统中有摄像头，可以通过以下方式添加：

1. **在recorder_node中取消注释摄像头订阅**:
```python
self.camera_sub = self.create_subscription(
    Image, "/camera/image_raw", self.camera_callback, 10
)
```

2. **添加多个摄像头**:
```python
# 正面摄像头
self.front_camera_sub = self.create_subscription(
    Image, "/camera/front/image_raw", 
    lambda msg: self.camera_callback(msg, "front"), 10
)

# 侧面摄像头
self.side_camera_sub = self.create_subscription(
    Image, "/camera/side/image_raw", 
    lambda msg: self.camera_callback(msg, "side"), 10
)
```

## 🐛 故障排除

### 1. 导入错误
```bash
# 安装缺失的Python包
pip3 install h5py opencv-python numpy

# 安装ROS2包
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs
```

### 2. 权限问题
```bash
# 创建数据目录并设置权限
sudo mkdir -p /tmp/lerobot_datasets
sudo chown $USER:$USER /tmp/lerobot_datasets
```

### 3. 没有录制到数据
- 检查VR设备连接
- 确保所有节点正常运行
- 查看日志输出确认数据流

## 📈 使用建议

1. **录制前准备**:
   - 确保VR设备校准正确
   - 检查机器人初始位置
   - 确认所有传感器工作正常

2. **录制过程**:
   - 执行多样化的动作序列
   - 包含正确和错误的操作示例
   - 记录不同环境条件下的数据

3. **数据质量**:
   - 每个episode建议20-60秒
   - 收集100+个高质量episodes
   - 定期检查数据完整性

## 📚 相关资源

- [Lerobot项目](https://github.com/huggingface/lerobot)
- [ROS2文档](https://docs.ros.org/en/humble/)
- [HDF5格式说明](https://docs.h5py.org/)

---

如有问题，请查看日志输出或联系开发团队。
