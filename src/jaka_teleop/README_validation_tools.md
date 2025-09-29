# HDF5数据验证工具集

这是一套用于验证JAKA机器人遥操作数据集的工具集，可以检查HDF5文件的结构和内容，并创建合并的验证视频。

## 工具概览

### 1. quick_hdf5_check.py
**快速结构检查工具**
- 🔍 检查HDF5文件结构
- 📊 显示数据集大小和形状
- ⚠️  识别数据问题
- 📋 提供详细统计信息

### 2. validate_hdf5_data.py
**视频验证工具**
- 🎥 创建多相机合并视频
- 📈 叠加轨迹数据
- 🎬 支持多种布局
- 💾 高质量视频输出

### 3. batch_validate_datasets.py
**批量处理工具**
- 📁 处理整个数据集目录
- 🔄 批量检查和视频生成
- 📊 统计处理结果
- ⚙️  可配置输出选项

### 4. validate_tools.sh
**一键启动脚本**
- 🖥️  交互式菜单界面
- 🚀 快速访问所有功能
- 📂 自动文件选择
- 🎯 预设常用路径

## 快速开始

### 方法1：使用一键脚本（推荐）

```bash
cd /home/sht/DIJA/jaka_ros2/src/jaka_teleop
./validate_tools.sh
```

这会打开一个交互式菜单，您可以选择：
1. 检查单个HDF5文件
2. 创建单个验证视频
3. 批量检查所有数据集
4. 批量创建所有验证视频
5. 完整验证（检查+视频）
6. 清理验证视频目录

### 方法2：直接使用Python脚本

#### 检查单个文件
```bash
python3 quick_hdf5_check.py /path/to/data.hdf5
```

#### 创建单个验证视频
```bash
python3 validate_hdf5_data.py /path/to/data.hdf5 /path/to/output.mp4
```

#### 批量处理数据集
```bash
# 只检查，不创建视频
python3 batch_validate_datasets.py /home/sht/DIJA/lerobot_datasets --check-only

# 只创建视频，不检查
python3 batch_validate_datasets.py /home/sht/DIJA/lerobot_datasets --video-only --output-dir /path/to/videos

# 完整处理
python3 batch_validate_datasets.py /home/sht/DIJA/lerobot_datasets --output-dir /path/to/videos
```

## 预期的HDF5数据格式

工具期望以下数据结构：

```
数据集根目录/
├── observation/
│   ├── camera1      # 第一个相机的图像数据 (N, H, W, 3)
│   ├── camera2      # 第二个相机的图像数据 (N, H, W, 3)
│   └── ...          # 更多相机
└── trajectory/
    ├── position     # 位置数据 (N, 7) - [x,y,z,qx,qy,qz,gripper]
    └── joint        # 关节角度数据 (N, 6) - 6个关节角度
```

## 生成的视频特性

### 双相机布局
- 📱 side_by_side: 左右并排显示
- 📺 top_bottom: 上下排列显示
- 🖼️  picture_in_picture: 画中画显示

### 多相机布局
- 🏗️  grid: 网格布局（2x2, 2x3等）
- 📏 适应性调整尺寸

### 轨迹数据叠加
- 📍 位置：X, Y, Z坐标和夹爪状态
- 🔧 关节：6个关节角度实时显示
- 🎨 彩色编码和透明度

## 文件输出位置

### 默认位置
- **验证视频**: `/home/sht/DIJA/validation_videos/`
- **日志文件**: 控制台输出

### 自定义位置
使用 `--output-dir` 参数指定输出目录

## 故障排除

### 常见问题

1. **"找不到相机数据"**
   - 检查HDF5文件中是否存在 `observation/camera1`, `observation/camera2` 等
   - 确保相机数据是uint8格式的图像

2. **"轨迹数据格式错误"**
   - `trajectory/position` 应该是 (N, 7) 形状
   - `trajectory/joint` 应该是 (N, 6) 形状

3. **"视频创建失败"**
   - 检查输出目录是否有写权限
   - 确保OpenCV安装正确：`pip install opencv-python`

4. **"内存不足"**
   - 对于大数据集，考虑批量处理较小的块
   - 减少视频分辨率或帧率

### 依赖检查

确保安装了必需的Python包：
```bash
pip install h5py opencv-python numpy
```

## 示例输出

### 检查输出示例
```
📁 文件: jaka_teleop_0000_20250916_152730.hdf5
📊 文件大小: 245.2 MB

📷 相机数据:
  ✅ observation/camera1: (500, 480, 640, 3) - RGB图像
  ✅ observation/camera2: (500, 480, 640, 3) - RGB图像

🎯 轨迹数据:
  ✅ trajectory/position: (500, 7) - [x,y,z,qx,qy,qz,gripper]
  ✅ trajectory/joint: (500, 6) - 关节角度

✅ 数据格式验证通过
```

### 视频生成示例
```
🎬 正在创建验证视频...
📷 发现2个相机视图
📐 使用side_by_side布局
🎥 分辨率: 1280x480, 30.0 FPS
📈 添加轨迹数据叠加
⏱️  处理进度: 100% (500/500帧)
✅ 视频已保存: validation_videos/jaka_teleop_0000.mp4
```

## 使用建议

1. **首次使用**: 先用快速检查工具验证数据格式
2. **批量处理**: 使用一键脚本进行大量数据的处理
3. **质量控制**: 查看生成的视频以验证录制质量
4. **存储管理**: 定期清理验证视频目录以节省空间

## 技术说明

- **图像格式**: 支持RGB和BGR格式自动转换
- **视频编解码**: 使用H.264编码，兼容性好
- **内存优化**: 流式处理大型数据集，避免内存溢出
- **错误处理**: 详细的错误信息和恢复建议
