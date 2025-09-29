#!/usr/bin/env python3
"""
convert_to_lerobot.py
作用：将录制的HDF5数据转换为标准的Lerobot数据格式
"""

import argparse
import h5py
import json
import numpy as np
from pathlib import Path
import shutil


def convert_episode_to_lerobot(episode_dir: Path, output_dir: Path):
    """
    将单个episode转换为Lerobot标准格式
    """
    print(f"🔄 转换episode: {episode_dir.name}")
    
    # 读取原始数据
    data_file = episode_dir / "data.hdf5"
    metadata_file = episode_dir / "metadata.json"
    
    if not data_file.exists():
        print(f"❌ 未找到数据文件: {data_file}")
        return False
    
    # 读取元数据
    metadata = {}
    if metadata_file.exists():
        with open(metadata_file, 'r') as f:
            metadata = json.load(f)
    
    # 创建输出目录
    output_episode_dir = output_dir / episode_dir.name
    output_episode_dir.mkdir(parents=True, exist_ok=True)
    
    # 转换HDF5数据
    with h5py.File(data_file, 'r') as src_f:
        output_hdf5 = output_episode_dir / "replay_buffer.hdf5"
        
        with h5py.File(output_hdf5, 'w') as dst_f:
            # 创建Lerobot标准的数据结构
            convert_to_lerobot_format(src_f, dst_f, metadata)
    
    # 创建Lerobot元数据文件
    create_lerobot_metadata(output_episode_dir, metadata)
    
    print(f"✅ 转换完成: {output_episode_dir}")
    return True


def convert_to_lerobot_format(src_f, dst_f, metadata):
    """
    转换为Lerobot的HDF5格式
    """
    # Lerobot期望的数据结构：
    # - action: 动作数据 (T, action_dim)
    # - observation: 观测数据
    #   - state: 状态向量 (T, state_dim)  
    #   - images: 图像数据
    
    num_frames = src_f.attrs.get('num_frames', 0)
    
    # 处理动作数据
    if 'action' in src_f:
        action_group = dst_f.create_group('action')
        
        # 合并VR数据为单一动作向量
        actions = []
        for i in range(num_frames):
            action_vec = []
            
            # VR位置 (3维)
            if 'vr_position' in src_f['action']:
                vr_pos = src_f['action']['vr_position'][i]
                action_vec.extend(vr_pos)
            
            # VR姿态 (4维四元数或3维欧拉角)
            if 'vr_orientation' in src_f['action']:
                vr_ori = src_f['action']['vr_orientation'][i]
                if len(vr_ori) == 4:  # 四元数
                    action_vec.extend(vr_ori)
                else:  # 欧拉角，转换为四元数
                    action_vec.extend(euler_to_quat(vr_ori))
            
            # VR按钮 (n维)
            if 'vr_buttons' in src_f['action']:
                vr_buttons = src_f['action']['vr_buttons'][i]
                action_vec.extend(vr_buttons)
            
            actions.append(action_vec)
        
        # 保存动作数据
        action_group.create_dataset('data', data=np.array(actions))
    
    # 处理观测数据
    if 'observation' in src_f:
        obs_group = dst_f.create_group('observation')
        
        # 状态数据（关节位置、末端位姿等）
        states = []
        for i in range(num_frames):
            state_vec = []
            
            # 关节位置
            if 'joint_position' in src_f['observation']:
                joint_pos = src_f['observation']['joint_position'][i]
                state_vec.extend(joint_pos)
            
            # 关节速度
            if 'joint_velocity' in src_f['observation']:
                joint_vel = src_f['observation']['joint_velocity'][i]
                state_vec.extend(joint_vel)
            
            # 末端位置
            if 'end_effector_position' in src_f['observation']:
                ee_pos = src_f['observation']['end_effector_position'][i]
                state_vec.extend(ee_pos)
            
            # 末端姿态
            if 'end_effector_orientation' in src_f['observation']:
                ee_ori = src_f['observation']['end_effector_orientation'][i]
                state_vec.extend(ee_ori)
            
            states.append(state_vec)
        
        obs_group.create_dataset('state', data=np.array(states))
        
        # 图像数据
        img_group = obs_group.create_group('images')
        for key in src_f['observation'].keys():
            if key.startswith('camera_'):
                camera_name = key[7:]  # 移除'camera_'前缀
                img_data = src_f['observation'][key][:]
                img_group.create_dataset(camera_name, data=img_data)
    
    # 复制属性
    for key, value in src_f.attrs.items():
        dst_f.attrs[key] = value


def euler_to_quat(euler):
    """
    欧拉角转四元数 (简化版本)
    """
    # 简单实现，实际使用时建议使用scipy.spatial.transform
    rx, ry, rz = euler
    
    cy = np.cos(rz * 0.5)
    sy = np.sin(rz * 0.5)
    cp = np.cos(ry * 0.5)
    sp = np.sin(ry * 0.5)
    cr = np.cos(rx * 0.5)
    sr = np.sin(rx * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [x, y, z, w]


def create_lerobot_metadata(output_dir: Path, original_metadata: dict):
    """
    创建Lerobot标准的元数据文件
    """
    lerobot_metadata = {
        "codebase_version": "1.0.0",
        "data_format": "lerobot",
        "created_at": original_metadata.get("created_at", ""),
        "fps": original_metadata.get("fps", 30),
        "episode_name": original_metadata.get("episode_name", ""),
        "num_frames": original_metadata.get("num_frames", 0),
        
        # 数据描述
        "data_description": {
            "action_space": {
                "type": "continuous",
                "description": "VR controller input (position, orientation, buttons)"
            },
            "observation_space": {
                "state": {
                    "type": "continuous", 
                    "description": "Robot joint positions, velocities, and end-effector pose"
                },
                "images": {
                    "type": "images",
                    "description": "Camera observations"
                }
            }
        },
        
        # 原始元数据
        "original_metadata": original_metadata
    }
    
    # 保存元数据
    with open(output_dir / "metadata.json", 'w') as f:
        json.dump(lerobot_metadata, f, indent=2)


def main():
    parser = argparse.ArgumentParser(description="转换JAKA teleop数据为Lerobot格式")
    parser.add_argument("input_dir", type=str, help="输入数据目录")
    parser.add_argument("output_dir", type=str, help="输出数据目录")
    parser.add_argument("--episode", type=str, help="仅转换指定episode")
    
    args = parser.parse_args()
    
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    
    if not input_dir.exists():
        print(f"❌ 输入目录不存在: {input_dir}")
        return
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    if args.episode:
        # 转换单个episode
        episode_dir = input_dir / args.episode
        if episode_dir.exists():
            convert_episode_to_lerobot(episode_dir, output_dir)
        else:
            print(f"❌ Episode不存在: {episode_dir}")
    else:
        # 转换所有episode
        episodes = [d for d in input_dir.iterdir() if d.is_dir()]
        
        if not episodes:
            print(f"❌ 在 {input_dir} 中未找到episode目录")
            return
        
        print(f"🔄 找到 {len(episodes)} 个episodes，开始批量转换...")
        
        success_count = 0
        for episode_dir in episodes:
            if convert_episode_to_lerobot(episode_dir, output_dir):
                success_count += 1
        
        print(f"✅ 转换完成: {success_count}/{len(episodes)} 个episodes成功")
        print(f"📁 输出目录: {output_dir}")


if __name__ == "__main__":
    main()
