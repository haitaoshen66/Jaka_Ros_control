#!/usr/bin/env python3
"""
quick_hdf5_check.py
作用：快速检查HDF5文件内容和结构
"""

import h5py
import numpy as np
import argparse
import os
from pathlib import Path


def print_hdf5_structure(group, indent=0, max_depth=3):
    """递归打印HDF5结构"""
    if indent > max_depth:
        return
        
    prefix = "  " * indent
    for key in group.keys():
        item = group[key]
        if isinstance(item, h5py.Group):
            print(f"{prefix}📁 {key}/")
            print_hdf5_structure(item, indent + 1, max_depth)
        else:
            shape_str = f"{item.shape}" if hasattr(item, 'shape') else "scalar"
            dtype_str = f"{item.dtype}" if hasattr(item, 'dtype') else "unknown"
            print(f"{prefix}📄 {key}: {shape_str} {dtype_str}")
            
            # 如果是小数组，显示一些示例值
            if hasattr(item, 'shape') and np.prod(item.shape) <= 10:
                try:
                    values = item[...]
                    print(f"{prefix}   Values: {values}")
                except:
                    pass


def check_hdf5_file(hdf5_path):
    """检查HDF5文件"""
    hdf5_path = Path(hdf5_path)
    
    if not hdf5_path.exists():
        print(f"❌ 文件不存在: {hdf5_path}")
        return False
    
    print(f"📁 检查HDF5文件: {hdf5_path}")
    print(f"📊 文件大小: {hdf5_path.stat().st_size / (1024*1024):.2f} MB")
    
    try:
        with h5py.File(hdf5_path, 'r') as f:
            print("\n📋 HDF5文件结构:")
            print_hdf5_structure(f)
            
            print("\n📊 元数据 (Attributes):")
            for attr_name in f.attrs.keys():
                attr_value = f.attrs[attr_name]
                print(f"  {attr_name}: {attr_value}")
            
            # 检查观测数据
            if 'observation' in f:
                print("\n📷 观测数据 (Observation):")
                obs_group = f['observation']
                for key in obs_group.keys():
                    if key.startswith('camera'):
                        data = obs_group[key]
                        print(f"  {key}: {data.shape} {data.dtype}")
                        if len(data.shape) >= 3:
                            print(f"    帧数: {data.shape[0]}")
                            print(f"    图像尺寸: {data.shape[1]}x{data.shape[2]}")
                            if len(data.shape) == 4:
                                print(f"    通道数: {data.shape[3]}")
            
            # 检查轨迹数据
            if 'trajectory' in f:
                print("\n📈 轨迹数据 (Trajectory):")
                traj_group = f['trajectory']
                for key in traj_group.keys():
                    data = traj_group[key]
                    print(f"  {key}: {data.shape} {data.dtype}")
                    if len(data.shape) >= 2:
                        print(f"    帧数: {data.shape[0]}")
                        print(f"    维度: {data.shape[1]}")
                        
                        # 显示一些统计信息
                        try:
                            values = data[...]
                            print(f"    范围: [{np.min(values):.3f}, {np.max(values):.3f}]")
                            print(f"    均值: {np.mean(values):.3f}")
                        except:
                            pass
            
            print(f"\n✅ HDF5文件检查完成")
            return True
            
    except Exception as e:
        print(f"❌ 检查HDF5文件失败: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="快速检查HDF5文件内容")
    parser.add_argument("hdf5_path", help="HDF5数据文件路径")
    
    args = parser.parse_args()
    
    check_hdf5_file(args.hdf5_path)


if __name__ == "__main__":
    main()
