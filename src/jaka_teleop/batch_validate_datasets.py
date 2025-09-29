#!/usr/bin/env python3
"""
batch_validate_datasets.py
作用：批量验证数据集目录中的所有HDF5文件
"""

import os
import sys
from pathlib import Path
import argparse
from quick_hdf5_check import check_hdf5_file
from validate_hdf5_data import HDF5VideoValidator


def find_hdf5_files(dataset_dir):
    """查找数据集目录中的所有HDF5文件"""
    dataset_path = Path(dataset_dir)
    
    if not dataset_path.exists():
        print(f"❌ 数据集目录不存在: {dataset_dir}")
        return []
    
    hdf5_files = list(dataset_path.rglob("*.hdf5"))
    print(f"📁 在 {dataset_dir} 中找到 {len(hdf5_files)} 个HDF5文件")
    
    return hdf5_files


def batch_check_files(hdf5_files):
    """批量检查HDF5文件"""
    print(f"\n🔍 开始批量检查 {len(hdf5_files)} 个文件...")
    
    results = []
    
    for i, hdf5_file in enumerate(hdf5_files, 1):
        print(f"\n{'='*60}")
        print(f"📋 检查文件 {i}/{len(hdf5_files)}: {hdf5_file.name}")
        print(f"📁 路径: {hdf5_file}")
        print(f"{'='*60}")
        
        try:
            success = check_hdf5_file(hdf5_file)
            results.append((hdf5_file, success))
        except Exception as e:
            print(f"❌ 检查失败: {e}")
            results.append((hdf5_file, False))
    
    # 统计结果
    successful = sum(1 for _, success in results if success)
    print(f"\n📊 批量检查完成:")
    print(f"✅ 成功: {successful}/{len(hdf5_files)}")
    print(f"❌ 失败: {len(hdf5_files) - successful}/{len(hdf5_files)}")
    
    # 显示失败的文件
    failed_files = [file for file, success in results if not success]
    if failed_files:
        print(f"\n❌ 失败的文件:")
        for file in failed_files:
            print(f"  - {file}")
    
    return results


def batch_create_videos(hdf5_files, output_dir=None):
    """批量创建验证视频"""
    print(f"\n🎬 开始批量创建视频...")
    
    if output_dir:
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
    
    successful_videos = 0
    
    for i, hdf5_file in enumerate(hdf5_files, 1):
        print(f"\n🎥 处理视频 {i}/{len(hdf5_files)}: {hdf5_file.name}")
        
        try:
            # 确定输出视频路径
            if output_dir:
                video_name = hdf5_file.stem + ".mp4"
                video_path = output_path / video_name
            else:
                video_path = hdf5_file.parent / (hdf5_file.stem + "_merged.mp4")
            
            # 创建验证器并生成视频
            validator = HDF5VideoValidator(hdf5_file, video_path)
            success = validator.validate_and_create_video()
            
            if success:
                successful_videos += 1
                print(f"✅ 视频创建成功: {video_path}")
            else:
                print(f"❌ 视频创建失败")
                
        except Exception as e:
            print(f"❌ 视频创建异常: {e}")
    
    print(f"\n🎬 批量视频创建完成:")
    print(f"✅ 成功: {successful_videos}/{len(hdf5_files)}")
    
    return successful_videos


def main():
    parser = argparse.ArgumentParser(description="批量验证数据集")
    parser.add_argument("dataset_dir", help="数据集目录路径")
    parser.add_argument("--check-only", action="store_true", help="只检查文件，不创建视频")
    parser.add_argument("--video-only", action="store_true", help="只创建视频，不检查文件")
    parser.add_argument("--output-dir", help="视频输出目录")
    parser.add_argument("--pattern", default="*.hdf5", help="文件匹配模式")
    
    args = parser.parse_args()
    
    # 查找HDF5文件
    hdf5_files = find_hdf5_files(args.dataset_dir)
    
    if not hdf5_files:
        print("❌ 没有找到HDF5文件")
        return
    
    # 执行检查
    if not args.video_only:
        batch_check_files(hdf5_files)
    
    # 创建视频
    if not args.check_only:
        batch_create_videos(hdf5_files, args.output_dir)
    
    print(f"\n🎉 所有任务完成！")


if __name__ == "__main__":
    main()
