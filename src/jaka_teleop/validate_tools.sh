#!/bin/bash
# validate_tools.sh
# 一键验证脚本

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 数据集目录
DATASET_DIR="/home/sht/DIJA/lerobot_datasets"
OUTPUT_DIR="/home/sht/DIJA/validation_videos"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}      JAKA数据集验证工具箱${NC}"
echo -e "${BLUE}========================================${NC}"

# 显示菜单
show_menu() {
    echo -e "\n${YELLOW}请选择操作:${NC}"
    echo "1) 检查单个HDF5文件"
    echo "2) 创建单个验证视频"
    echo "3) 批量检查所有数据集"
    echo "4) 批量创建所有验证视频"
    echo "5) 完整验证（检查+视频）"
    echo "6) 清理验证视频目录"
    echo "0) 退出"
    echo
}

# 检查Python脚本
check_scripts() {
    if [[ ! -f "$SCRIPT_DIR/quick_hdf5_check.py" ]]; then
        echo -e "${RED}❌ 缺少: quick_hdf5_check.py${NC}"
        exit 1
    fi
    
    if [[ ! -f "$SCRIPT_DIR/validate_hdf5_data.py" ]]; then
        echo -e "${RED}❌ 缺少: validate_hdf5_data.py${NC}"
        exit 1
    fi
    
    if [[ ! -f "$SCRIPT_DIR/batch_validate_datasets.py" ]]; then
        echo -e "${RED}❌ 缺少: batch_validate_datasets.py${NC}"
        exit 1
    fi
}

# 选择文件
select_hdf5_file() {
    echo -e "${YELLOW}可用的HDF5文件:${NC}"
    mapfile -t files < <(find "$DATASET_DIR" -name "*.hdf5" 2>/dev/null)
    
    if [[ ${#files[@]} -eq 0 ]]; then
        echo -e "${RED}❌ 在 $DATASET_DIR 中没有找到HDF5文件${NC}"
        return 1
    fi
    
    for i in "${!files[@]}"; do
        basename_file=$(basename "${files[i]}")
        dirname_file=$(dirname "${files[i]}" | sed "s|$DATASET_DIR||" | sed 's|^/||')
        echo "$((i+1))) $dirname_file/$basename_file"
    done
    
    echo
    read -p "选择文件编号: " choice
    
    if [[ "$choice" =~ ^[0-9]+$ ]] && [[ "$choice" -ge 1 ]] && [[ "$choice" -le ${#files[@]} ]]; then
        selected_file="${files[$((choice-1))]}"
        echo -e "${GREEN}选择了: $(basename "$selected_file")${NC}"
        return 0
    else
        echo -e "${RED}❌ 无效选择${NC}"
        return 1
    fi
}

# 主循环
main() {
    check_scripts
    
    while true; do
        show_menu
        read -p "请输入选择: " choice
        
        case $choice in
            1)
                echo -e "\n${BLUE}=== 检查单个HDF5文件 ===${NC}"
                if select_hdf5_file; then
                    echo -e "${YELLOW}正在检查文件...${NC}"
                    python3 "$SCRIPT_DIR/quick_hdf5_check.py" "$selected_file"
                fi
                ;;
            2)
                echo -e "\n${BLUE}=== 创建单个验证视频 ===${NC}"
                if select_hdf5_file; then
                    mkdir -p "$OUTPUT_DIR"
                    output_video="$OUTPUT_DIR/$(basename "${selected_file%.*}").mp4"
                    echo -e "${YELLOW}正在创建视频...${NC}"
                    python3 "$SCRIPT_DIR/validate_hdf5_data.py" "$selected_file" -o "$output_video"
                fi
                ;;
            3)
                echo -e "\n${BLUE}=== 批量检查所有数据集 ===${NC}"
                python3 "$SCRIPT_DIR/batch_validate_datasets.py" "$DATASET_DIR" --check-only
                ;;
            4)
                echo -e "\n${BLUE}=== 批量创建所有验证视频 ===${NC}"
                mkdir -p "$OUTPUT_DIR"
                python3 "$SCRIPT_DIR/batch_validate_datasets.py" "$DATASET_DIR" --video-only --output-dir "$OUTPUT_DIR"
                ;;
            5)
                echo -e "\n${BLUE}=== 完整验证（检查+视频） ===${NC}"
                mkdir -p "$OUTPUT_DIR"
                python3 "$SCRIPT_DIR/batch_validate_datasets.py" "$DATASET_DIR" --output-dir "$OUTPUT_DIR"
                ;;
            6)
                echo -e "\n${BLUE}=== 清理验证视频目录 ===${NC}"
                if [[ -d "$OUTPUT_DIR" ]]; then
                    read -p "确定要删除 $OUTPUT_DIR 中的所有文件吗？ (y/N): " confirm
                    if [[ "$confirm" =~ ^[Yy]$ ]]; then
                        rm -rf "$OUTPUT_DIR"/*
                        echo -e "${GREEN}✅ 清理完成${NC}"
                    else
                        echo -e "${YELLOW}取消清理${NC}"
                    fi
                else
                    echo -e "${YELLOW}验证视频目录不存在${NC}"
                fi
                ;;
            0)
                echo -e "${GREEN}退出程序${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}❌ 无效选择，请重新输入${NC}"
                ;;
        esac
        
        echo
        read -p "按Enter键继续..."
    done
}

# 运行主程序
main
