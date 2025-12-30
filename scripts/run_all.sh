#!/bin/bash
# 完整流程：点云清理 -> 转换地图 -> 编辑地图 -> 智能过滤

set -e  # 遇到错误立即退出

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 显示帮助信息
show_help() {
    echo "用法: $0 [起始步骤] [输入PCD] [地图名] [屋顶阈值]"
    echo ""
    echo "参数:"
    echo "  起始步骤      从哪一步开始运行 (s1|s2|s3|s4, 默认: s1)"
    echo "  输入PCD       输入PCD文件路径 (默认: /media/data/slam_ws/src/kuavo_slam/PCD/scans.pcd)"
    echo "  地图名        地图名称 (默认: map_demo)"
    echo "  屋顶阈值      屋顶高度阈值 (默认: 0.8)"
    echo ""
    echo "示例:"
    echo "  $0                                    # 从步骤1开始，使用默认参数"
    echo "  $0 s2                                 # 从步骤2开始"
    echo "  $0 s1 /path/to/scans.pcd my_map 0.8  # 自定义所有参数"
    echo ""
    echo "步骤说明:"
    echo "  s1 - 点云清理（提取地面和屋顶，去除噪声）"
    echo "  s2 - 转换栅格地图（PCD转2D地图）"
    echo "  s3 - 编辑地图（手动擦除杂质）"
    echo "  s4 - 智能点云过滤（根据编辑后的地图过滤点云）"
    exit 0
}

# 解析起始步骤参数
START_STEP="s1"
if [[ "$1" =~ ^s[1-4]$ ]]; then
    START_STEP="$1"
    shift
elif [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
    show_help
fi

# 解析其他参数
INPUT_PCD=${1:-"/media/data/slam_ws/src/kuavo_slam/PCD/scans.pcd"}
MAP_NAME=${2:-"map_demo"}
CEILING_THRESHOLD=${3:-"0.8"}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${GREEN}=== PCD Cleaner 完整流程 ===${NC}\n"
echo "起始步骤: $START_STEP"
echo "输入PCD: $INPUT_PCD"
echo "地图名称: $MAP_NAME"
echo "屋顶阈值: $CEILING_THRESHOLD"
echo ""

# 激活conda环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate demo

# 步骤1：点云清理
if [[ "$START_STEP" == "s1" ]]; then
    echo -e "${BLUE}[步骤 1/4] 清理点云...${NC}"
    python3 "$SCRIPT_DIR/extract_ground_ceiling.py" \
        -i "$INPUT_PCD" \
        -o "$SCRIPT_DIR/../data/z_range_data.json" \
        -c "$CEILING_THRESHOLD"
    
    if [ $? -ne 0 ]; then
        echo -e "${YELLOW}点云清理失败${NC}"
        exit 1
    fi
    echo ""
else
    echo -e "${YELLOW}[步骤 1/4] 跳过${NC}"
fi

# 步骤2：转换地图
if [[ "$START_STEP" =~ ^s[1-2]$ ]]; then
    echo -e "${BLUE}[步骤 2/4] 转换栅格地图...${NC}"
    python3 "$SCRIPT_DIR/pcd_to_map.py" \
        -c "$SCRIPT_DIR/../data/z_range_data.json" \
        -n "$MAP_NAME" \
        -o /media/data/slam_ws/src/kuavo_slam/maps
    
    if [ $? -ne 0 ]; then
        echo -e "${YELLOW}地图转换失败${NC}"
        exit 1
    fi
    echo ""
else
    echo -e "${YELLOW}[步骤 2/4] 跳过${NC}"
fi

# 步骤3：编辑地图
if [[ "$START_STEP" =~ ^s[1-3]$ ]]; then
    echo -e "${BLUE}[步骤 3/4] 编辑栅格地图...${NC}"
    "$SCRIPT_DIR/edit_map.sh" "$MAP_NAME" /media/data/slam_ws/src/kuavo_slam/maps
    
    if [ $? -ne 0 ]; then
        echo -e "${YELLOW}地图编辑失败${NC}"
        exit 1
    fi
    echo ""
else
    echo -e "${YELLOW}[步骤 3/4] 跳过${NC}"
fi

# 步骤4：智能点云过滤
if [[ "$START_STEP" =~ ^s[1-4]$ ]]; then
    echo -e "${BLUE}[步骤 4/4] 智能点云过滤...${NC}"
    python3 "$SCRIPT_DIR/filter_pcd_by_map.py" \
        -c "$SCRIPT_DIR/../data/z_range_data.json" \
        -n "$MAP_NAME" \
        -d /media/data/slam_ws/src/kuavo_slam/maps
    
    if [ $? -ne 0 ]; then
        echo -e "${YELLOW}点云过滤失败${NC}"
        exit 1
    fi
    echo ""
fi

echo -e "${GREEN}=== 完成！===${NC}"
echo "输出位置: /media/data/slam_ws/src/kuavo_slam/maps/$MAP_NAME/"
echo "  - ${MAP_NAME}.pgm/yaml         原始地图"
echo "  - ${MAP_NAME}_cleaned.pgm      清理后的地图"
echo "  - ${MAP_NAME}_final.pcd        最终点云"
