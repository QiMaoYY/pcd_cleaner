#!/bin/bash
# 工程入口：点云清理+2D地图+编辑+点云过滤

set -e  # 遇到错误立即退出

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

show_help() {
  echo "用法: $0 [地图根目录] [地图名] [屋顶高度阈值] [运行方式]"
  echo ""
  echo "参数:"
  echo "  地图根目录       默认 /media/data/slam_ws/src/kuavo_slam/maps"
  echo "  地图名           默认 map_demo"
  echo "  屋顶高度阈值     默认 0.8"
  echo "  运行方式:"
  echo "    0 全流程（清理+2D转换+重新从init开始编辑+过滤）"
  echo "    1 仅执行 点云清理+2D转换"
  echo "    2 仅执行 编辑2D地图（从init复制一份重新编辑）"
  echo "    3 仅执行 智能点云过滤"
  echo "    4 仅执行 继续编辑2D地图（不从init复制）"
  echo ""
  echo "示例:"
  echo "  $0"
  echo "  $0 /media/data/slam_ws/src/kuavo_slam/maps test2 0.8 1"
  exit 0
}

# help
if [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
  show_help
fi

# 参数
MAP_ROOT=${1:-"/media/data/slam_ws/src/kuavo_slam/maps"}
MAP_NAME=${2:-"map_demo"}
CEILING_THRESHOLD=${3:-"0.8"}
MODE=${4:-"0"}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"


# 激活conda环境
source ~/miniconda3/etc/profile.d/conda.sh
conda activate demo

# ROS 环境（用于 roslaunch/map_saver）
source /media/data/slam_ws/devel/setup.bash
source /media/data/slam_ws/roslib/devel/setup.bash

echo -e "${GREEN}地图根目录: ${MAP_ROOT}${NC}"
echo -e "${GREEN}地图名: ${MAP_NAME}${NC}"
echo -e "${GREEN}屋顶阈值: ${CEILING_THRESHOLD}${NC}"
echo -e "${GREEN}运行方式: ${MODE}${NC}"

case "$MODE" in
  0)
    echo -e "${BLUE}[1/3] 点云清理 + 2D转换...${NC}"
    python3 "$SCRIPT_DIR/preprocess_and_map2d.py" -r "$MAP_ROOT" -n "$MAP_NAME" -c "$CEILING_THRESHOLD"

    echo -e "${BLUE}[2/3] 编辑2D地图（从init重新开始）...${NC}"
    python3 "$SCRIPT_DIR/edit_map.py" --base-dir "$MAP_ROOT" --map-name "$MAP_NAME"

    echo -e "${BLUE}[3/3] 智能点云过滤...${NC}"
    python3 "$SCRIPT_DIR/filter_pcd_by_map.py" -d "$MAP_ROOT" -n "$MAP_NAME"
    ;;
  1)
    echo -e "${BLUE}[1/1] 点云清理 + 2D转换...${NC}"
    python3 "$SCRIPT_DIR/preprocess_and_map2d.py" -r "$MAP_ROOT" -n "$MAP_NAME" -c "$CEILING_THRESHOLD"
    ;;
  2)
    echo -e "${BLUE}[1/1] 编辑2D地图（从init重新开始）...${NC}"
    python3 "$SCRIPT_DIR/edit_map.py" --base-dir "$MAP_ROOT" --map-name "$MAP_NAME"
    ;;
  3)
    echo -e "${BLUE}[1/1] 智能点云过滤...${NC}"
    python3 "$SCRIPT_DIR/filter_pcd_by_map.py" -d "$MAP_ROOT" -n "$MAP_NAME"
    ;;
  4)
    echo -e "${BLUE}[1/1] 继续编辑2D地图（不从init复制）...${NC}"
    python3 "$SCRIPT_DIR/edit_map.py" --base-dir "$MAP_ROOT" --map-name "$MAP_NAME" --resume
    ;;
  *)
    echo -e "${YELLOW}未知运行方式: ${MODE}${NC}"
    show_help
    ;;
esac

echo -e "${GREEN}=== 完成！===${NC}"
echo "输出位置: ${MAP_ROOT}/${MAP_NAME}/"
echo "  - pointcloud_original.pcd   原始点云(输入)"
echo "  - pointcloud_tmp.pcd        预处理点云(步骤1输出)"
echo "  - range_z.json              地面范围/屋顶阈值"
echo "  - map2d_init.pgm/.yaml      初始2D地图(步骤2输出)"
echo "  - map2d.pgm                 编辑后的2D地图(步骤3输出)"
echo "  - map2d.yaml                指向编辑地图的yaml(步骤3输出)"
echo "  - sketch.png                编辑地图png预览(步骤3输出)"
echo "  - pointcloud.pcd            最终点云(步骤4输出)"
