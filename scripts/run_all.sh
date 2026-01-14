#!/bin/bash
# 工程入口：点云清理+2D地图+编辑+点云过滤

set -e  # 遇到错误立即退出

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

echo "maps: ${MAP_ROOT}/${MAP_NAME} | ceiling=${CEILING_THRESHOLD} | mode=${MODE}"

case "$MODE" in
  0)
    echo "[1/3] 点云清理 + 2D转换..."
    python3 "$SCRIPT_DIR/preprocess_and_map2d.py" -r "$MAP_ROOT" -n "$MAP_NAME" -c "$CEILING_THRESHOLD"

    echo "[2/3] 编辑2D地图（从init重新开始）..."
    python3 "$SCRIPT_DIR/edit_map.py" --base-dir "$MAP_ROOT" --map-name "$MAP_NAME"

    echo "[3/3] 智能点云过滤..."
    python3 "$SCRIPT_DIR/filter_pcd_by_map.py" -d "$MAP_ROOT" -n "$MAP_NAME"
    ;;
  1)
    echo "[1/1] 点云清理 + 2D转换..."
    python3 "$SCRIPT_DIR/preprocess_and_map2d.py" -r "$MAP_ROOT" -n "$MAP_NAME" -c "$CEILING_THRESHOLD"
    ;;
  2)
    echo "[1/1] 编辑2D地图（从init重新开始）..."
    python3 "$SCRIPT_DIR/edit_map.py" --base-dir "$MAP_ROOT" --map-name "$MAP_NAME"
    ;;
  3)
    echo "[1/1] 智能点云过滤..."
    python3 "$SCRIPT_DIR/filter_pcd_by_map.py" -d "$MAP_ROOT" -n "$MAP_NAME"
    ;;
  4)
    echo "[1/1] 继续编辑2D地图（不从init复制）..."
    python3 "$SCRIPT_DIR/edit_map.py" --base-dir "$MAP_ROOT" --map-name "$MAP_NAME" --resume
    ;;
  *)
    echo "未知运行方式: ${MODE}"
    show_help
    ;;
esac

echo "done: ${MAP_ROOT}/${MAP_NAME}"
