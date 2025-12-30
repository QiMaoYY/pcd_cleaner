#!/bin/bash
# 步骤三：编辑栅格地图

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 参数
MAP_NAME=${1:-"map_demo"}
MAP_BASE_DIR=${2:-"/media/data/slam_ws/src/kuavo_slam/maps"}

MAP_DIR="$MAP_BASE_DIR/$MAP_NAME"
ORIGINAL_MAP="$MAP_DIR/${MAP_NAME}.pgm"
CLEANED_MAP="$MAP_DIR/${MAP_NAME}_cleaned.pgm"

echo -e "${GREEN}=== 步骤三：编辑栅格地图 ===${NC}\n"

# 检查地图文件是否存在
if [ ! -f "$ORIGINAL_MAP" ]; then
    echo -e "${YELLOW}错误: 地图文件不存在: $ORIGINAL_MAP${NC}"
    echo "请先运行步骤二生成地图"
    exit 1
fi

echo "地图目录: $MAP_DIR"
echo "原始地图: $ORIGINAL_MAP"
echo "清理地图: $CLEANED_MAP"
echo ""

# 复制原始地图为清理地图
if [ -f "$CLEANED_MAP" ]; then
    echo -e "${YELLOW}清理地图已存在，将覆盖${NC}"
fi

cp "$ORIGINAL_MAP" "$CLEANED_MAP"
echo "已复制原始地图到: $CLEANED_MAP"
echo ""

# 检查kolourpaint是否安装
if ! command -v kolourpaint &> /dev/null; then
    echo -e "${YELLOW}警告: 未找到 kolourpaint${NC}"
    echo "请安装: sudo apt install kolourpaint"
    echo ""
    echo "你也可以使用其他图像编辑器手动编辑: $CLEANED_MAP"
    exit 1
fi

# 打开地图编辑器
echo -e "${GREEN}打开地图编辑器...${NC}"
echo "请在编辑器中擦除杂质点（使用白色擦除）"
echo "编辑完成后保存并关闭编辑器"
echo ""

kolourpaint "$CLEANED_MAP"

echo ""
echo -e "${GREEN}=== 编辑完成 ===${NC}"
echo "原始地图: $ORIGINAL_MAP"
echo "清理地图: $CLEANED_MAP"
echo ""
echo "接下来可以运行步骤四进行智能点云过滤"

