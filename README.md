# Map Editor

点云清理工具包，用于SLAM点云地图噪声去除和处理。

## 安装依赖

```bash
conda activate demo
pip3 install open3d numpy scikit-learn opencv-python pyyaml
```

## 快速使用

```bash
cd /media/data/slam_ws/src/map_editor/scripts
conda activate demo

# 完整流程（推荐）
./run_all.sh [地图根目录] [地图名] [屋顶高度阈值] [运行方式]

# 示例
./run_all.sh                                        # 默认参数（全流程）
./run_all.sh /media/data/slam_ws/src/kuavo_slam/maps test2 0.8 1
./run_all.sh /media/data/slam_ws/src/kuavo_slam/maps test2 0.8 4

# 查看帮助
./run_all.sh --help
```

## 分步使用

### 第一步：地面提取与2D地图初始化

```bash
conda activate demo
python3 scripts/preprocess_and_map2d.py \
    -r /media/data/slam_ws/src/kuavo_slam/maps \
    -n map_demo \
    -c 0.8
```

**功能**：提取地面范围、去除地面以下点，并生成 map2d_init.pgm/yaml  
**说明**：当前流程不再进行屋顶过滤，`-c` 仅作为生成 2D 地图的上限阈值使用

**输出**：
- `<地图根目录>/<地图名>/range_z.json` - z轴范围配置
- `<地图根目录>/<地图名>/pointcloud_tmp.pcd` - 预处理点云
- `<地图根目录>/<地图名>/map2d_init.pgm/.yaml` - 初始2D地图

### 第二步：编辑栅格地图

```bash
python3 scripts/edit_map.py --base-dir /media/data/slam_ws/src/kuavo_slam/maps --map-name map_demo
```

**功能**：使用kolourpaint手动擦除杂质

**输出**：
- `<地图根目录>/<地图名>/map2d.pgm/.yaml` - 编辑后的地图

### 第三步：智能点云过滤

```bash
conda activate demo
python3 scripts/filter_pcd_by_map.py \
    -d /media/data/slam_ws/src/kuavo_slam/maps \
    -n map_demo
```

**功能**：根据地图差异过滤点云（地面点豁免），并进行体素降采样  
**配置**：体素尺寸 `filter_size_map` 位于 `config/map_editor.yaml`

**输出**：
- `<地图根目录>/<地图名>/pointcloud.pcd` - 最终ICP参考点云

## 处理流程

1. **地面提取与2D地图初始化** ✓
   - 提取地面平面并计算z轴范围
   - 去除z < ground_z_min - margin的地面以下点
   - 生成 map2d_init.pgm/yaml（ceiling_threshold 仅用于2D地图高度上限）

2. **地图编辑** ✓
   - 复制原始地图为 map2d.pgm
   - 使用编辑器手动擦除杂质并生成 map2d.yaml

3. **智能点云过滤** ✓
   - 比对原始地图和清理地图，找出擦除区域
   - 转换像素坐标为世界坐标（考虑分辨率和原点对齐）
   - 过滤对应点云并进行体素降采样

## 依赖

- ROS包: rospy, nav_msgs, map_server, pcd2pgm
- Python包: open3d, numpy, scikit-learn, opencv-python, pyyaml, tqdm

## 项目结构

```
map_editor/
├── scripts/
│   ├── preprocess_and_map2d.py    # 步骤1：预处理+生成map2d_init
│   ├── edit_map.py                # 步骤2：地图编辑
│   ├── filter_pcd_by_map.py       # 步骤3：智能点云过滤+体素降采样
│   ├── run_all.sh                 # 完整流程脚本
│   ├── pcd_to_pgm.py              # 工具：PCD转PGM/YAML
│   └── extract_ground_ceiling.py  # 旧版单步脚本（可选）
├── data/                          # 数据目录
├── config/                        # 配置文件
└── README.md
```
