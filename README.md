# PCD Cleaner

点云清理工具包，用于SLAM点云地图噪声去除和处理。

## 安装依赖

```bash
conda activate demo
pip3 install open3d numpy scikit-learn opencv-python pyyaml
```

## 快速使用

```bash
cd /media/data/slam_ws/src/pcd_cleaner/scripts
conda activate demo

# 完整流程（推荐）
./run_all.sh [起始步骤] [输入PCD] [地图名] [屋顶阈值]

# 示例
./run_all.sh                                        # 从步骤1开始，使用默认参数
./run_all.sh s2                                     # 从步骤2开始
./run_all.sh s1 /path/to/scans.pcd my_map 0.8     # 自定义参数

# 查看帮助
./run_all.sh --help
```

## 分步使用

### 第一步：地面和屋顶提取及点云清理

```bash
conda activate demo
python3 scripts/extract_ground_ceiling.py \
    -i /media/data/slam_ws/src/kuavo_slam/PCD/scans.pcd \
    -c 0.8 \
    -m 0.1
```

**功能**：去除屋顶、提取地面范围、去除地面以下点

**输出**：
- `data/z_range_data.json` - z轴范围配置
- `scans_cleaned.pcd` - 清理后的点云

### 第二步：点云转栅格地图

```bash
conda activate demo
python3 scripts/pcd_to_map.py \
    -c data/z_range_data.json \
    -n map_demo
```

**功能**：调用pcd2pgm转换，自动保存地图

**输出**：
- `<地图名>.pgm/.yaml` - 栅格地图

### 第三步：编辑栅格地图

```bash
./scripts/edit_map.sh map_demo /media/data/slam_ws/src/kuavo_slam/maps
```

**功能**：使用kolourpaint手动擦除杂质

**输出**：
- `<地图名>_cleaned.pgm` - 编辑后的地图

### 第四步：智能点云过滤

```bash
conda activate demo
python3 scripts/filter_pcd_by_map.py \
    -c data/z_range_data.json \
    -n map_demo
```

**功能**：根据地图差异过滤点云（地面点豁免）

**输出**：
- `<地图名>_final.pcd` - 最终清理的点云

## 处理流程

1. **地面和屋顶提取** ✓
   - 去除z > ceiling_threshold的屋顶点
   - 提取地面平面并计算z轴范围
   - 去除z < ground_z_min - margin的地面以下点

2. **点云转栅格地图** ✓
   - 自动读取第一步配置
   - 调用pcd2pgm转换
   - 使用map_saver自动保存地图

3. **地图编辑** ✓
   - 复制原始地图为_cleaned版本
   - 使用kolourpaint编辑器手动擦除杂质

4. **智能点云过滤** ✓
   - 比对原始地图和清理地图，找出擦除区域
   - 转换像素坐标为世界坐标（考虑分辨率和原点对齐）
   - 过滤对应点云（地面点享有豁免权）

## 依赖

- ROS包: rospy, nav_msgs, map_server, pcd2pgm
- Python包: open3d, numpy, scikit-learn, opencv-python, pyyaml, tqdm

## 项目结构

```
pcd_cleaner/
├── scripts/
│   ├── extract_ground_ceiling.py  # 步骤1：地面和屋顶提取
│   ├── pcd_to_map.py              # 步骤2：点云转栅格地图
│   ├── edit_map.sh                # 步骤3：地图编辑
│   ├── filter_pcd_by_map.py       # 步骤4：智能点云过滤
│   └── run_all.sh                 # 完整流程脚本
├── data/                          # 数据目录
├── config/                        # 配置文件
└── README.md
```
