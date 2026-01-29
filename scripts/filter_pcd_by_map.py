#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能点云过滤脚本
比对原始地图和清理地图，根据差异过滤点云
"""

import numpy as np
import open3d as o3d
import json
import os
import sys
import argparse
import cv2
import yaml


class PCDMapFilter:
    def __init__(self, z_range_json_path, map_name, map_base_dir, config_path=None):
        """
        初始化点云地图过滤器
        
        Args:
            z_range_json_path: z轴范围数据JSON文件路径
            map_name: 地图名称
            map_base_dir: 地图基础目录
        """
        self.z_range_json_path = z_range_json_path
        self.map_name = map_name
        self.map_dir = os.path.join(map_base_dir, map_name)
        self.config_path = config_path or self._default_config_path()
        
        self.z_data = None
        self.pcd = None
        self.original_map = None
        self.cleaned_map = None
        self.map_info = None
        self.filter_size_map = 0.05

    @staticmethod
    def _default_config_path():
        script_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(script_dir, "..", "config", "map_editor.yaml"))

    def load_global_config(self):
        """加载全局配置（体素滤波参数）"""
        if not os.path.exists(self.config_path):
            print(f"警告: 配置文件不存在: {self.config_path}，使用默认体素 {self.filter_size_map}m")
            return True

        try:
            with open(self.config_path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
            self.filter_size_map = float(cfg.get('filter_size_map', self.filter_size_map))
        except Exception as e:
            print(f"警告: 读取配置失败: {e}，使用默认体素 {self.filter_size_map}m")
        return True
        
    def load_z_range_data(self):
        """加载z轴范围数据"""

        if not os.path.exists(self.z_range_json_path):
            print(f"错误: 配置文件不存在: {self.z_range_json_path}")
            return False

        with open(self.z_range_json_path, 'r') as f:
            self.z_data = json.load(f)
        
        # 简化输出：不打印过多调试信息
        
        return True
    
    def load_pcd(self):
        """加载PCD点云"""
        pcd_path = os.path.join(self.map_dir, "pointcloud_tmp.pcd")
        
        if not os.path.exists(pcd_path):
            print(f"错误: PCD文件不存在: {pcd_path}")
            return False
        
        self.pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(self.pcd.points)
        print(f"点数: {len(points)}")
        
        return True
    
    def load_maps(self):
        """加载原始地图和清理地图"""
        original_map_path = os.path.join(self.map_dir, "map2d_init.pgm")
        cleaned_map_path = os.path.join(self.map_dir, "map2d.pgm")
        map_yaml_path = os.path.join(self.map_dir, "map2d.yaml")
        
        # 检查文件存在
        if not os.path.exists(original_map_path):
            print(f"错误: 原始地图不存在: {original_map_path}")
            return False
        
        if not os.path.exists(cleaned_map_path):
            print(f"错误: 清理地图不存在: {cleaned_map_path}")
            return False
        
        if not os.path.exists(map_yaml_path):
            print(f"错误: 地图配置不存在: {map_yaml_path}")
            return False
        
        # 加载地图图像

        self.original_map = cv2.imread(original_map_path, cv2.IMREAD_GRAYSCALE)
        self.cleaned_map = cv2.imread(cleaned_map_path, cv2.IMREAD_GRAYSCALE)
        if self.original_map is None:
            print(f"错误: 无法读取原始地图: {original_map_path}")
            return False
        if self.cleaned_map is None:
            print(f"错误: 无法读取清理地图: {cleaned_map_path}")
            return False
        
        # 加载地图配置
        with open(map_yaml_path, 'r') as f:
            self.map_info = yaml.safe_load(f)
        
        # 简化输出
        
        return True
    
    def find_erased_regions(self, threshold=30):
        """找出被擦除的区域"""
        print("分析擦除区域...")
        
        # 比对两张地图的差异
        # 在cleared_map中，擦除的地方会变成白色（255）或更亮
        # 原始地图中，障碍物是黑色（0），自由空间是白色（255），未知是灰色（205）
        
        # 找出清理地图中变亮的区域（被擦除的障碍物）
        diff = self.cleaned_map.astype(int) - self.original_map.astype(int)
        
        # 擦除区域：清理地图比原始地图亮的地方（阈值可调）
        erased_mask = diff > threshold
        
        erased_pixels = np.argwhere(erased_mask)
        print(f"擦除像素: {len(erased_pixels)}")
        
        return erased_mask
    
    def map_pixels_to_world_coords(self, erased_mask):
        """将地图像素坐标转换为世界坐标"""
        # 无需额外输出
        
        # 获取地图参数
        resolution = self.map_info['resolution']  # m/pixel
        origin_x = self.map_info['origin'][0]  # 地图原点在世界坐标系中的x
        origin_y = self.map_info['origin'][1]  # 地图原点在世界坐标系中的y
        
        height, width = erased_mask.shape
        
        # 获取擦除区域的像素坐标
        erased_pixels = np.argwhere(erased_mask)
        
        if len(erased_pixels) == 0:
            return []
        
        # 转换坐标
        # 在PGM图像中：
        # - 图像坐标 (row, col)，其中 row=0 在图像顶部
        # - 世界坐标：需要根据origin和resolution转换
        # 
        # 世界坐标 = origin + pixel * resolution
        # 注意：图像的y轴是向下的，而世界坐标的y轴是向上的
        
        erased_regions = []
        for row, col in erased_pixels:
            # 将图像坐标转换为世界坐标
            # 注意：图像row从上到下增加，但世界y从下到上增加
            world_x = origin_x + col * resolution
            world_y = origin_y + (height - 1 - row) * resolution
            
            erased_regions.append([world_x, world_y])
        
        erased_regions = np.array(erased_regions)
        
        return erased_regions
    
    def filter_points(self, erased_regions):
        """根据擦除区域过滤点云（高效版本）"""
        print("过滤点云...")
        
        points = np.asarray(self.pcd.points)
        
        if len(erased_regions) == 0:
            print("没有需要过滤的区域，保留所有点")
            return self.pcd
        
        # 获取地面z范围
        ground_z_min = self.z_data['ground_z_range']['min']
        ground_z_max = self.z_data['ground_z_range']['max']
        
        # 地图参数
        resolution = self.map_info['resolution']
        origin_x = self.map_info['origin'][0]
        origin_y = self.map_info['origin'][1]
        
        # 方法：使用网格哈希表快速查询
        # 1. 将擦除区域栅格化到网格单元，存入集合
        grid_x_e = np.round((erased_regions[:, 0] - origin_x) / resolution).astype(np.int32)
        grid_y_e = np.round((erased_regions[:, 1] - origin_y) / resolution).astype(np.int32)
        erased_grid_set = set(zip(grid_x_e.tolist(), grid_y_e.tolist()))
        
        # 2. 向量化处理点云
        # 无需额外输出
        
        # 分离地面点和非地面点
        z_coords = points[:, 2]
        is_ground = (z_coords >= ground_z_min) & (z_coords <= ground_z_max)
        
        # 地面点直接保留
        keep_mask = is_ground.copy()
        
        # 对于非地面点，检查是否在擦除区域
        non_ground_indices = np.where(~is_ground)[0]
        
        if len(non_ground_indices) > 0:
            # 将非地面点的xy坐标栅格化
            non_ground_points = points[non_ground_indices]
            grid_x = np.round((non_ground_points[:, 0] - origin_x) / resolution).astype(int)
            grid_y = np.round((non_ground_points[:, 1] - origin_y) / resolution).astype(int)
            
            # 批量检查是否在擦除集合中（无进度条）
            remove_mask = np.fromiter(
                ((gx, gy) in erased_grid_set for gx, gy in zip(grid_x, grid_y)),
                dtype=bool,
                count=len(grid_x),
            )
            keep_mask[non_ground_indices] = ~remove_mask
        
        # 统计
        num_removed = np.sum(~keep_mask)
        num_kept = np.sum(keep_mask)
        num_ground_kept = np.sum(is_ground)
        
        print(f"过滤完成: 保留 {num_kept}/{len(points)}，移除 {num_removed}（地面豁免 {num_ground_kept}）")
        
        # 创建过滤后的点云
        filtered_points = points[keep_mask]
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
        
        # 如果有颜色信息，也保留
        if self.pcd.has_colors():
            colors = np.asarray(self.pcd.colors)
            filtered_pcd.colors = o3d.utility.Vector3dVector(colors[keep_mask])
        
        return filtered_pcd

    def voxel_downsample(self, pcd):
        """对最终点云进行体素降采样"""
        if self.filter_size_map <= 0:
            print("体素大小<=0，跳过降采样")
            return pcd

        before = len(pcd.points)
        down_pcd = pcd.voxel_down_sample(self.filter_size_map)
        after = len(down_pcd.points)
        print(f"体素降采样: voxel={self.filter_size_map}m, {before} -> {after}")
        return down_pcd
    
    def save_filtered_pcd(self, filtered_pcd):
        """保存过滤后的点云"""
        output_path = os.path.join(self.map_dir, "pointcloud.pcd")
        
        o3d.io.write_point_cloud(output_path, filtered_pcd)
        print("保存成功!")
        
        return output_path
    
    def process(self):
        """执行完整的过滤流程"""
        # 0. 加载全局配置（体素滤波参数）
        self.load_global_config()

        # 1. 加载配置
        if not self.load_z_range_data():
            return False
        
        # 2. 加载PCD
        if not self.load_pcd():
            return False
        
        # 3. 加载地图
        if not self.load_maps():
            return False
        
        # 4. 找出擦除区域（阈值30，更灵敏）
        erased_mask = self.find_erased_regions(threshold=30)
        
        # 5. 转换为世界坐标
        erased_regions = self.map_pixels_to_world_coords(erased_mask)
        
        # 6. 过滤点云
        filtered_pcd = self.filter_points(erased_regions)

        # 6.1 体素降采样（用于ICP参考点云）
        filtered_pcd = self.voxel_downsample(filtered_pcd)
        
        # 7. 保存结果
        output_path = self.save_filtered_pcd(filtered_pcd)
        
        print(f"\n=== 处理完成 ===")
        
        return True


def main():
    parser = argparse.ArgumentParser(description='根据编辑后的地图过滤点云')
    parser.add_argument('--map-name', '-n', type=str, default='map_demo',
                        help='地图名称')
    parser.add_argument('--map-dir', '-d', type=str, 
                        default='/media/data/slam_ws/src/kuavo_slam/maps',
                        help='地图基础目录')
    parser.add_argument('--config', type=str, default=None,
                        help='map_editor配置文件路径（默认使用包内config/map_editor.yaml）')
    
    args = parser.parse_args()
    
    # 设置默认配置文件路径
    config_dir = args.map_dir + "/" + args.map_name + "/range_z.json"
    # 创建过滤器
    filter_obj = PCDMapFilter(config_dir, args.map_name, args.map_dir, args.config)
    
    # 执行过滤
    success = filter_obj.process()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

