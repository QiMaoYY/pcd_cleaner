#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地面和屋顶提取脚本
使用RANSAC算法提取地面和屋顶，并计算z轴范围
"""

import numpy as np
import open3d as o3d
import json
import os
import sys
import argparse
from sklearn.linear_model import RANSACRegressor


class GroundCeilingExtractor:
    def __init__(self, pcd_path, output_data_path, output_pcd_path=None):
        """
        初始化地面和屋顶提取器
        
        Args:
            pcd_path: 输入PCD文件路径
            output_data_path: 输出数据文件路径（JSON格式）
            output_pcd_path: 输出清理后的PCD文件路径
        """
        self.pcd_path = pcd_path
        self.output_data_path = output_data_path
        self.output_pcd_path = output_pcd_path or pcd_path.replace('.pcd', '_cleaned.pcd')
        self.pcd = None
        self.ground_z_range = None
        self.ceiling_z_threshold = None
        
    def load_pcd(self):
        """加载PCD文件"""
        print(f"正在加载PCD文件: {self.pcd_path}")
        try:
            self.pcd = o3d.io.read_point_cloud(self.pcd_path)
            points = np.asarray(self.pcd.points)
            print(f"成功加载点云，共 {len(points)} 个点")
            print(f"Z轴范围: {points[:, 2].min():.3f} ~ {points[:, 2].max():.3f}")
            return True
        except Exception as e:
            print(f"加载PCD文件失败: {e}")
            return False
    
    def remove_ceiling(self, ceiling_z_threshold=0.8):
        """
        直接去除屋顶（z > ceiling_z_threshold的点）
        
        Args:
            ceiling_z_threshold: 屋顶高度阈值（米）
            
        Returns:
            去除屋顶后的点云
        """
        print(f"\n=== 去除屋顶 (z > {ceiling_z_threshold}) ===")
        points = np.asarray(self.pcd.points)
        
        # 统计z > ceiling_z_threshold的点
        ceiling_mask = points[:, 2] > ceiling_z_threshold
        num_ceiling_points = np.sum(ceiling_mask)
        
        print(f"z > {ceiling_z_threshold} 的点数: {num_ceiling_points}")
        
        # 保留z <= ceiling_z_threshold的点
        remaining_mask = points[:, 2] <= ceiling_z_threshold
        remaining_points = points[remaining_mask]
        
        print(f"去除屋顶后剩余点数: {len(remaining_points)} (去除了 {num_ceiling_points} 个点)")
        
        # 记录屋顶阈值
        self.ceiling_z_threshold = ceiling_z_threshold
        
        # 创建去除屋顶后的点云
        remaining_pcd = o3d.geometry.PointCloud()
        remaining_pcd.points = o3d.utility.Vector3dVector(remaining_points)
        
        # 如果原点云有颜色，也保留对应的颜色
        if self.pcd.has_colors():
            colors = np.asarray(self.pcd.colors)
            remaining_pcd.colors = o3d.utility.Vector3dVector(colors[remaining_mask])
        
        return remaining_pcd
    
    def extract_ground(self, pcd, distance_threshold=0.05, ransac_n=3, num_iterations=1000, percentile=95):
        """
        使用RANSAC算法提取地面（z < 0的部分）
        
        Args:
            pcd: 输入点云
            distance_threshold: RANSAC距离阈值
            ransac_n: RANSAC采样点数
            num_iterations: RANSAC迭代次数
            percentile: 地面点百分比阈值（默认95%）
            
        Returns:
            提取的地面点云
        """
        print("\n=== 开始提取地面 (z < 0) ===")
        points = np.asarray(pcd.points)
        
        # 只处理z < 0的点
        ground_mask = points[:, 2] < 0
        ground_points = points[ground_mask]
        
        if len(ground_points) == 0:
            print("没有找到z < 0的点，跳过地面提取")
            return None
        
        print(f"z < 0的点数: {len(ground_points)}")
        
        # 使用Open3D的RANSAC平面分割
        ground_pcd = o3d.geometry.PointCloud()
        ground_pcd.points = o3d.utility.Vector3dVector(ground_points)
        
        plane_model, inliers = ground_pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )
        
        [a, b, c, d] = plane_model
        print(f"地面平面方程: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
        print(f"地面内点数: {len(inliers)} ({len(inliers)/len(ground_points)*100:.1f}%)")
        
        # 计算地面高度范围
        ground_inlier_points = ground_points[inliers]
        ground_z_values = ground_inlier_points[:, 2]
        
        # 计算包含至少percentile%地面点的z轴范围
        z_min = np.percentile(ground_z_values, (100 - percentile) / 2)
        z_max = np.percentile(ground_z_values, percentile + (100 - percentile) / 2)
        z_mean = ground_z_values.mean()
        z_std = ground_z_values.std()
        
        print(f"地面平均高度: {z_mean:.3f} m")
        print(f"地面高度标准差: {z_std:.3f} m")
        print(f"地面{percentile}%点的z轴范围: [{z_min:.3f}, {z_max:.3f}] m")
        
        self.ground_z_range = {
            'min': float(z_min),
            'max': float(z_max),
            'mean': float(z_mean),
            'std': float(z_std)
        }
        
        return ground_pcd.select_by_index(inliers)
    
    def save_z_range_data(self):
        """保存提取的z轴范围数据到JSON文件"""
        data = {
            'ground_z_range': self.ground_z_range,
            'ceiling_z_threshold': self.ceiling_z_threshold,
            'input_pcd': self.pcd_path,
            'output_pcd': self.output_pcd_path
        }
        
        # 确保输出目录存在
        output_dir = os.path.dirname(self.output_data_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        with open(self.output_data_path, 'w') as f:
            json.dump(data, f, indent=4)
        
        print(f"\n=== Z轴范围数据已保存到: {self.output_data_path} ===")
        print(json.dumps(data, indent=4, ensure_ascii=False))
    
    def remove_below_ground(self, pcd, ground_margin=0.1):
        """
        去除地面以下的点
        
        Args:
            pcd: 输入点云
            ground_margin: 地面边距（米），去除z < ground_z_min - margin的点
            
        Returns:
            去除地面以下点后的点云
        """
        print(f"\n=== 去除地面以下的点 (z < {self.ground_z_range['min']:.3f} - {ground_margin}) ===")
        points = np.asarray(pcd.points)
        
        # 计算地面以下阈值
        ground_threshold = self.ground_z_range['min'] - ground_margin
        
        # 统计地面以下的点
        below_ground_mask = points[:, 2] < ground_threshold
        num_below_ground = np.sum(below_ground_mask)
        
        print(f"z < {ground_threshold:.3f} 的点数: {num_below_ground}")
        
        # 保留z >= ground_threshold的点
        remaining_mask = points[:, 2] >= ground_threshold
        remaining_points = points[remaining_mask]
        
        print(f"去除地面以下点后剩余点数: {len(remaining_points)} (去除了 {num_below_ground} 个点)")
        
        # 创建清理后的点云
        cleaned_pcd = o3d.geometry.PointCloud()
        cleaned_pcd.points = o3d.utility.Vector3dVector(remaining_points)
        
        # 如果原点云有颜色，也保留对应的颜色
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            cleaned_pcd.colors = o3d.utility.Vector3dVector(colors[remaining_mask])
        
        return cleaned_pcd
    
    def process(self, ceiling_z_threshold=0.8, distance_threshold=0.05, ransac_n=3, 
                num_iterations=1000, ground_percentile=95, ground_margin=0.1):
        """
        完整处理流程
        
        Args:
            ceiling_z_threshold: 屋顶高度阈值（米）
            distance_threshold: RANSAC距离阈值（米）
            ransac_n: RANSAC采样点数
            num_iterations: RANSAC迭代次数
            ground_percentile: 地面点百分比阈值
            ground_margin: 地面边距（米）
            
        Returns:
            处理是否成功
        """
        # 1. 加载PCD文件
        if not self.load_pcd():
            return False
        
        # 2. 去除屋顶
        pcd_no_ceiling = self.remove_ceiling(ceiling_z_threshold=ceiling_z_threshold)
        
        # 3. 提取地面并计算z轴范围
        ground_pcd = self.extract_ground(
            pcd_no_ceiling,
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations,
            percentile=ground_percentile
        )
        
        # 4. 去除地面以下的点
        cleaned_pcd = self.remove_below_ground(pcd_no_ceiling, ground_margin=ground_margin)
        
        # 5. 保存清理后的点云
        o3d.io.write_point_cloud(self.output_pcd_path, cleaned_pcd)
        print(f"\n清理后的点云已保存到: {self.output_pcd_path}")
        
        # 6. 保存z轴范围数据
        self.save_z_range_data()
        
        print("\n=== 处理完成 ===")
        return True


def main():
    parser = argparse.ArgumentParser(description='提取点云中的地面和屋顶高度信息，并清理点云')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='输入PCD文件路径')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='输出JSON数据文件路径（默认：data/z_range_data.json）')
    parser.add_argument('--output-pcd', type=str, default=None,
                        help='输出清理后的PCD文件路径（默认：输入文件名_cleaned.pcd）')
    parser.add_argument('--ceiling-threshold', '-c', type=float, default=0.8,
                        help='屋顶高度阈值，单位：米（默认：0.8）')
    parser.add_argument('--distance-threshold', '-d', type=float, default=0.05,
                        help='RANSAC距离阈值，单位：米（默认：0.05）')
    parser.add_argument('--ransac-n', '-n', type=int, default=3,
                        help='RANSAC采样点数（默认：3）')
    parser.add_argument('--num-iterations', '-t', type=int, default=1000,
                        help='RANSAC迭代次数（默认：1000）')
    parser.add_argument('--ground-percentile', '-p', type=float, default=95,
                        help='地面点百分比阈值（默认：95）')
    parser.add_argument('--ground-margin', '-m', type=float, default=0.1,
                        help='地面边距，单位：米（默认：0.1）')
    
    args = parser.parse_args()
    
    # 设置默认输出路径
    if args.output is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        package_dir = os.path.dirname(script_dir)
        args.output = os.path.join(package_dir, 'data', 'z_range_data.json')
    
    # 创建提取器并处理
    extractor = GroundCeilingExtractor(args.input, args.output, args.output_pcd)
    success = extractor.process(
        ceiling_z_threshold=args.ceiling_threshold,
        distance_threshold=args.distance_threshold,
        ransac_n=args.ransac_n,
        num_iterations=args.num_iterations,
        ground_percentile=args.ground_percentile,
        ground_margin=args.ground_margin
    )
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

