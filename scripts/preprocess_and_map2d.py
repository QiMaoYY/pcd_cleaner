#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
步骤1+2 合并版：
从 <地图根目录>/<地图名>/pointcloud_original.pcd 读取原始点云
1) 去除屋顶（z > ceiling）
2) 提取地面（RANSAC），计算 ground_z_range
3) 去除地面以下点（z < ground_z_min - 0.1）
4) 输出：
   - <地图根目录>/<地图名>/range_z.json
   - <地图根目录>/<地图名>/pointcloud_tmp.pcd
5) 生成 2D 栅格地图（纯Python，直接写 pgm+yaml）：
   - <地图根目录>/<地图名>/map2d_init.pgm
   - <地图根目录>/<地图名>/map2d_init.yaml（image 为绝对路径）
"""

import argparse
import json
import os
import sys

import numpy as np
import open3d as o3d
from pcd_to_pgm import pcd_to_map_files


class PreprocessAndMap2D:
    def __init__(self, map_root: str, map_name: str, ceiling: float):
        self.map_root = os.path.abspath(os.path.expanduser(map_root))
        self.map_name = map_name
        self.map_dir = os.path.join(self.map_root, self.map_name)
        self.ceiling = float(ceiling)

        self.input_pcd = os.path.join(self.map_dir, "pointcloud_original.pcd")
        self.tmp_pcd = os.path.join(self.map_dir, "pointcloud_tmp.pcd")
        self.range_json = os.path.join(self.map_dir, "range_z.json")

        self.ground_z_range = None

    def ensure_dir(self):
        os.makedirs(self.map_dir, exist_ok=True)

    def load_pcd(self) -> o3d.geometry.PointCloud:
        if not os.path.exists(self.input_pcd):
            raise FileNotFoundError(f"未找到输入点云：{self.input_pcd}")
        pcd = o3d.io.read_point_cloud(self.input_pcd)
        if pcd.is_empty():
            raise RuntimeError(f"点云为空：{self.input_pcd}")
        return pcd

    @staticmethod
    def _pcd_from_mask(pcd: o3d.geometry.PointCloud, keep_mask: np.ndarray) -> o3d.geometry.PointCloud:
        points = np.asarray(pcd.points)
        out = o3d.geometry.PointCloud()
        out.points = o3d.utility.Vector3dVector(points[keep_mask])
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            out.colors = o3d.utility.Vector3dVector(colors[keep_mask])
        return out

    def remove_ceiling(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        pts = np.asarray(pcd.points)
        keep = pts[:, 2] <= self.ceiling
        return self._pcd_from_mask(pcd, keep)

    def extract_ground_range(self, pcd: o3d.geometry.PointCloud, distance_threshold=0.05, ransac_n=3, num_iterations=1000, percentile=95):
        pts = np.asarray(pcd.points)
        neg_mask = pts[:, 2] < 0
        neg_pts = pts[neg_mask]
        if neg_pts.shape[0] == 0:
            raise RuntimeError("未找到 z<0 的点，无法提取地面")

        neg_pcd = o3d.geometry.PointCloud()
        neg_pcd.points = o3d.utility.Vector3dVector(neg_pts)
        _, inliers = neg_pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations,
        )
        if len(inliers) == 0:
            raise RuntimeError("地面RANSAC未找到内点")

        ground_z = neg_pts[inliers][:, 2]
        z_min = float(np.percentile(ground_z, (100 - percentile) / 2))
        z_max = float(np.percentile(ground_z, percentile + (100 - percentile) / 2))
        z_mean = float(ground_z.mean())
        z_std = float(ground_z.std())
        self.ground_z_range = {"min": z_min, "max": z_max, "mean": z_mean, "std": z_std}

    def remove_below_ground(self, pcd: o3d.geometry.PointCloud, margin=0.1) -> o3d.geometry.PointCloud:
        if not self.ground_z_range:
            raise RuntimeError("ground_z_range 未生成")
        thr = self.ground_z_range["min"] - float(margin)
        pts = np.asarray(pcd.points)
        keep = pts[:, 2] >= thr
        return self._pcd_from_mask(pcd, keep)

    def save_outputs(self, tmp_pcd: o3d.geometry.PointCloud):
        o3d.io.write_point_cloud(self.tmp_pcd, tmp_pcd)
        data = {
            "ground_z_range": self.ground_z_range,
            "ceiling_z_threshold": self.ceiling,
            "input_pcd": self.input_pcd,
            "output_pcd": self.tmp_pcd,
        }
        with open(self.range_json, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def _generate_map2d_init(self, tmp_pcd: o3d.geometry.PointCloud):
        """
        参考 pcd2pgm：
        - 直通滤波范围：thre_z_min = ground_z_max + 0.1, thre_z_max = ceiling
        - 输出：map2d_init.pgm / map2d_init.yaml
        """
        thre_z_min = float(self.ground_z_range["max"]) + 0.1
        thre_z_max = float(self.ceiling)
        out_prefix = os.path.join(self.map_dir, "map2d_init")
        pcd_to_map_files(
            pcd=tmp_pcd,
            out_prefix=out_prefix,
            thre_z_min=thre_z_min,
            thre_z_max=thre_z_max,
            flag_pass_through=0,
            resolution=0.05,
            chunk_size=2_000_000,
        )

    def run(self) -> int:
        self.ensure_dir()

        # 点云预处理
        print("点云预处理...")
        pcd = self.load_pcd()
        pts = np.asarray(pcd.points)
        print(f"原始点数: {pts.shape[0]}")

        pcd_no_ceiling = self.remove_ceiling(pcd)
        pts2 = np.asarray(pcd_no_ceiling.points)
        print(f"去屋顶后点数: {pts2.shape[0]}")

        self.extract_ground_range(pcd_no_ceiling)
        print(f"地面z范围: [{self.ground_z_range['min']:.3f}, {self.ground_z_range['max']:.3f}]")

        tmp = self.remove_below_ground(pcd_no_ceiling, margin=0.1)
        pts3 = np.asarray(tmp.points)
        print(f"去地面以下后点数: {pts3.shape[0]}")

        self.save_outputs(tmp)

        # 生成 2D 地图（纯Python，直接输出 pgm+yaml，不依赖 map_saver）
        print("生成2D栅格地图...")
        self._generate_map2d_init(tmp)
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="点云预处理 + 生成2D栅格地图（合并版）")
    parser.add_argument("--map-root", "-r", default="/media/data/slam_ws/src/kuavo_slam/maps", help="地图根目录")
    parser.add_argument("--map-name", "-n", default="map_demo", help="地图名称（子目录名）")
    parser.add_argument("--ceiling", "-c", type=float, default=0.8, help="屋顶高度阈值（米）")
    args = parser.parse_args()

    app = PreprocessAndMap2D(args.map_root, args.map_name, args.ceiling)
    return app.run()


if __name__ == "__main__":
    sys.exit(main())


