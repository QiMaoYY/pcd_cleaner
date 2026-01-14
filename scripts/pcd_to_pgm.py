#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pcd -> 2D 栅格地图（pgm + yaml），参考 src/pcd2pgm/src/pcd2pgm.cpp 的核心逻辑实现。

特点：
- 直通滤波（z 范围）
- 高性能栅格化：分块向量化
- 直接写出 <prefix>.pgm / <prefix>.yaml（无需 map_saver）
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Tuple

import cv2
import numpy as np
import open3d as o3d
import yaml


@dataclass
class MapMeta:
    resolution: float
    origin: Tuple[float, float, float]
    width: int
    height: int
    occupied_value: int = 0
    free_value: int = 254
    unknown_value: int = 205


def passthrough_z(points_xyz: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    """保留 z 在 [z_min, z_max] 范围内的点"""
    z = points_xyz[:, 2]
    return (z >= z_min) & (z <= z_max)


def rasterize_occupancy(
    points_xy: np.ndarray,
    resolution: float,
    occupied_value: int = 0,
    free_value: int = 254,
    chunk_size: int = 2_000_000,
) -> Tuple[np.ndarray, MapMeta]:
    """
    参考 pcd2pgm.cpp 的 SetMapTopicMsg：
    - origin: (x_min, y_min)
    - width/height: int((x_max-x_min)/res), int((y_max-y_min)/res)
    - 对每个点：i=(x-x_min)/res, j=(y-y_min)/res，标记占据
    - 写 pgm 时做 y 轴翻转，使 origin 对齐“左下角”
    """
    if points_xy.shape[0] == 0:
        raise ValueError("points_xy 为空，无法生成地图")

    x = points_xy[:, 0]
    y = points_xy[:, 1]

    x_min = float(np.min(x))
    x_max = float(np.max(x))
    y_min = float(np.min(y))
    y_max = float(np.max(y))

    width = int((x_max - x_min) / resolution)
    height = int((y_max - y_min) / resolution)
    width = max(1, width)
    height = max(1, height)

    img = np.full((height, width), free_value, dtype=np.uint8)

    n = points_xy.shape[0]
    n_chunks = int(math.ceil(n / chunk_size))

    for c in range(n_chunks):
        s = c * chunk_size
        e = min(n, (c + 1) * chunk_size)
        px = points_xy[s:e, 0]
        py = points_xy[s:e, 1]

        # 与 C++ int(...) 语义一致：截断到 int（对正数等同 floor）
        i = ((px - x_min) / resolution).astype(np.int32)
        j = ((py - y_min) / resolution).astype(np.int32)

        valid = (i >= 0) & (i < width) & (j >= 0) & (j < height)
        if not np.any(valid):
            continue

        ii = i[valid]
        jj = j[valid]

        # y 轴翻转：j=0（y_min）应落在图像底部（最后一行）
        row = (height - 1) - jj
        col = ii
        img[row, col] = occupied_value

    meta = MapMeta(
        resolution=float(resolution),
        origin=(x_min, y_min, 0.0),
        width=width,
        height=height,
        occupied_value=int(occupied_value),
        free_value=int(free_value),
    )
    return img, meta


def save_yaml(yaml_path: str, pgm_path: str, meta: MapMeta) -> None:
    data = {
        "image": os.path.abspath(pgm_path),
        "resolution": float(meta.resolution),
        "origin": [float(meta.origin[0]), float(meta.origin[1]), float(meta.origin[2])],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)


def pcd_to_map_files(
    pcd: o3d.geometry.PointCloud,
    out_prefix: str,
    thre_z_min: float,
    thre_z_max: float,
    resolution: float = 0.05,
    chunk_size: int = 2_000_000,
) -> Tuple[str, str]:
    """
    生成 <out_prefix>.pgm / <out_prefix>.yaml
    """
    out_pgm = out_prefix + ".pgm"
    out_yaml = out_prefix + ".yaml"
    os.makedirs(os.path.dirname(os.path.abspath(out_prefix)), exist_ok=True)

    pts = np.asarray(pcd.points)
    if pts.shape[0] == 0:
        raise ValueError("输入点云为空")

    # 1) PassThrough(z)
    print(f"直通滤波: z in [{thre_z_min:.3f}, {thre_z_max:.3f}]", flush=True)
    mask = passthrough_z(pts, thre_z_min, thre_z_max)
    pts_z = pts[mask]
    if pts_z.shape[0] == 0:
        raise ValueError("直通滤波后点云为空（请检查 thre_z_min/thre_z_max/flag_pass_through）")
    print(f"直通滤波后点数: {pts_z.shape[0]}", flush=True)

    # 2) Rasterize (XY)
    print(f"栅格化: resolution={resolution} m/pixel", flush=True)
    img, meta = rasterize_occupancy(
        pts_z[:, :2],
        resolution=float(resolution),
        occupied_value=0,
        free_value=254,
        chunk_size=int(chunk_size),
    )

    # 4) Save files
    # OpenCV 直接写 pgm（P5）
    ok = cv2.imwrite(out_pgm, img)
    if not ok:
        raise RuntimeError(f"写入失败: {out_pgm}")
    save_yaml(out_yaml, out_pgm, meta)
    return out_pgm, out_yaml


def main() -> int:
    parser = argparse.ArgumentParser(description="PCD 转 2D 栅格地图（pgm+yaml）")
    parser.add_argument("--pcd", required=True, help="输入PCD路径")
    parser.add_argument("--out-prefix", required=True, help="输出前缀（不含扩展名）")
    parser.add_argument("--thre-z-min", type=float, required=True, help="直通滤波最小高度")
    parser.add_argument("--thre-z-max", type=float, required=True, help="直通滤波最大高度")
    parser.add_argument("--resolution", type=float, default=0.05, help="地图分辨率(米/像素)，默认0.05")
    args = parser.parse_args()

    print(f"输入: {args.pcd}", flush=True)
    try:
        t0 = time.time()
        pcd = o3d.io.read_point_cloud(args.pcd)
        if pcd.is_empty():
            print("失败: 输入点云为空", flush=True)
            return 1
        pgm, yml = pcd_to_map_files(
            pcd=pcd,
            out_prefix=args.out_prefix,
            thre_z_min=args.thre_z_min,
            thre_z_max=args.thre_z_max,
            resolution=args.resolution,
        )
        print(f"已生成: {pgm}", flush=True)
        print(f"已生成: {yml}", flush=True)
        print(f"耗时: {time.time()-t0:.2f}s", flush=True)
        return 0
    except Exception as e:
        print(f"失败: {e}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())


