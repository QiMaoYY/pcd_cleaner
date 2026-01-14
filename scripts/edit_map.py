#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
步骤三：地图编辑（Python版）

流程：
1) 复制 map2d_init.pgm -> map2d.pgm
2) 调用 kolourpaint 打开 map2d.pgm，等待用户保存并关闭
3) 基于 map2d_init.yaml 生成 map2d.yaml（image 指向 map2d.pgm）
4) 从 map2d.pgm 生成 sketch.png
"""

import argparse
import os
import shutil
import subprocess
import sys

import cv2
import yaml


def _abs(p: str) -> str:
    return os.path.abspath(os.path.expanduser(p))


def _read_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _write_yaml(path: str, data: dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="编辑栅格地图，并生成新的yaml与png预览图")
    parser.add_argument("--map-name", default="map_demo", help="地图文件夹名（默认map_demo）")
    parser.add_argument("--base-dir", default="/media/data/slam_ws/src/kuavo_slam/maps", help="地图根目录")
    parser.add_argument("--map-dir", default="", help="直接指定地图目录（优先级高于base-dir+map-name）")
    parser.add_argument("--editor", default="kolourpaint", help="地图编辑器命令（默认kolourpaint）")
    parser.add_argument("--init-pgm", default="map2d_init.pgm", help="原始pgm文件名（默认map2d_init.pgm）")
    parser.add_argument("--init-yaml", default="map2d_init.yaml", help="原始yaml文件名（默认map2d_init.yaml）")
    parser.add_argument("--edited-pgm", default="map2d.pgm", help="编辑pgm文件名（默认map2d.pgm）")
    parser.add_argument("--edited-yaml", default="map2d.yaml", help="输出yaml文件名（默认map2d.yaml）")
    parser.add_argument("--sketch", default="sketch.png", help="输出png文件名（默认sketch.png）")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="继续编辑：直接打开已存在的编辑地图（不会从init复制）。若编辑地图不存在则仍会从init复制。",
    )
    args = parser.parse_args()

    map_dir = _abs(args.map_dir) if args.map_dir else _abs(os.path.join(args.base_dir, args.map_name))
    init_pgm = os.path.join(map_dir, args.init_pgm)
    init_yaml = os.path.join(map_dir, args.init_yaml)
    edited_pgm = os.path.join(map_dir, args.edited_pgm)
    edited_yaml = os.path.join(map_dir, args.edited_yaml)
    sketch_png = os.path.join(map_dir, args.sketch)

    if not os.path.isdir(map_dir):
        print(f"错误：地图目录不存在：{map_dir}")
        return 1
    if not os.path.exists(init_pgm):
        print(f"错误：原始地图不存在：{init_pgm}")
        return 1
    if not os.path.exists(init_yaml):
        print(f"错误：原始yaml不存在：{init_yaml}")
        return 1

    # 1) 复制 init -> edited（或继续编辑）
    if args.resume and os.path.exists(edited_pgm):
        print(f"继续编辑：{edited_pgm}")
    else:
        shutil.copyfile(init_pgm, edited_pgm)
        print(f"已准备编辑地图：{edited_pgm}")

    # 2) 启动编辑器并等待退出
    if shutil.which(args.editor) is None:
        print(f"错误：未找到编辑器命令：{args.editor}")
        print("请安装 kolourpaint 或用 --editor 指定其他编辑器命令")
        return 1

    print(f"打开编辑器：{args.editor} {edited_pgm}")
    subprocess.run([args.editor, edited_pgm], check=False)

    # 3) 生成新的 yaml：image 指向编辑后的 pgm（绝对路径）
    data = _read_yaml(init_yaml)
    data["image"] = edited_pgm
    _write_yaml(edited_yaml, data)
    print(f"已生成：{edited_yaml}")

    # 4) 生成 sketch.png
    img = cv2.imread(edited_pgm, cv2.IMREAD_UNCHANGED)
    if img is None:
        print(f"错误：无法读取编辑后的pgm：{edited_pgm}")
        return 1
    cv2.imwrite(sketch_png, img)
    print(f"已生成：{sketch_png}")

    return 0


if __name__ == "__main__":
    sys.exit(main())


