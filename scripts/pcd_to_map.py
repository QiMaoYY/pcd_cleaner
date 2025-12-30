#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
点云转栅格地图脚本
自动读取第一步的配置，调用pcd2pgm转换，并保存地图
"""

import rospy
import rospkg
import json
import os
import sys
import argparse
import subprocess
import time
import signal
from nav_msgs.msg import OccupancyGrid


class PcdToMapConverter:
    def __init__(self, z_range_json_path, map_name="map_demo", 
                 output_base_dir="/media/data/slam_ws/src/kuavo_slam/maps"):
        """
        初始化点云转地图转换器
        
        Args:
            z_range_json_path: z轴范围数据JSON文件路径
            map_name: 地图名称（也是输出文件夹名）
            output_base_dir: 地图输出基础目录
        """
        self.z_range_json_path = z_range_json_path
        self.map_name = map_name
        self.output_base_dir = output_base_dir
        self.output_dir = os.path.join(output_base_dir, map_name)
        
        self.z_data = None
        self.pcd_file_dir = None
        self.pcd_file_name = None
        self.thre_z_min = None
        self.thre_z_max = None
        
        self.map_received = False
        self.launch_process = None
        
    def load_z_range_data(self):
        """加载z轴范围数据"""
        print(f"加载配置文件: {self.z_range_json_path}")
        
        if not os.path.exists(self.z_range_json_path):
            print(f"错误: 配置文件不存在: {self.z_range_json_path}")
            return False
        
        with open(self.z_range_json_path, 'r') as f:
            self.z_data = json.load(f)
        
        # 解析PCD文件路径
        output_pcd = self.z_data.get('output_pcd', '')
        if not output_pcd or not os.path.exists(output_pcd):
            print(f"错误: 清理后的PCD文件不存在: {output_pcd}")
            return False
        
        # 分离目录和文件名
        self.pcd_file_dir = os.path.dirname(output_pcd) + "/"
        self.pcd_file_name = os.path.basename(output_pcd).replace('.pcd', '')
        
        # 计算z轴阈值
        ground_z_max = self.z_data['ground_z_range']['max']
        ceiling_z_threshold = self.z_data['ceiling_z_threshold']
        
        self.thre_z_min = ground_z_max + 0.1
        self.thre_z_max = ceiling_z_threshold
        
        print(f"\n配置信息:")
        print(f"  PCD文件目录: {self.pcd_file_dir}")
        print(f"  PCD文件名: {self.pcd_file_name}")
        print(f"  地面高度范围: [{self.z_data['ground_z_range']['min']:.3f}, {ground_z_max:.3f}]")
        print(f"  屋顶高度阈值: {ceiling_z_threshold:.3f}")
        print(f"  转换高度范围: [{self.thre_z_min:.3f}, {self.thre_z_max:.3f}]")
        print(f"  地图输出目录: {self.output_dir}")
        
        return True
    
    def create_output_dir(self):
        """创建输出目录"""
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"\n创建输出目录: {self.output_dir}")
        else:
            print(f"\n输出目录已存在: {self.output_dir}")
    
    def generate_launch_file(self, launch_path):
        """生成launch文件"""
        launch_content = f"""<!-- Auto-generated launch file for pcd2pgm -->
<launch>
<node pkg="pcd2pgm" name="pcd2pgm" type="pcd2pgm" output="screen">
    <!-- PCD文件路径 -->
    <param name="file_directory" value="{self.pcd_file_dir}" />
    <param name="file_name" value="{self.pcd_file_name}" />
    
    <!-- 高度范围 -->
    <param name="thre_z_min" value="{self.thre_z_min:.4f}" />
    <param name="thre_z_max" value="{self.thre_z_max:.4f}" />
    <param name="flag_pass_through" value="0" />
    
    <!-- 滤波参数 -->
    <param name="thre_radius" value="0.5" />
    <param name="thres_point_count" value="10" />
    
    <!-- 地图参数 -->
    <param name="map_resolution" value="0.05" />
    <param name="map_topic_name" value="map" />
    <param name="map_frame_id" value="map" />
    
    <!-- 2D方向修正 -->
    <param name="yaw_deg" value="0.0" />
    <param name="flip_x" value="false" />
    <param name="flip_y" value="false" />
    <param name="swap_xy" value="false" />
</node>
</launch>
"""
        
        with open(launch_path, 'w') as f:
            f.write(launch_content)
        
        print(f"\n生成launch文件: {launch_path}")
    
    def map_callback(self, msg):
        """地图话题回调函数"""
        if not self.map_received:
            print(f"\n接收到地图数据:")
            print(f"  宽度: {msg.info.width}")
            print(f"  高度: {msg.info.height}")
            print(f"  分辨率: {msg.info.resolution}")
            self.map_received = True
    
    def wait_for_map(self, timeout=60):
        """等待地图数据"""
        print("\n" + "=" * 60)
        print("等待地图数据...")
        
        # 订阅地图话题
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        
        start_time = time.time()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown() and not self.map_received:
            if time.time() - start_time > timeout:
                print(f"超时: {timeout}秒内未接收到地图数据")
                return False
            rate.sleep()
        
        # 再等待1秒确保地图完全生成
        time.sleep(1)
        print("=" * 60)
        return self.map_received
    
    def save_map(self):
        """使用map_saver保存地图"""
        map_file = os.path.join(self.output_dir, self.map_name)
        
        print(f"\n保存地图到: {map_file}")
        
        # 调用map_saver
        cmd = f"rosrun map_server map_saver -f {map_file}"
        
        try:
            result = subprocess.run(cmd, shell=True, check=True, timeout=10)
            print(f"\n地图保存成功!")
            print(f"  {map_file}.pgm")
            print(f"  {map_file}.yaml")
            return True
        except subprocess.CalledProcessError as e:
            print(f"保存地图失败: {e}")
            return False
        except subprocess.TimeoutExpired:
            print("保存地图超时")
            return False
    
    def cleanup(self):
        """清理资源"""
        if self.launch_process and self.launch_process.poll() is None:
            print("\n" + "=" * 60)
            print("停止pcd2pgm节点...")
            print("=" * 60)
            self.launch_process.terminate()
            try:
                self.launch_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.launch_process.kill()
    
    def convert(self):
        """执行完整的转换流程"""
        # 1. 加载配置
        if not self.load_z_range_data():
            return False
        
        # 2. 创建输出目录
        self.create_output_dir()
        
        # 3. 生成launch文件
        launch_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            f"pcd2pgm_{self.map_name}.launch"
        )
        self.generate_launch_file(launch_path)
        
        # 4. 初始化ROS节点
        try:
            rospy.init_node('pcd_to_map_converter', anonymous=True)
        except rospy.exceptions.ROSException:
            print("ROS节点已初始化")
        
        # 5. 启动pcd2pgm节点
        print(f"\n{'=' * 60}")
        print("启动pcd2pgm节点...")
        print("=" * 60 + "\n")
        self.launch_process = subprocess.Popen(
            ['roslaunch', launch_path]
        )
        
        # 6. 等待地图生成
        if not self.wait_for_map(timeout=180):
            self.cleanup()
            return False
        
        # 7. 保存地图
        success = self.save_map()
        
        # 8. 清理
        self.cleanup()
        
        # 9. 删除临时launch文件
        if os.path.exists(launch_path):
            os.remove(launch_path)
            print(f"\n删除临时launch文件: {launch_path}")
        
        if success:
            print(f"\n=== 转换完成 ===")
            print(f"地图已保存到: {self.output_dir}/")
        
        return success


def main():
    parser = argparse.ArgumentParser(description='将PCD点云转换为2D栅格地图')
    parser.add_argument('--config', '-c', type=str, default=None,
                        help='z轴范围配置JSON文件路径（默认：../data/z_range_data.json）')
    parser.add_argument('--map-name', '-n', type=str, default='map_demo',
                        help='地图名称（默认：map_demo）')
    parser.add_argument('--output-dir', '-o', type=str, 
                        default='/media/data/slam_ws/src/kuavo_slam/maps',
                        help='地图输出基础目录（默认：/media/data/slam_ws/src/kuavo_slam/maps）')
    
    args = parser.parse_args()
    
    # 设置默认配置文件路径
    if args.config is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        package_dir = os.path.dirname(script_dir)
        args.config = os.path.join(package_dir, 'data', 'z_range_data.json')
    
    # 创建转换器
    converter = PcdToMapConverter(
        z_range_json_path=args.config,
        map_name=args.map_name,
        output_base_dir=args.output_dir
    )
    
    # 设置信号处理
    def signal_handler(sig, frame):
        print("\n收到中断信号，正在清理...")
        converter.cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # 执行转换
    success = converter.convert()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

