#!/usr/bin/env python3
"""
简单的 Waypoint 录制器 - 记录机器人到达的位置。

最可靠的使用方法：
1. 在 RViz 中用 Nav2 Goal 让机器人导航到目标
2. 机器人到达后，按 'r' 记录该位置
3. 重复步骤 1-2
4. 按 's' 保存所有 waypoints

这样可以记录机器人实际到达的位置，而不是点击的位置。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import yaml
import os
import sys
import math
import threading
import select
from datetime import datetime

try:
    import termios
    import tty
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False


class SimpleWaypointRecorder(Node):
    def __init__(self, output_file: str):
        super().__init__('simple_waypoint_recorder')
        
        self.output_file = output_file
        self.waypoints = []
        self.current_odom_pose = None
        self.current_amcl_pose = None
        self.frame_id = 'map'
        self.recording = True
        
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # 订阅 AMCL 位姿（更准确的 map 坐标系位置）
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10
        )
        
        self.print_banner()
        
    def print_banner(self):
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('   简单 Waypoint 录制器')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'输出文件: {self.output_file}')
        self.get_logger().info('')
        self.get_logger().info('使用方法:')
        self.get_logger().info('  1. 在 RViz 中用 Nav2 Goal 发送目标')
        self.get_logger().info('  2. 等待机器人到达目标位置')
        self.get_logger().info('  3. 按 "r" 记录当前位置')
        self.get_logger().info('  4. 重复 1-3 直到完成')
        self.get_logger().info('  5. 按 "s" 保存')
        self.get_logger().info('')
        self.get_logger().info('快捷键:')
        self.get_logger().info('  r = 记录当前位置 (推荐!)')
        self.get_logger().info('  s = 保存文件')
        self.get_logger().info('  l = 显示已记录的点')
        self.get_logger().info('  u = 撤销上一个点')
        self.get_logger().info('  c = 清空所有点')
        self.get_logger().info('  q = 退出')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        
    def odom_callback(self, msg: Odometry):
        self.current_odom_pose = msg.pose.pose
        
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.current_amcl_pose = msg.pose.pose
        self.frame_id = msg.header.frame_id or 'map'
        
    def get_yaw_from_quaternion(self, orientation):
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def record_current_position(self):
        """记录当前位置（优先使用 AMCL 位姿）"""
        pose = self.current_amcl_pose or self.current_odom_pose
        source = 'AMCL' if self.current_amcl_pose else 'Odom'
        
        if pose is None:
            self.get_logger().warn('尚未收到位置信息，请稍候...')
            return
        
        x = pose.position.x
        y = pose.position.y
        yaw = self.get_yaw_from_quaternion(pose.orientation)
        yaw_deg = math.degrees(yaw)
        
        waypoint_num = len(self.waypoints) + 1
        waypoint = {
            'name': f'point_{waypoint_num}',
            'pose': {
                'x': round(x, 3),
                'y': round(y, 3),
                'yaw_deg': round(yaw_deg, 1)
            }
        }
        
        self.waypoints.append(waypoint)
        self.get_logger().info(
            f'✓ [{waypoint_num}] 已记录: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}° ({source})'
        )
        
    def undo_last(self):
        if self.waypoints:
            removed = self.waypoints.pop()
            self.get_logger().info(f'已撤销: {removed["name"]}')
        else:
            self.get_logger().info('没有可撤销的点')
            
    def list_waypoints(self):
        if not self.waypoints:
            self.get_logger().info('尚未记录任何点')
            return
        
        self.get_logger().info(f'已记录 {len(self.waypoints)} 个点:')
        for i, wp in enumerate(self.waypoints, 1):
            p = wp['pose']
            self.get_logger().info(f'  [{i}] x={p["x"]}, y={p["y"]}, yaw={p["yaw_deg"]}°')
            
    def clear_waypoints(self):
        count = len(self.waypoints)
        self.waypoints = []
        self.get_logger().info(f'已清空 {count} 个点')
        
    def save_waypoints(self):
        if not self.waypoints:
            self.get_logger().warn('没有点可保存')
            return False
        
        data = {
            'frame_id': self.frame_id,
            'count': 0,
            'waypoints': self.waypoints,
            '_metadata': {
                'recorded_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'total_points': len(self.waypoints)
            }
        }
        
        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        
        with open(self.output_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        self.get_logger().info(f'✓ 已保存 {len(self.waypoints)} 个点到 {self.output_file}')
        return True


def keyboard_listener(node: SimpleWaypointRecorder):
    if not HAS_TERMIOS or not sys.stdin.isatty():
        return
    
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        tty.setcbreak(sys.stdin.fileno())
        
        while rclpy.ok() and node.recording:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1)
                
                if ch == 'r':
                    node.record_current_position()
                elif ch == 's':
                    node.save_waypoints()
                elif ch == 'l':
                    node.list_waypoints()
                elif ch == 'u':
                    node.undo_last()
                elif ch == 'c':
                    node.clear_waypoints()
                elif ch == 'q' or ch == '\x03':
                    node.recording = False
                    break
    except Exception as e:
        print(f"键盘监听错误: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='简单 Waypoint 录制器')
    parser.add_argument('-o', '--output', default='config/recorded_waypoints.yaml')
    args = parser.parse_args()
    
    rclpy.init()
    node = SimpleWaypointRecorder(args.output)
    
    if HAS_TERMIOS and sys.stdin.isatty():
        kb_thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
        kb_thread.start()
    
    try:
        while rclpy.ok() and node.recording:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node.waypoints:
            print('\n')
            node.get_logger().info(f'共记录 {len(node.waypoints)} 个点')
            if sys.stdin.isatty():
                try:
                    if HAS_TERMIOS:
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, 
                                         termios.tcgetattr(sys.stdin))
                except:
                    pass
                response = input('是否保存? (Y/n): ').strip().lower()
                if response != 'n':
                    node.save_waypoints()
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
