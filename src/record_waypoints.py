#!/usr/bin/env python3
"""
录制 RViz 中点击的目标点，保存为 waypoints.yaml 文件。

使用方法：
1. 启动导航系统
2. 运行此脚本: python3 src/record_waypoints.py
3. 在 RViz 中使用 "2D Goal Pose" 工具点击目标点
4. 机器人会导航到每个点，同时脚本会记录该点
5. 按 Ctrl+C 结束录制，waypoints 会自动保存

快捷键（在终端中）：
- 's': 保存当前记录的 waypoints
- 'u': 撤销最后一个 waypoint
- 'l': 列出所有已记录的 waypoints
- 'c': 清空所有 waypoints
- 'q': 退出程序
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
import yaml
import os
import sys
import math
import threading
import termios
import tty
from datetime import datetime


class WaypointRecorder(Node):
    def __init__(self, output_file: str):
        super().__init__('waypoint_recorder')
        
        self.output_file = output_file
        self.waypoints = []
        self.current_pose = None
        self.frame_id = 'map'
        self.recording = True
        self.received_count = 0
        
        # QoS 配置 - 兼容 RViz 的设置
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅 RViz 发送的目标点（多个 QoS 配置）
        self.goal_sub1 = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_reliable
        )
        
        # 也尝试用 best effort
        self.goal_sub2 = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_best_effort
        )
        
        # 监听 clicked_point（用 "Publish Point" 工具点击的点）
        from geometry_msgs.msg import PointStamped
        self.clicked_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            qos_reliable
        )
        
        # 订阅当前位置（用于记录机器人实际到达的位置）
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Waypoint 录制器已启动')
        self.get_logger().info(f'输出文件: {self.output_file}')
        self.get_logger().info('=' * 50)
        self.get_logger().info('方法1: 在 RViz 中使用 "2D Goal Pose" 点击')
        self.get_logger().info('方法2: 在 RViz 中使用 "Publish Point" 点击')
        self.get_logger().info('方法3: 在终端按 "r" 记录当前机器人位置')
        self.get_logger().info('方法4: 在另一个终端运行:')
        self.get_logger().info('  ros2 topic pub /goal_pose geometry_msgs/PoseStamped \\')
        self.get_logger().info('    "{pose: {position: {x: 1.0, y: 2.0}}}" --once')
        self.get_logger().info('-' * 50)
        self.get_logger().info('快捷键: s=保存, u=撤销, l=列表, c=清空, r=记录当前位置, q=退出')
        self.get_logger().info('=' * 50)
        
    def clicked_point_callback(self, msg):
        """处理点击的点（来自 Publish Point 工具）"""
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position.x = msg.point.x
        pose_msg.pose.position.y = msg.point.y
        pose_msg.pose.position.z = msg.point.z
        pose_msg.pose.orientation.w = 1.0
        self.goal_callback(pose_msg)
        self.get_logger().info('(通过 Publish Point 工具记录)')

    def goal_callback(self, msg: PoseStamped):
        """收到新的目标点时记录"""
        if not self.recording:
            return
            
        # 提取位置和朝向
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # 四元数转欧拉角（只取 yaw）
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # 计算 yaw 角度
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw)
        
        # 保存 frame_id
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id
        
        # 创建 waypoint
        waypoint_num = len(self.waypoints) + 1
        waypoint = {
            'name': f'point_{waypoint_num}',
            'pose': {
                'x': round(x, 3),
                'y': round(y, 3),
                'yaw_deg': round(yaw_deg, 1)
            },
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
        
        self.waypoints.append(waypoint)
        
        self.get_logger().info(
            f'[{waypoint_num}] 记录目标点: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}°'
        )

    def odom_callback(self, msg: Odometry):
        """更新当前位置"""
        self.current_pose = msg.pose.pose

    def undo_last(self):
        """撤销最后一个 waypoint"""
        if self.waypoints:
            removed = self.waypoints.pop()
            self.get_logger().info(f'已撤销: {removed["name"]}')
        else:
            self.get_logger().info('没有可撤销的 waypoint')

    def list_waypoints(self):
        """列出所有已记录的 waypoints"""
        if not self.waypoints:
            self.get_logger().info('尚未记录任何 waypoint')
            return
        
        self.get_logger().info(f'已记录 {len(self.waypoints)} 个 waypoints:')
        for i, wp in enumerate(self.waypoints, 1):
            pose = wp['pose']
            self.get_logger().info(
                f'  [{i}] {wp["name"]}: x={pose["x"]}, y={pose["y"]}, yaw={pose["yaw_deg"]}°'
            )

    def clear_waypoints(self):
        """清空所有 waypoints"""
        count = len(self.waypoints)
        self.waypoints = []
        self.get_logger().info(f'已清空 {count} 个 waypoints')

    def save_waypoints(self):
        """保存 waypoints 到文件"""
        if not self.waypoints:
            self.get_logger().warn('没有 waypoints 可保存')
            return False
        
        # 构建输出数据（移除 timestamp，只保留必要信息）
        output_waypoints = []
        for wp in self.waypoints:
            output_waypoints.append({
                'name': wp['name'],
                'pose': wp['pose']
            })
        
        data = {
            'frame_id': self.frame_id,
            'count': 0,  # 0 表示发送所有点
            'waypoints': output_waypoints,
            '_metadata': {
                'recorded_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'total_points': len(output_waypoints)
            }
        }
        
        # 确保目录存在
        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        
        # 写入文件
        with open(self.output_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        self.get_logger().info(f'已保存 {len(self.waypoints)} 个 waypoints 到 {self.output_file}')
        return True

    def record_current_pose(self):
        """记录机器人当前位置作为 waypoint"""
        if self.current_pose is None:
            self.get_logger().warn('尚未收到机器人位置信息')
            return
        
        # 创建一个 PoseStamped 消息并调用 goal_callback
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose = self.current_pose
        self.goal_callback(msg)


def keyboard_listener(node: WaypointRecorder):
    """监听键盘输入的线程"""
    # 保存终端设置
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        tty.setraw(sys.stdin.fileno())
        
        while rclpy.ok() and node.recording:
            # 非阻塞读取
            import select
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1)
                
                if ch == 's':
                    # 保存
                    node.save_waypoints()
                elif ch == 'u':
                    # 撤销
                    node.undo_last()
                elif ch == 'l':
                    # 列表
                    node.list_waypoints()
                elif ch == 'c':
                    # 清空
                    node.clear_waypoints()
                elif ch == 'r':
                    # 记录当前位置
                    node.record_current_pose()
                elif ch == 'q' or ch == '\x03':  # q 或 Ctrl+C
                    node.recording = False
                    break
    finally:
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='录制 RViz 目标点为 waypoints')
    parser.add_argument(
        '-o', '--output',
        default='config/recorded_waypoints.yaml',
        help='输出文件路径 (默认: config/recorded_waypoints.yaml)'
    )
    parser.add_argument(
        '--no-keyboard',
        action='store_true',
        help='禁用键盘监听（用于非交互模式）'
    )
    
    args = parser.parse_args()
    
    rclpy.init()
    node = WaypointRecorder(args.output)
    
    # 启动键盘监听线程
    kb_thread = None
    if not args.no_keyboard and sys.stdin.isatty():
        kb_thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
        kb_thread.start()
    
    try:
        while rclpy.ok() and node.recording:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出时提示保存
        if node.waypoints:
            print('\n')
            node.get_logger().info(f'录制结束，共 {len(node.waypoints)} 个 waypoints')
            
            # 在非 TTY 模式下自动保存
            if not sys.stdin.isatty():
                node.save_waypoints()
            else:
                # 恢复终端设置后询问
                try:
                    old_settings = termios.tcgetattr(sys.stdin)
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                except:
                    pass
                
                response = input('是否保存? (Y/n): ').strip().lower()
                if response != 'n':
                    node.save_waypoints()
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
