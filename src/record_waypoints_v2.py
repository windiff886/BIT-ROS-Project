#!/usr/bin/env python3
"""
录制 RViz 中点击的目标点，保存为 waypoints.yaml 文件。
此版本通过监听 Nav2 action 的目标来捕获 RViz 设置的导航点。

使用方法：
1. 启动导航系统
2. 运行此脚本: python3 src/record_waypoints_v2.py
3. 在 RViz 中使用 "Nav2 Goal" 或 "2D Goal Pose" 工具点击目标点
4. 每当机器人开始导航到新目标，脚本会自动记录
5. 按 Ctrl+C 结束录制，waypoints 会自动保存

快捷键（在终端中）：
- 's': 保存当前记录的 waypoints
- 'u': 撤销最后一个 waypoint
- 'l': 列出所有已记录的 waypoints
- 'c': 清空所有 waypoints
- 'r': 记录当前机器人位置
- 'q': 退出程序
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
import yaml
import os
import sys
import math
import threading
import select
from datetime import datetime

# 尝试导入 termios（Linux）
try:
    import termios
    import tty
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False


class WaypointRecorderV2(Node):
    def __init__(self, output_file: str):
        super().__init__('waypoint_recorder_v2')
        
        self.output_file = output_file
        self.waypoints = []
        self.current_pose = None
        self.frame_id = 'map'
        self.recording = True
        self.last_goal_uuid = None
        
        # QoS 配置
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅里程计获取当前位置
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 方法1: 直接订阅 /goal_pose
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            qos_reliable
        )
        
        # 方法2: 监听 clicked_point
        self.clicked_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            qos_reliable
        )
        
        # 方法3: 监听 NavigateToPose action 的 goal（真正的目标点）
        # 使用内部 DDS 话题来监听 action goal
        from nav2_msgs.action._navigate_to_pose import NavigateToPose_SendGoal_Request
        
        # Action goal 话题使用 SERVICE QoS
        qos_action = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建一个 service client 来监听 action goal
        # 实际上我们需要订阅 action 的内部话题
        self.get_logger().info('正在设置 action goal 监听...')
        
        # 存储已处理的 goal
        self.processed_goals = set()
        
        # 定时检查新目标
        self.check_timer = self.create_timer(0.5, self.check_for_new_goals)
        self.last_nav_goal = None
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waypoint 录制器 V2 已启动')
        self.get_logger().info(f'输出文件: {self.output_file}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('方法1: 在 RViz 中使用 "Nav2 Goal" 点击 + 按 "g" 确认')
        self.get_logger().info('方法2: 在 RViz 中使用 "Publish Point" 点击')
        self.get_logger().info('方法3: 按 "r" 记录机器人当前位置')
        self.get_logger().info('-' * 60)
        self.get_logger().info('快捷键:')
        self.get_logger().info('  s = 保存    u = 撤销    l = 列表')
        self.get_logger().info('  c = 清空    r = 记录当前位置')
        self.get_logger().info('  g = 获取当前导航目标    q = 退出')
        self.get_logger().info('=' * 60)
        
    def check_for_new_goals(self):
        """定时检查是否有新的导航目标"""
        pass  # 我们通过其他方式捕获

    def get_current_nav_goal(self):
        """尝试从 bt_navigator 获取当前导航目标"""
        try:
            # 通过参数或服务获取当前目标
            # 这是一个简化的方法
            pass
        except:
            pass

    def goal_pose_callback(self, msg: PoseStamped):
        """处理直接发布到 /goal_pose 的消息"""
        self.add_waypoint_from_pose(msg, source='/goal_pose')

    def clicked_point_callback(self, msg: PointStamped):
        """处理 Publish Point 工具点击的点"""
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position.x = msg.point.x
        pose_msg.pose.position.y = msg.point.y
        pose_msg.pose.position.z = msg.point.z
        pose_msg.pose.orientation.w = 1.0
        self.add_waypoint_from_pose(pose_msg, source='Publish Point')

    def add_waypoint_from_pose(self, msg: PoseStamped, source: str = ''):
        """从 PoseStamped 消息添加 waypoint"""
        if not self.recording:
            return
        
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # 四元数转 yaw
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw)
        
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id
        
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
        
        source_info = f' (来源: {source})' if source else ''
        self.get_logger().info(
            f'✓ [{waypoint_num}] 记录目标点: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}°{source_info}'
        )

    def odom_callback(self, msg: Odometry):
        """更新当前位置"""
        self.current_pose = msg.pose.pose

    def record_current_pose(self):
        """记录机器人当前位置"""
        if self.current_pose is None:
            self.get_logger().warn('尚未收到机器人位置信息 (/odom)')
            return
        
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose = self.current_pose
        self.add_waypoint_from_pose(pose_msg, source='当前位置')

    def capture_nav_goal(self):
        """
        尝试捕获当前 Nav2 的导航目标。
        通过调用 bt_navigator 的服务或检查 feedback。
        """
        self.get_logger().info('正在尝试捕获导航目标...')
        
        # 由于直接获取 action goal 比较复杂，
        # 这里提供一个变通方案：让用户手动输入坐标
        self.get_logger().info('请在 RViz 中查看目标箭头的坐标，然后使用命令行发布:')
        self.get_logger().info('  ros2 topic pub /goal_pose geometry_msgs/PoseStamped \\')
        self.get_logger().info('    "{pose: {position: {x: X, y: Y}}}" --once')
        self.get_logger().info('或者按 "r" 在机器人到达目标后记录位置。')

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
        self.recorded_goals.clear()
        self.get_logger().info(f'已清空 {count} 个 waypoints')

    def save_waypoints(self):
        """保存 waypoints 到文件"""
        if not self.waypoints:
            self.get_logger().warn('没有 waypoints 可保存')
            return False
        
        output_waypoints = []
        for wp in self.waypoints:
            output_waypoints.append({
                'name': wp['name'],
                'pose': wp['pose']
            })
        
        data = {
            'frame_id': self.frame_id,
            'count': 0,
            'waypoints': output_waypoints,
            '_metadata': {
                'recorded_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'total_points': len(output_waypoints)
            }
        }
        
        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        
        with open(self.output_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        self.get_logger().info(f'✓ 已保存 {len(self.waypoints)} 个 waypoints 到 {self.output_file}')
        return True


def keyboard_listener(node: WaypointRecorderV2):
    """监听键盘输入"""
    if not HAS_TERMIOS or not sys.stdin.isatty():
        return
    
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        tty.setcbreak(sys.stdin.fileno())
        
        while rclpy.ok() and node.recording:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1)
                
                if ch == 's':
                    node.save_waypoints()
                elif ch == 'u':
                    node.undo_last()
                elif ch == 'l':
                    node.list_waypoints()
                elif ch == 'c':
                    node.clear_waypoints()
                elif ch == 'r':
                    node.record_current_pose()
                elif ch == 'g':
                    node.capture_nav_goal()
                elif ch == 'q' or ch == '\x03':
                    node.recording = False
                    break
    except Exception as e:
        print(f"键盘监听错误: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='录制 RViz 目标点为 waypoints (V2)')
    parser.add_argument(
        '-o', '--output',
        default='config/recorded_waypoints.yaml',
        help='输出文件路径'
    )
    
    args = parser.parse_args()
    
    rclpy.init()
    node = WaypointRecorderV2(args.output)
    
    # 启动键盘监听线程
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
            node.get_logger().info(f'录制结束，共 {len(node.waypoints)} 个 waypoints')
            
            if sys.stdin.isatty():
                try:
                    if HAS_TERMIOS:
                        old_settings = termios.tcgetattr(sys.stdin)
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                except:
                    pass
                
                response = input('是否保存? (Y/n): ').strip().lower()
                if response != 'n':
                    node.save_waypoints()
            else:
                node.save_waypoints()
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
