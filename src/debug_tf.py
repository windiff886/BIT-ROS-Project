#!/usr/bin/env python3
"""诊断 TF 变换链问题的工具脚本"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time


class TFDebugger(Node):
    def __init__(self):
        super().__init__('tf_debugger')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.laser_frame = None
        self.odom_received = False
        
        # 订阅激光和里程计
        self.create_subscription(LaserScan, '/scan', self.laser_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # 定时器检查 TF
        self.create_timer(2.0, self.check_tf)
        
        self.get_logger().info('TF 诊断工具已启动...')

    def laser_cb(self, msg):
        if self.laser_frame != msg.header.frame_id:
            self.laser_frame = msg.header.frame_id
            self.get_logger().info(f'激光传感器 frame_id: {self.laser_frame}')

    def odom_cb(self, msg):
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(f'里程计 header.frame_id: {msg.header.frame_id}')
            self.get_logger().info(f'里程计 child_frame_id: {msg.child_frame_id}')

    def check_tf(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('检查 TF 变换链...')
        
        # 获取所有 frame
        frames = self.tf_buffer.all_frames_as_string()
        self.get_logger().info(f'当前 TF 树:\n{frames}')
        
        # 检查关键变换
        transforms_to_check = [
            ('map', 'odom'),
            ('odom', 'base_footprint'),
            ('odom', 'base_link'),
            ('base_footprint', 'base_link'),
            ('base_link', 'base_laser_link'),
        ]
        
        if self.laser_frame:
            transforms_to_check.append(('base_link', self.laser_frame))
            transforms_to_check.append(('map', self.laser_frame))
        
        for parent, child in transforms_to_check:
            try:
                trans = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                self.get_logger().info(f'✓ {parent} → {child}: OK')
            except Exception as e:
                self.get_logger().warn(f'✗ {parent} → {child}: {str(e)[:80]}')


def main():
    rclpy.init()
    node = TFDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
