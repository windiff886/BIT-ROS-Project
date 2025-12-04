#!/usr/bin/env python3
"""转换激光扫描的 frame_id。

Gazebo 发布的 /scan 话题 frame_id 是 'tiago/base_footprint/base_laser'，
但 Nav2 需要一个在 TF 树中存在的 frame。
这个节点将激光数据重新发布，使用正确的 frame_id。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class LaserFrameRemapper(Node):
    def __init__(self):
        super().__init__('laser_frame_remapper')
        
        # 参数
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_remapped')
        self.declare_parameter('target_frame', 'base_laser_link')
        
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        
        # QoS 设置 - 使用 RELIABLE 以兼容 Nav2
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 订阅使用 BEST_EFFORT 以兼容 Gazebo bridge
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.pub = self.create_publisher(LaserScan, output_topic, qos_pub)
        self.sub = self.create_subscription(LaserScan, input_topic, self.scan_callback, qos_sub)
        
        self.get_logger().info(
            f'激光 frame 转换器已启动: {input_topic} → {output_topic} (frame: {self.target_frame})'
        )

    def scan_callback(self, msg: LaserScan):
        # 创建新的消息，只修改 frame_id，保留原始时间戳
        # 原始时间戳来自 Gazebo，与 TF 时间同步
        new_msg = LaserScan()
        new_msg.header.stamp = msg.header.stamp  # 保留原始时间戳
        new_msg.header.frame_id = self.target_frame
        
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        new_msg.ranges = msg.ranges
        new_msg.intensities = msg.intensities
        
        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserFrameRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
