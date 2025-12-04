#!/usr/bin/env python3
"""将 /odom 话题的里程计数据转换为 TF 变换。

Gazebo 的 ros_gz_bridge 发布的 /odom 话题不会自动发布 odom → base_footprint 的 TF。
这个节点订阅 /odom 并发布相应的 TF 变换。
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        # 参数
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, 
            odom_topic, 
            self.odom_callback, 
            10
        )
        
        self.get_logger().info(
            f'odom_to_tf 节点已启动: {odom_topic} → TF ({self.odom_frame} → {self.base_frame})'
        )

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        
        # 使用消息中的 header（包含时间戳）
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.odom_frame
        
        # child_frame_id
        t.child_frame_id = msg.child_frame_id if msg.child_frame_id else self.base_frame
        
        # 从 Odometry 消息中提取位姿
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
