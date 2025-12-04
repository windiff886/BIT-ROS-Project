#!/usr/bin/env python3
"""Send waypoints from a YAML config to Nav2's follow_waypoints action."""

import argparse
import math
from pathlib import Path
from typing import List, Tuple

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import FollowWaypoints
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


def yaw_to_quaternion(z_yaw: float) -> Tuple[float, float]:
    """Return (z, w) quaternion components for a planar yaw."""
    return math.sin(z_yaw / 2.0), math.cos(z_yaw / 2.0)


def make_pose(node: Node, x: float, y: float, yaw: float, frame: str = "map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class WaypointClient(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_follower_client")
        self._client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        
        # 创建 MarkerArray 发布器用于可视化
        self._marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", 10)
        
    def publish_waypoint_markers(self, poses: List[PoseStamped]):
        """发布 waypoint 标记到 RViz"""
        marker_array = MarkerArray()
        
        # 清除旧标记
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self._marker_pub.publish(marker_array)
        
        marker_array = MarkerArray()
        
        # 创建连接线（路径）
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "waypoint_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05  # 线宽
        line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # 绿色
        line_marker.pose.orientation.w = 1.0
        
        for pose in poses:
            p = Point()
            p.x = pose.pose.position.x
            p.y = pose.pose.position.y
            p.z = 0.1
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # 创建每个 waypoint 的标记
        for idx, pose in enumerate(poses):
            # 球体标记
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = "waypoint_spheres"
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = pose.pose.position.x
            sphere.pose.position.y = pose.pose.position.y
            sphere.pose.position.z = 0.2
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.3
            sphere.scale.y = 0.3
            sphere.scale.z = 0.3
            # 起点蓝色，终点红色，中间黄色
            if idx == 0:
                sphere.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)  # 蓝色
            elif idx == len(poses) - 1:
                sphere.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # 红色
            else:
                sphere.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # 黄色
            marker_array.markers.append(sphere)
            
            # 文字标记（显示序号）
            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "waypoint_labels"
            text.id = idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pose.pose.position.x
            text.pose.position.y = pose.pose.position.y
            text.pose.position.z = 0.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.3  # 文字大小
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # 白色
            text.text = f"WP{idx + 1}"
            marker_array.markers.append(text)
            
            # 箭头标记（显示朝向）
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = "waypoint_arrows"
            arrow.id = idx
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose.pose
            arrow.pose.position.z = 0.1
            arrow.scale.x = 0.4  # 箭头长度
            arrow.scale.y = 0.1  # 箭头宽度
            arrow.scale.z = 0.1
            arrow.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # 橙色
            marker_array.markers.append(arrow)
        
        # 发布标记
        self._marker_pub.publish(marker_array)
        self.get_logger().info(f"已发布 {len(poses)} 个 waypoint 标记到 /waypoint_markers")

    def send(self, poses: List[PoseStamped]):
        if not poses:
            self.get_logger().error("No poses provided.")
            return None

        self.get_logger().info("Waiting for follow_waypoints action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("follow_waypoints action server not available.")
            return None

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        self.get_logger().info(f"Sending {len(poses)} waypoints.")
        goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error("Failed to get result.")
            return None

        self.get_logger().info(f"Finished with status {goal_handle.status}")
        self.get_logger().info(f"Result: {result.result}")
        return result.result


def _load_config(node: Node, path: Path) -> List[Tuple[float, float, float, str]]:
    if not path.exists():
        node.get_logger().error(f"Config file not found: {path}")
        return []

    try:
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"Failed to read config: {exc}")
        return []

    frame_default = data.get("frame_id", "map")
    limit = data.get("count") or data.get("limit") or 0
    waypoints_cfg = data.get("waypoints") or []

    waypoints: List[Tuple[float, float, float, str]] = []
    for idx, wp in enumerate(waypoints_cfg):
        pose_cfg = wp.get("pose", {})
        x = float(pose_cfg.get("x", 0.0))
        y = float(pose_cfg.get("y", 0.0))
        yaw_rad = pose_cfg.get("yaw_rad")
        yaw_deg = pose_cfg.get("yaw_deg")
        if yaw_rad is None and yaw_deg is not None:
            yaw_rad = math.radians(float(yaw_deg))
        yaw = float(yaw_rad or 0.0)
        frame = wp.get("frame_id", frame_default)
        name = wp.get("name", f"wp_{idx + 1}")
        waypoints.append((x, y, yaw, frame))
        node.get_logger().info(
            f"Loaded {name}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad, frame={frame}"
        )

    if isinstance(limit, int) and limit > 0:
        waypoints = waypoints[:limit]
        node.get_logger().info(f"Using first {limit} waypoint(s) as requested.")

    return waypoints


def main(args=None):
    default_cfg = Path(__file__).resolve().parent.parent / "config" / "waypoints.yaml"
    parser = argparse.ArgumentParser(description="Follow Nav2 waypoints from config.")
    parser.add_argument(
        "--config",
        type=str,
        default=str(default_cfg),
        help="Path to YAML waypoint config (default: config/waypoints.yaml).",
    )
    parsed_args = parser.parse_args(args=args)

    rclpy.init(args=None)
    node = WaypointClient()

    cfg_path = Path(parsed_args.config).expanduser()
    waypoints = _load_config(node, cfg_path)
    poses = [make_pose(node, x, y, yaw, frame) for x, y, yaw, frame in waypoints]

    if poses:
        # 先发布可视化标记
        node.publish_waypoint_markers(poses)
        # 等待一下让 RViz 有时间接收
        import time
        time.sleep(0.5)
        # 发送导航目标
        node.send(poses)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
