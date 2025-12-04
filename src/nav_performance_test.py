#!/usr/bin/env python3
"""
Nav2 导航性能测试脚本
自动记录：导航时间、路径长度、位置误差、成功率等指标
"""

import argparse
import math
import time
import yaml
import os
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


@dataclass
class WaypointResult:
    """单个 waypoint 的导航结果"""
    name: str
    target_x: float
    target_y: float
    target_yaw: float
    success: bool = False
    time_seconds: float = 0.0
    distance_meters: float = 0.0
    final_x: float = 0.0
    final_y: float = 0.0
    final_yaw: float = 0.0
    position_error: float = 0.0
    yaw_error: float = 0.0
    min_obstacle_dist: float = float('inf')
    avg_velocity: float = 0.0
    recoveries: int = 0


@dataclass
class TestReport:
    """完整测试报告"""
    test_name: str = ''
    start_time: str = ''
    end_time: str = ''
    config_file: str = ''
    nav2_params: str = ''
    
    # 汇总指标
    total_waypoints: int = 0
    successful: int = 0
    failed: int = 0
    success_rate: float = 0.0
    
    total_time: float = 0.0
    total_distance: float = 0.0
    avg_velocity: float = 0.0
    
    avg_position_error: float = 0.0
    max_position_error: float = 0.0
    avg_yaw_error: float = 0.0
    
    total_recoveries: int = 0
    
    results: List[WaypointResult] = field(default_factory=list)


class NavPerformanceTester(Node):
    def __init__(self, config_file: str, output_dir: str, test_name: str = '', waypoint_timeout: float = 120.0):
        super().__init__('nav_performance_tester')
        
        self.config_file = config_file
        self.output_dir = output_dir
        self.test_name = test_name or datetime.now().strftime('%Y%m%d_%H%M%S')
        self.waypoint_timeout = waypoint_timeout  # 每个 waypoint 的超时时间
        
        self.report = TestReport(
            test_name=self.test_name,
            config_file=config_file
        )
        
        # 状态跟踪
        self.current_pose = None
        self.current_odom = None
        self.path_distance = 0.0
        self.last_odom_pose = None
        self.velocity_samples = []
        
        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 订阅
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, qos)
        
        # Marker 发布
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        
        # 加载 waypoints
        self.waypoints = self.load_waypoints()
        self.frame_id = 'map'
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Nav2 性能测试器已启动')
        self.get_logger().info(f'测试名称: {self.test_name}')
        self.get_logger().info(f'Waypoints: {len(self.waypoints)} 个')
        self.get_logger().info(f'每个 Waypoint 超时: {self.waypoint_timeout}s')
        self.get_logger().info('=' * 60)

    def load_waypoints(self) -> list:
        with open(self.config_file, 'r') as f:
            data = yaml.safe_load(f)
        
        self.frame_id = data.get('frame_id', 'map')
        waypoints = data.get('waypoints', [])
        
        count = data.get('count', 0)
        if count > 0:
            waypoints = waypoints[:count]
        
        return waypoints

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg.pose.pose
        
        # 累计距离
        if self.last_odom_pose is not None:
            dx = msg.pose.pose.position.x - self.last_odom_pose.position.x
            dy = msg.pose.pose.position.y - self.last_odom_pose.position.y
            self.path_distance += math.sqrt(dx*dx + dy*dy)
        
        self.last_odom_pose = msg.pose.pose
        
        # 记录速度
        vel = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.velocity_samples.append(vel)

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def get_yaw(self, orientation) -> float:
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def create_goal(self, waypoint: dict) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        pose = waypoint['pose']
        goal.pose.pose.position.x = float(pose['x'])
        goal.pose.pose.position.y = float(pose['y'])
        
        yaw = math.radians(float(pose.get('yaw_deg', 0)))
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        
        return goal

    def navigate_to_waypoint(self, waypoint: dict, index: int) -> WaypointResult:
        pose = waypoint['pose']
        result = WaypointResult(
            name=waypoint.get('name', f'point_{index}'),
            target_x=float(pose['x']),
            target_y=float(pose['y']),
            target_yaw=float(pose.get('yaw_deg', 0))
        )
        
        # 重置计数
        self.path_distance = 0.0
        self.velocity_samples = []
        start_time = time.time()
        
        self.get_logger().info(
            f'[{index}/{len(self.waypoints)}] 导航到 {result.name}: '
            f'x={result.target_x:.2f}, y={result.target_y:.2f}'
        )
        
        # 等待 action server
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server 不可用')
            result.success = False
            return result
        
        # 发送目标
        goal = self.create_goal(waypoint)
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            result.success = False
            return result
        
        # 等待结果（使用配置的超时时间）
        result_future = goal_handle.get_result_async()
        
        # 使用超时等待
        wait_start = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = time.time() - wait_start
            if elapsed > self.waypoint_timeout:
                # 超时，取消导航目标
                self.get_logger().warn(f'  ⏱ Waypoint 超时 ({self.waypoint_timeout}s)，取消导航...')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                result.success = False
                result.time_seconds = elapsed
                result.distance_meters = self.path_distance
                self.get_logger().warn(f'  ✗ 失败 (超时)')
                return result
        
        nav_result = result_future.result()
        result.time_seconds = time.time() - start_time
        result.distance_meters = self.path_distance
        
        if nav_result and nav_result.status == 4:  # SUCCEEDED
            result.success = True
            
            # 计算误差
            final_pose = self.current_pose or self.current_odom
            if final_pose:
                result.final_x = final_pose.position.x
                result.final_y = final_pose.position.y
                result.final_yaw = math.degrees(self.get_yaw(final_pose.orientation))
                
                result.position_error = math.sqrt(
                    (result.final_x - result.target_x)**2 +
                    (result.final_y - result.target_y)**2
                )
                
                yaw_diff = result.final_yaw - result.target_yaw
                while yaw_diff > 180: yaw_diff -= 360
                while yaw_diff < -180: yaw_diff += 360
                result.yaw_error = abs(yaw_diff)
            
            # 平均速度
            if self.velocity_samples:
                result.avg_velocity = sum(self.velocity_samples) / len(self.velocity_samples)
            
            self.get_logger().info(
                f'  ✓ 成功! 时间={result.time_seconds:.1f}s, '
                f'距离={result.distance_meters:.2f}m, '
                f'误差={result.position_error:.3f}m'
            )
        else:
            result.success = False
            self.get_logger().warn(f'  ✗ 失败')
        
        return result

    def publish_markers(self):
        """发布 waypoint 可视化标记"""
        marker_array = MarkerArray()
        
        for idx, wp in enumerate(self.waypoints):
            pose = wp['pose']
            
            # 球体
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'waypoints'
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(pose['x'])
            sphere.pose.position.y = float(pose['y'])
            sphere.pose.position.z = 0.2
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
            
            if idx == 0:
                sphere.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)
            elif idx == len(self.waypoints) - 1:
                sphere.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            else:
                sphere.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            
            marker_array.markers.append(sphere)
            
            # 文字
            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'labels'
            text.id = idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(pose['x'])
            text.pose.position.y = float(pose['y'])
            text.pose.position.z = 0.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f"{idx+1}"
            marker_array.markers.append(text)
        
        self.marker_pub.publish(marker_array)

    def run_test(self):
        """运行完整测试"""
        self.report.start_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.report.total_waypoints = len(self.waypoints)
        
        # 发布标记
        self.publish_markers()
        time.sleep(1)
        
        self.get_logger().info('')
        self.get_logger().info('开始导航性能测试...')
        self.get_logger().info('')
        
        for i, waypoint in enumerate(self.waypoints, 1):
            result = self.navigate_to_waypoint(waypoint, i)
            self.report.results.append(result)
            
            if result.success:
                self.report.successful += 1
            else:
                self.report.failed += 1
            
            # 短暂停顿
            time.sleep(1.0)
        
        self.report.end_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.calculate_stats()
        self.print_report()
        self.save_report()

    def calculate_stats(self):
        """计算统计数据"""
        successful = [r for r in self.report.results if r.success]
        
        if self.report.total_waypoints > 0:
            self.report.success_rate = (self.report.successful / self.report.total_waypoints) * 100
        
        if successful:
            self.report.total_time = sum(r.time_seconds for r in self.report.results)
            self.report.total_distance = sum(r.distance_meters for r in successful)
            self.report.avg_position_error = sum(r.position_error for r in successful) / len(successful)
            self.report.max_position_error = max(r.position_error for r in successful)
            self.report.avg_yaw_error = sum(r.yaw_error for r in successful) / len(successful)
            
            if self.report.total_time > 0:
                self.report.avg_velocity = self.report.total_distance / self.report.total_time

    def print_report(self):
        """打印测试报告"""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('         导航性能测试报告')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'测试名称: {self.report.test_name}')
        self.get_logger().info(f'开始时间: {self.report.start_time}')
        self.get_logger().info(f'结束时间: {self.report.end_time}')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'总目标点: {self.report.total_waypoints}')
        self.get_logger().info(f'成功: {self.report.successful}')
        self.get_logger().info(f'失败: {self.report.failed}')
        self.get_logger().info(f'成功率: {self.report.success_rate:.1f}%')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'总耗时: {self.report.total_time:.2f} 秒')
        self.get_logger().info(f'总距离: {self.report.total_distance:.2f} 米')
        self.get_logger().info(f'平均速度: {self.report.avg_velocity:.3f} m/s')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'平均位置误差: {self.report.avg_position_error:.4f} 米')
        self.get_logger().info(f'最大位置误差: {self.report.max_position_error:.4f} 米')
        self.get_logger().info(f'平均朝向误差: {self.report.avg_yaw_error:.2f}°')
        self.get_logger().info('=' * 60)

    def save_report(self):
        """保存报告到文件"""
        os.makedirs(self.output_dir, exist_ok=True)
        
        report_file = os.path.join(
            self.output_dir, 
            f'nav_test_{self.test_name}.yaml'
        )
        
        report_dict = {
            'test_info': {
                'name': self.report.test_name,
                'start_time': self.report.start_time,
                'end_time': self.report.end_time,
                'config_file': self.report.config_file,
            },
            'summary': {
                'total_waypoints': self.report.total_waypoints,
                'successful': self.report.successful,
                'failed': self.report.failed,
                'success_rate_percent': round(self.report.success_rate, 1),
                'total_time_seconds': round(self.report.total_time, 2),
                'total_distance_meters': round(self.report.total_distance, 2),
                'avg_velocity_mps': round(self.report.avg_velocity, 3),
                'avg_position_error_meters': round(self.report.avg_position_error, 4),
                'max_position_error_meters': round(self.report.max_position_error, 4),
                'avg_yaw_error_degrees': round(self.report.avg_yaw_error, 2),
            },
            'waypoint_results': [
                {
                    'name': r.name,
                    'success': r.success,
                    'target': {
                        'x': r.target_x,
                        'y': r.target_y,
                        'yaw_deg': r.target_yaw
                    },
                    'final': {
                        'x': round(r.final_x, 3),
                        'y': round(r.final_y, 3),
                        'yaw_deg': round(r.final_yaw, 1)
                    },
                    'time_seconds': round(r.time_seconds, 2),
                    'distance_meters': round(r.distance_meters, 2),
                    'position_error_meters': round(r.position_error, 4),
                    'yaw_error_degrees': round(r.yaw_error, 2),
                }
                for r in self.report.results
            ]
        }
        
        with open(report_file, 'w') as f:
            yaml.dump(report_dict, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        self.get_logger().info(f'报告已保存: {report_file}')
        
        # 同时保存一份简洁的 CSV 便于对比
        csv_file = os.path.join(self.output_dir, f'nav_test_{self.test_name}.csv')
        with open(csv_file, 'w') as f:
            f.write('waypoint,success,time_s,distance_m,pos_error_m,yaw_error_deg\n')
            for r in self.report.results:
                f.write(f'{r.name},{r.success},{r.time_seconds:.2f},'
                       f'{r.distance_meters:.2f},{r.position_error:.4f},{r.yaw_error:.2f}\n')
        
        self.get_logger().info(f'CSV 已保存: {csv_file}')


def main():
    parser = argparse.ArgumentParser(description='Nav2 导航性能测试')
    parser.add_argument('-c', '--config', required=True, help='Waypoints 配置文件')
    parser.add_argument('-o', '--output', default='test_results', help='输出目录')
    parser.add_argument('-n', '--name', default='', help='测试名称')
    parser.add_argument('-t', '--timeout', type=float, default=120.0, 
                        help='每个 waypoint 的超时时间（秒），默认 120')
    
    args = parser.parse_args()
    
    rclpy.init()
    tester = NavPerformanceTester(args.config, args.output, args.name, args.timeout)
    
    try:
        tester.run_test()
    except KeyboardInterrupt:
        tester.get_logger().info('测试被中断')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
