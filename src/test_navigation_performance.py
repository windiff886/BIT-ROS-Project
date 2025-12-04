#!/usr/bin/env python3
"""
性能测试脚本：使用录制的 waypoints 测试导航性能。

功能：
- 依次导航到每个 waypoint
- 记录每段的时间、距离、成功率
- 生成性能报告

使用方法：
    python3 src/test_navigation_performance.py --config config/recorded_waypoints.yaml
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import yaml
import math
import time
import os
from datetime import datetime
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class WaypointResult:
    """单个 waypoint 的导航结果"""
    name: str
    target_x: float
    target_y: float
    target_yaw: float
    success: bool = False
    time_taken: float = 0.0
    distance_traveled: float = 0.0
    final_x: float = 0.0
    final_y: float = 0.0
    final_yaw: float = 0.0
    position_error: float = 0.0
    yaw_error: float = 0.0
    error_message: str = ''


@dataclass 
class TestReport:
    """测试报告"""
    start_time: str = ''
    end_time: str = ''
    config_file: str = ''
    total_waypoints: int = 0
    successful_waypoints: int = 0
    failed_waypoints: int = 0
    total_time: float = 0.0
    total_distance: float = 0.0
    avg_position_error: float = 0.0
    avg_yaw_error: float = 0.0
    results: List[WaypointResult] = field(default_factory=list)


class NavigationPerformanceTester(Node):
    def __init__(self, config_file: str, output_dir: str):
        super().__init__('navigation_performance_tester')
        
        self.config_file = config_file
        self.output_dir = output_dir
        self.report = TestReport(config_file=config_file)
        
        # 当前位置追踪
        self.current_pose = None
        self.path_distance = 0.0
        self.last_pose = None
        
        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # 加载 waypoints
        self.waypoints = self.load_waypoints(config_file)
        self.frame_id = 'map'
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('导航性能测试器已启动')
        self.get_logger().info(f'配置文件: {config_file}')
        self.get_logger().info(f'Waypoints 数量: {len(self.waypoints)}')
        self.get_logger().info('=' * 60)

    def load_waypoints(self, config_file: str) -> list:
        """加载 waypoints 配置"""
        with open(config_file, 'r') as f:
            data = yaml.safe_load(f)
        
        self.frame_id = data.get('frame_id', 'map')
        waypoints = data.get('waypoints', [])
        
        count = data.get('count', 0)
        if count > 0:
            waypoints = waypoints[:count]
        
        return waypoints

    def odom_callback(self, msg: Odometry):
        """更新当前位置并累计行驶距离"""
        self.current_pose = msg.pose.pose
        
        if self.last_pose is not None:
            dx = msg.pose.pose.position.x - self.last_pose.position.x
            dy = msg.pose.pose.position.y - self.last_pose.position.y
            self.path_distance += math.sqrt(dx*dx + dy*dy)
        
        self.last_pose = msg.pose.pose

    def get_yaw_from_quaternion(self, q):
        """从四元数提取 yaw 角"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def create_goal_pose(self, waypoint: dict) -> PoseStamped:
        """创建目标位姿"""
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        
        pose = waypoint['pose']
        goal.pose.position.x = float(pose['x'])
        goal.pose.position.y = float(pose['y'])
        goal.pose.position.z = 0.0
        
        yaw = math.radians(float(pose.get('yaw_deg', 0)))
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        
        return goal

    async def navigate_to_waypoint(self, waypoint: dict) -> WaypointResult:
        """导航到单个 waypoint 并记录结果"""
        pose = waypoint['pose']
        result = WaypointResult(
            name=waypoint.get('name', 'unnamed'),
            target_x=float(pose['x']),
            target_y=float(pose['y']),
            target_yaw=float(pose.get('yaw_deg', 0))
        )
        
        # 重置距离计数
        self.path_distance = 0.0
        start_time = time.time()
        
        # 创建目标
        goal_pose = self.create_goal_pose(waypoint)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(
            f'导航到 {result.name}: x={result.target_x:.2f}, y={result.target_y:.2f}, yaw={result.target_yaw:.1f}°'
        )
        
        # 等待 action server
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            result.success = False
            result.error_message = 'Navigation action server not available'
            return result
        
        # 发送目标
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future
        
        if not goal_handle.accepted:
            result.success = False
            result.error_message = 'Goal was rejected'
            return result
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        nav_result = await result_future
        
        # 记录结果
        result.time_taken = time.time() - start_time
        result.distance_traveled = self.path_distance
        
        if nav_result.status == 4:  # SUCCEEDED
            result.success = True
            
            # 计算最终误差
            if self.current_pose:
                result.final_x = self.current_pose.position.x
                result.final_y = self.current_pose.position.y
                result.final_yaw = math.degrees(
                    self.get_yaw_from_quaternion(self.current_pose.orientation)
                )
                
                result.position_error = math.sqrt(
                    (result.final_x - result.target_x) ** 2 +
                    (result.final_y - result.target_y) ** 2
                )
                
                yaw_diff = result.final_yaw - result.target_yaw
                # 归一化到 [-180, 180]
                while yaw_diff > 180:
                    yaw_diff -= 360
                while yaw_diff < -180:
                    yaw_diff += 360
                result.yaw_error = abs(yaw_diff)
            
            self.get_logger().info(
                f'  ✓ 成功! 耗时={result.time_taken:.2f}s, '
                f'距离={result.distance_traveled:.2f}m, '
                f'位置误差={result.position_error:.3f}m'
            )
        else:
            result.success = False
            result.error_message = f'Navigation failed with status {nav_result.status}'
            self.get_logger().warn(f'  ✗ 失败: {result.error_message}')
        
        return result

    async def run_test(self):
        """运行完整测试"""
        self.report.start_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.report.total_waypoints = len(self.waypoints)
        
        self.get_logger().info('开始导航性能测试...')
        self.get_logger().info('')
        
        for i, waypoint in enumerate(self.waypoints, 1):
            self.get_logger().info(f'[{i}/{len(self.waypoints)}] ', end='')
            
            result = await self.navigate_to_waypoint(waypoint)
            self.report.results.append(result)
            
            if result.success:
                self.report.successful_waypoints += 1
            else:
                self.report.failed_waypoints += 1
            
            # 短暂停顿
            await self.sleep(1.0)
        
        self.report.end_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.calculate_statistics()
        self.print_report()
        self.save_report()

    async def sleep(self, seconds: float):
        """异步睡眠"""
        await rclpy.task.sleep(Duration(seconds=seconds))

    def calculate_statistics(self):
        """计算统计数据"""
        successful_results = [r for r in self.report.results if r.success]
        
        if successful_results:
            self.report.total_time = sum(r.time_taken for r in self.report.results)
            self.report.total_distance = sum(r.distance_traveled for r in successful_results)
            self.report.avg_position_error = sum(r.position_error for r in successful_results) / len(successful_results)
            self.report.avg_yaw_error = sum(r.yaw_error for r in successful_results) / len(successful_results)

    def print_report(self):
        """打印测试报告"""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('导航性能测试报告')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'开始时间: {self.report.start_time}')
        self.get_logger().info(f'结束时间: {self.report.end_time}')
        self.get_logger().info(f'配置文件: {self.report.config_file}')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'总 Waypoints: {self.report.total_waypoints}')
        self.get_logger().info(f'成功: {self.report.successful_waypoints}')
        self.get_logger().info(f'失败: {self.report.failed_waypoints}')
        self.get_logger().info(f'成功率: {self.report.successful_waypoints/self.report.total_waypoints*100:.1f}%')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'总耗时: {self.report.total_time:.2f} 秒')
        self.get_logger().info(f'总行驶距离: {self.report.total_distance:.2f} 米')
        self.get_logger().info(f'平均位置误差: {self.report.avg_position_error:.4f} 米')
        self.get_logger().info(f'平均朝向误差: {self.report.avg_yaw_error:.2f}°')
        self.get_logger().info('=' * 60)
        
        # 详细结果
        self.get_logger().info('')
        self.get_logger().info('详细结果:')
        for r in self.report.results:
            status = '✓' if r.success else '✗'
            self.get_logger().info(
                f'  {status} {r.name}: time={r.time_taken:.2f}s, dist={r.distance_traveled:.2f}m, '
                f'pos_err={r.position_error:.3f}m, yaw_err={r.yaw_error:.1f}°'
            )

    def save_report(self):
        """保存报告到文件"""
        os.makedirs(self.output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_file = os.path.join(self.output_dir, f'nav_test_report_{timestamp}.yaml')
        
        # 转换为可序列化的字典
        report_dict = {
            'summary': {
                'start_time': self.report.start_time,
                'end_time': self.report.end_time,
                'config_file': self.report.config_file,
                'total_waypoints': self.report.total_waypoints,
                'successful_waypoints': self.report.successful_waypoints,
                'failed_waypoints': self.report.failed_waypoints,
                'success_rate': f'{self.report.successful_waypoints/self.report.total_waypoints*100:.1f}%',
                'total_time_seconds': round(self.report.total_time, 2),
                'total_distance_meters': round(self.report.total_distance, 2),
                'avg_position_error_meters': round(self.report.avg_position_error, 4),
                'avg_yaw_error_degrees': round(self.report.avg_yaw_error, 2),
            },
            'results': [
                {
                    'name': r.name,
                    'success': r.success,
                    'target': {'x': r.target_x, 'y': r.target_y, 'yaw_deg': r.target_yaw},
                    'final': {'x': round(r.final_x, 3), 'y': round(r.final_y, 3), 'yaw_deg': round(r.final_yaw, 1)},
                    'time_seconds': round(r.time_taken, 2),
                    'distance_meters': round(r.distance_traveled, 2),
                    'position_error_meters': round(r.position_error, 4),
                    'yaw_error_degrees': round(r.yaw_error, 2),
                    'error_message': r.error_message if r.error_message else None
                }
                for r in self.report.results
            ]
        }
        
        with open(report_file, 'w') as f:
            yaml.dump(report_dict, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        self.get_logger().info(f'报告已保存到: {report_file}')


async def run_tester(node: NavigationPerformanceTester):
    """运行测试器的异步函数"""
    # 等待一下让系统稳定
    await node.sleep(2.0)
    await node.run_test()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='导航性能测试')
    parser.add_argument(
        '-c', '--config',
        default='config/recorded_waypoints.yaml',
        help='Waypoints 配置文件'
    )
    parser.add_argument(
        '-o', '--output-dir',
        default='report/nav_tests',
        help='测试报告输出目录'
    )
    
    args = parser.parse_args()
    
    if not os.path.exists(args.config):
        print(f'错误: 配置文件不存在: {args.config}')
        return
    
    rclpy.init()
    node = NavigationPerformanceTester(args.config, args.output_dir)
    
    # 使用 executor 运行异步任务
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        # 创建异步任务
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        # 简化版：使用同步方式运行
        import threading
        
        def spin_thread():
            executor.spin()
        
        spin_t = threading.Thread(target=spin_thread, daemon=True)
        spin_t.start()
        
        # 运行测试
        loop.run_until_complete(run_tester(node))
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
