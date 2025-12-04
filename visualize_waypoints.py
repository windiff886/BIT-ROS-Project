#!/usr/bin/env python3
"""
可视化 my_route.yaml 中的目标点，叠加在导航地图上
"""

import yaml
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from PIL import Image

def load_waypoints(yaml_file):
    """加载 YAML 文件中的目标点"""
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    return data

def load_map(map_yaml_file):
    """加载导航地图"""
    with open(map_yaml_file, 'r') as f:
        map_data = yaml.safe_load(f)
    
    # 加载地图图像
    map_dir = Path(map_yaml_file).parent
    image_path = map_dir / map_data['image']
    map_image = Image.open(image_path)
    
    return map_image, map_data

def visualize_waypoints(data, map_image=None, map_meta=None):
    """可视化目标点"""
    waypoints = data.get('waypoints', [])
    
    if not waypoints:
        print("No waypoints found!")
        return
    
    # 提取坐标和方向
    names = []
    xs = []
    ys = []
    yaws = []
    
    for wp in waypoints:
        names.append(wp['name'])
        # 目标点左右颠倒：X取反，上下颠倒：Y取反
        xs.append(-wp['pose']['y'])
        ys.append(wp['pose']['x'])  # Y取反变正，实现上下颠倒
        # 角度也要调整
        yaws.append(wp['pose']['yaw_deg'] + 90)
    
    # 如果有地图，先计算地图范围
    if map_image is not None and map_meta is not None:
        resolution = map_meta['resolution']
        origin = map_meta['origin']
        
        # 计算地图的实际坐标范围
        map_array = np.array(map_image)
        # 旋转地图90度（逆时针）
        map_array = np.rot90(map_array, k=1)
        # 只上下颠倒（Y轴反向）
        map_array = np.flipud(map_array)
        height, width = map_array.shape[:2]
        
        # 原始地图范围
        orig_x_min = origin[0]
        orig_y_min = origin[1]
        orig_width = np.array(map_image).shape[1]
        orig_height = np.array(map_image).shape[0]
        orig_x_max = orig_x_min + orig_width * resolution
        orig_y_max = orig_y_min + orig_height * resolution
        
        # 旋转后的范围：新X = -原Y，新Y = -原X
        x_min = -orig_y_max
        x_max = -orig_y_min
        y_min = -orig_x_max
        y_max = -orig_x_min
        
        # 根据地图实际范围设置图形尺寸（横向）
        map_width = x_max - x_min
        map_height = y_max - y_min
        aspect = map_width / map_height
        fig_height = 10
        fig_width = fig_height * aspect
        fig, ax = plt.subplots(figsize=(fig_width, fig_height))
        
        # 显示地图
        ax.imshow(map_array, cmap='gray', extent=[x_min, x_max, y_min, y_max], 
                  origin='lower', alpha=0.8, zorder=1)
        
        # 设置坐标轴范围紧贴地图
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
    else:
        # 没有地图时，根据点的范围设置
        margin = 2
        x_min, x_max = min(xs) - margin, max(xs) + margin
        y_min, y_max = min(ys) - margin, max(ys) + margin
        map_width = x_max - x_min
        map_height = y_max - y_min
        aspect = map_width / map_height
        fig_height = 10
        fig_width = fig_height * aspect
        fig, ax = plt.subplots(figsize=(fig_width, fig_height))
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
    
    # 绘制点
    scatter = ax.scatter(xs, ys, c=range(len(xs)), cmap='plasma', s=250, zorder=5, edgecolors='white', linewidths=2)
    
    # 绘制方向箭头
    arrow_length = 1.5
    for i, (x, y, yaw, name) in enumerate(zip(xs, ys, yaws, names)):
        # 将角度转换为弧度
        yaw_rad = np.radians(yaw)
        dx = arrow_length * np.cos(yaw_rad)
        dy = arrow_length * np.sin(yaw_rad)
        ax.arrow(x, y, dx, dy, head_width=0.4, head_length=0.2, fc='red', ec='red', zorder=6)
    
    # 添加标签
    for i, (x, y, name) in enumerate(zip(xs, ys, names)):
        ax.annotate(f'{name}\n({x:.2f}, {y:.2f})', 
                   (x, y), 
                   textcoords="offset points", 
                   xytext=(10, 10), 
                   ha='left',
                   fontsize=9,
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
    
    # 绘制连接线（按顺序）
    ax.plot(xs, ys, 'b--', alpha=0.5, linewidth=1.5, zorder=3, label='Route Order')
    
    # 标记起点和终点
    ax.scatter([xs[0]], [ys[0]], c='green', s=300, marker='s', zorder=7, label=f'Start: {names[0]}', edgecolors='black', linewidths=2)
    ax.scatter([xs[-1]], [ys[-1]], c='red', s=300, marker='*', zorder=7, label=f'End: {names[-1]}', edgecolors='black', linewidths=2)
    
    # 设置图形属性 - 简洁模式
    ax.set_aspect('equal')
    ax.axis('off')  # 关闭坐标轴
    
    # 添加简洁图例
    ax.legend(loc='upper left', fontsize=10, framealpha=0.8)
    
    # 紧凑布局，去除空白
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
    
    # 保存图片
    output_path = Path(__file__).parent / 'waypoints_visualization.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight', pad_inches=0.1)
    print(f"图片已保存到: {output_path}")
    
    # 显示图形
    plt.show()
    
    # 打印目标点信息表格
    print("\n" + "="*60)
    print("目标点详细信息:")
    print("="*60)
    print(f"{'序号':<6}{'名称':<12}{'X坐标':<12}{'Y坐标':<12}{'朝向(度)':<10}")
    print("-"*60)
    for i, wp in enumerate(waypoints, 1):
        print(f"{i:<6}{wp['name']:<12}{wp['pose']['x']:<12.3f}{wp['pose']['y']:<12.3f}{wp['pose']['yaw_deg']:<10.1f}")
    print("="*60)

if __name__ == '__main__':
    # 获取脚本所在目录
    script_dir = Path(__file__).parent
    yaml_file = script_dir / 'config' / 'my_route.yaml'
    map_yaml_file = script_dir / 'map' / 'nav_maps' / 'warehouse.yaml'
    
    print(f"Loading waypoints: {yaml_file}")
    data = load_waypoints(yaml_file)
    
    # 加载地图
    map_image = None
    map_meta = None
    if map_yaml_file.exists():
        print(f"Loading map: {map_yaml_file}")
        map_image, map_meta = load_map(map_yaml_file)
        print(f"  Resolution: {map_meta['resolution']} m/pixel")
        print(f"  Origin: {map_meta['origin']}")
    else:
        print("Map file not found, visualizing waypoints only")
    
    visualize_waypoints(data, map_image, map_meta)
