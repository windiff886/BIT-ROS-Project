#!/usr/bin/env python3
"""
Nav2 测试结果对比报告生成器

从批量测试结果中生成对比分析报告，包括：
- 各配置的性能指标对比
- 可视化图表
- 最优配置推荐
"""

import argparse
import os
import yaml
import csv
from datetime import datetime
from typing import Dict, List, Any, Optional
from pathlib import Path


def load_yaml_results(result_dir: str) -> Dict[str, Dict[str, Any]]:
    """加载所有 YAML 测试结果"""
    results = {}
    
    for file in Path(result_dir).glob("nav_test_*.yaml"):
        if file.name == "batch_summary.yaml":
            continue
        
        config_name = file.stem.replace("nav_test_", "")
        
        try:
            with open(file, 'r') as f:
                data = yaml.safe_load(f)
                if data:
                    results[config_name] = data
        except Exception as e:
            print(f"警告: 无法加载 {file}: {e}")
    
    return results


def load_csv_results(result_dir: str) -> Dict[str, List[Dict[str, Any]]]:
    """加载所有 CSV 详细结果"""
    results = {}
    
    for file in Path(result_dir).glob("nav_test_*.csv"):
        config_name = file.stem.replace("nav_test_", "")
        
        try:
            with open(file, 'r') as f:
                reader = csv.DictReader(f)
                results[config_name] = list(reader)
        except Exception as e:
            print(f"警告: 无法加载 {file}: {e}")
    
    return results


def calculate_metrics(results: Dict[str, Dict[str, Any]]) -> Dict[str, Dict[str, float]]:
    """计算各配置的性能指标"""
    metrics = {}
    
    for config_name, data in results.items():
        summary = data.get('summary', {})
        
        metrics[config_name] = {
            'total_time': summary.get('total_time', 0),
            'total_distance': summary.get('total_distance', 0),
            'avg_speed': summary.get('avg_speed', 0),
            'success_rate': summary.get('success_rate', 0),
            'waypoints_reached': summary.get('waypoints_reached', 0),
            'waypoints_total': summary.get('waypoints_total', 0),
            'avg_time_per_waypoint': 0,
            'efficiency_score': 0,
        }
        
        # 计算派生指标
        wp_reached = metrics[config_name]['waypoints_reached']
        if wp_reached > 0:
            metrics[config_name]['avg_time_per_waypoint'] = \
                metrics[config_name]['total_time'] / wp_reached
        
        # 效率评分 (综合考虑速度和成功率)
        speed = metrics[config_name]['avg_speed']
        success = metrics[config_name]['success_rate']
        if speed > 0:
            metrics[config_name]['efficiency_score'] = (speed * success) / 100
    
    return metrics


def generate_html_report(
    results: Dict[str, Dict[str, Any]],
    metrics: Dict[str, Dict[str, float]],
    output_path: str
):
    """生成 HTML 对比报告"""
    
    html = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Nav2 参数对比报告</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f5f5f5;
        }
        h1, h2, h3 {
            color: #333;
        }
        .summary-box {
            background: white;
            border-radius: 8px;
            padding: 20px;
            margin: 20px 0;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        table {
            width: 100%;
            border-collapse: collapse;
            margin: 15px 0;
            background: white;
        }
        th, td {
            padding: 12px;
            text-align: left;
            border-bottom: 1px solid #ddd;
        }
        th {
            background-color: #4CAF50;
            color: white;
        }
        tr:hover {
            background-color: #f5f5f5;
        }
        .best {
            background-color: #c8e6c9 !important;
            font-weight: bold;
        }
        .worst {
            background-color: #ffcdd2 !important;
        }
        .chart-container {
            background: white;
            border-radius: 8px;
            padding: 20px;
            margin: 20px 0;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .bar {
            height: 30px;
            margin: 5px 0;
            border-radius: 4px;
            display: flex;
            align-items: center;
            padding-left: 10px;
            color: white;
            font-weight: bold;
        }
        .metric-card {
            display: inline-block;
            width: 200px;
            padding: 15px;
            margin: 10px;
            background: white;
            border-radius: 8px;
            text-align: center;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .metric-value {
            font-size: 24px;
            font-weight: bold;
            color: #4CAF50;
        }
        .metric-label {
            color: #666;
            font-size: 14px;
        }
        .recommendation {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            border-radius: 8px;
            margin: 20px 0;
        }
        .recommendation h3 {
            color: white;
            margin-top: 0;
        }
    </style>
</head>
<body>
    <h1>🤖 Nav2 参数对比报告</h1>
    <p>生成时间: """ + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + """</p>
"""
    
    if not metrics:
        html += "<p>没有找到测试结果数据。</p></body></html>"
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(html)
        return
    
    # 找出最优配置
    best_speed = max(metrics.items(), key=lambda x: x[1]['avg_speed'], default=(None, {}))
    best_success = max(metrics.items(), key=lambda x: x[1]['success_rate'], default=(None, {}))
    best_efficiency = max(metrics.items(), key=lambda x: x[1]['efficiency_score'], default=(None, {}))
    
    # 推荐区域
    html += """
    <div class="recommendation">
        <h3>📊 最优配置推荐</h3>
        <p><strong>最快速度:</strong> """ + (best_speed[0] or 'N/A') + """ (平均速度: """ + f"{best_speed[1].get('avg_speed', 0):.3f}" + """ m/s)</p>
        <p><strong>最高成功率:</strong> """ + (best_success[0] or 'N/A') + """ (成功率: """ + f"{best_success[1].get('success_rate', 0):.1f}" + """%)</p>
        <p><strong>综合最优:</strong> """ + (best_efficiency[0] or 'N/A') + """ (效率评分: """ + f"{best_efficiency[1].get('efficiency_score', 0):.3f}" + """)</p>
    </div>
"""
    
    # 概览卡片
    html += """
    <div class="summary-box">
        <h2>📈 测试概览</h2>
        <div class="metric-card">
            <div class="metric-value">""" + str(len(metrics)) + """</div>
            <div class="metric-label">测试配置数</div>
        </div>
"""
    
    total_waypoints = sum(m['waypoints_total'] for m in metrics.values())
    total_reached = sum(m['waypoints_reached'] for m in metrics.values())
    
    html += """
        <div class="metric-card">
            <div class="metric-value">""" + str(total_reached) + """ / """ + str(total_waypoints) + """</div>
            <div class="metric-label">总到达/总目标点</div>
        </div>
    </div>
"""
    
    # 详细对比表格
    html += """
    <div class="summary-box">
        <h2>📋 详细对比</h2>
        <table>
            <tr>
                <th>配置名称</th>
                <th>总时间 (s)</th>
                <th>总距离 (m)</th>
                <th>平均速度 (m/s)</th>
                <th>成功率 (%)</th>
                <th>到达点数</th>
                <th>效率评分</th>
            </tr>
"""
    
    # 按效率评分排序
    sorted_metrics = sorted(metrics.items(), key=lambda x: x[1]['efficiency_score'], reverse=True)
    
    for i, (config_name, m) in enumerate(sorted_metrics):
        row_class = ""
        if i == 0:
            row_class = "best"
        elif i == len(sorted_metrics) - 1 and len(sorted_metrics) > 1:
            row_class = "worst"
        
        html += f"""
            <tr class="{row_class}">
                <td>{config_name}</td>
                <td>{m['total_time']:.2f}</td>
                <td>{m['total_distance']:.2f}</td>
                <td>{m['avg_speed']:.3f}</td>
                <td>{m['success_rate']:.1f}</td>
                <td>{m['waypoints_reached']} / {m['waypoints_total']}</td>
                <td>{m['efficiency_score']:.3f}</td>
            </tr>
"""
    
    html += """
        </table>
    </div>
"""
    
    # 速度对比图
    max_speed = max(m['avg_speed'] for m in metrics.values()) if metrics else 1
    colors = ['#4CAF50', '#2196F3', '#FF9800', '#9C27B0', '#F44336', '#00BCD4']
    
    html += """
    <div class="chart-container">
        <h2>🚀 平均速度对比</h2>
"""
    
    for i, (config_name, m) in enumerate(sorted_metrics):
        width = (m['avg_speed'] / max_speed * 100) if max_speed > 0 else 0
        color = colors[i % len(colors)]
        html += f"""
        <div class="bar" style="width: {max(width, 10):.1f}%; background-color: {color};">
            {config_name}: {m['avg_speed']:.3f} m/s
        </div>
"""
    
    html += """
    </div>
"""
    
    # 成功率对比图
    html += """
    <div class="chart-container">
        <h2>✅ 成功率对比</h2>
"""
    
    for i, (config_name, m) in enumerate(sorted(metrics.items(), key=lambda x: x[1]['success_rate'], reverse=True)):
        width = m['success_rate']
        color = colors[i % len(colors)]
        html += f"""
        <div class="bar" style="width: {max(width, 10):.1f}%; background-color: {color};">
            {config_name}: {m['success_rate']:.1f}%
        </div>
"""
    
    html += """
    </div>
"""
    
    # 各配置详情
    html += """
    <div class="summary-box">
        <h2>📝 各配置详情</h2>
"""
    
    for config_name, data in results.items():
        html += f"""
        <h3>{config_name}</h3>
        <p><strong>测试时间:</strong> {data.get('test_info', {}).get('start_time', 'N/A')}</p>
        <p><strong>参数文件:</strong> {data.get('test_info', {}).get('nav2_params', 'N/A')}</p>
"""
        
        # 各目标点结果
        waypoints = data.get('waypoints', [])
        if waypoints:
            html += """
        <table>
            <tr>
                <th>目标点</th>
                <th>状态</th>
                <th>耗时 (s)</th>
                <th>距离 (m)</th>
                <th>平均速度 (m/s)</th>
            </tr>
"""
            for wp in waypoints:
                status_emoji = "✅" if wp.get('success') else "❌"
                html += f"""
            <tr>
                <td>{wp.get('name', 'N/A')}</td>
                <td>{status_emoji}</td>
                <td>{wp.get('time', 0):.2f}</td>
                <td>{wp.get('distance', 0):.2f}</td>
                <td>{wp.get('avg_speed', 0):.3f}</td>
            </tr>
"""
            html += """
        </table>
"""
    
    html += """
    </div>
    
    <footer style="text-align: center; color: #666; margin-top: 40px;">
        <p>由 Nav2 批量测试系统自动生成</p>
    </footer>
</body>
</html>
"""
    
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html)
    
    print(f"报告已生成: {output_path}")


def generate_csv_summary(metrics: Dict[str, Dict[str, float]], output_path: str):
    """生成 CSV 汇总表"""
    
    fieldnames = [
        'config_name', 'total_time', 'total_distance', 'avg_speed',
        'success_rate', 'waypoints_reached', 'waypoints_total',
        'avg_time_per_waypoint', 'efficiency_score'
    ]
    
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        
        for config_name, m in sorted(metrics.items(), key=lambda x: x[1]['efficiency_score'], reverse=True):
            row = {'config_name': config_name}
            row.update(m)
            writer.writerow(row)
    
    print(f"CSV 汇总已生成: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Nav2 测试结果对比报告生成器')
    parser.add_argument('--input', '-i', required=True, help='测试结果目录')
    parser.add_argument('--output', '-o', help='输出报告路径 (默认: input/comparison_report.html)')
    parser.add_argument('--csv', action='store_true', help='同时生成 CSV 汇总')
    
    args = parser.parse_args()
    
    input_dir = args.input
    output_path = args.output or os.path.join(input_dir, 'comparison_report.html')
    
    print(f"加载测试结果: {input_dir}")
    
    # 加载结果
    results = load_yaml_results(input_dir)
    
    if not results:
        print("警告: 没有找到任何测试结果！")
        # 仍然生成空报告
        generate_html_report({}, {}, output_path)
        return
    
    print(f"找到 {len(results)} 个配置的测试结果")
    
    # 计算指标
    metrics = calculate_metrics(results)
    
    # 生成 HTML 报告
    generate_html_report(results, metrics, output_path)
    
    # 生成 CSV 汇总
    if args.csv:
        csv_path = output_path.replace('.html', '.csv')
        generate_csv_summary(metrics, csv_path)
    
    print("完成！")


if __name__ == '__main__':
    main()
