#!/usr/bin/env python3
"""
Nav2 参数合并工具

将基础参数文件与覆盖参数文件合并，生成完整的 Nav2 参数文件。
覆盖参数文件只需要包含要修改的参数。

用法:
    python3 merge_nav2_params.py --base base.yaml --override override.yaml --output merged.yaml
"""

import argparse
import yaml
from pathlib import Path
from typing import Any, Dict


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """
    递归深度合并两个字典。
    override 中的值会覆盖 base 中的值。
    """
    result = base.copy()
    
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    
    return result


def load_yaml(path: str) -> Dict[str, Any]:
    """加载 YAML 文件"""
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def save_yaml(data: Dict[str, Any], path: str):
    """保存 YAML 文件"""
    with open(path, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)


def main():
    parser = argparse.ArgumentParser(description='Nav2 参数合并工具')
    parser.add_argument('--base', '-b', required=True, help='基础参数文件路径')
    parser.add_argument('--override', '-o', required=True, help='覆盖参数文件路径')
    parser.add_argument('--output', '-O', required=True, help='输出文件路径')
    parser.add_argument('--verbose', '-v', action='store_true', help='显示详细信息')
    
    args = parser.parse_args()
    
    # 加载文件
    if args.verbose:
        print(f"加载基础配置: {args.base}")
    base_params = load_yaml(args.base)
    
    if args.verbose:
        print(f"加载覆盖配置: {args.override}")
    override_params = load_yaml(args.override)
    
    # 合并参数
    if args.verbose:
        print("合并参数...")
    merged_params = deep_merge(base_params, override_params)
    
    # 保存结果
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    save_yaml(merged_params, args.output)
    
    if args.verbose:
        print(f"已保存合并后的参数到: {args.output}")
    else:
        print(args.output)


if __name__ == '__main__':
    main()
