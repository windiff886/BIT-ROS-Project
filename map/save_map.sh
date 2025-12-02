#!/usr/bin/env bash
# 保存 SLAM 建图结果
# 用法：./save_map.sh [地图名称] [保存目录]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

MAP_NAME="${1:-tiago_map_$(date +%Y%m%d_%H%M%S)}"
SAVE_DIR="${2:-${ROOT_DIR}/maps}"

# 创建保存目录
mkdir -p "${SAVE_DIR}"

# ROS 环境
set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
set -u

echo "========================================"
echo "       保存 SLAM 地图"
echo "========================================"
echo ""
echo "地图名称: ${MAP_NAME}"
echo "保存目录: ${SAVE_DIR}"
echo ""

# 检查 SLAM 是否在运行
if ! ros2 topic list 2>/dev/null | grep -q "/map"; then
  echo "[错误] 未检测到 /map 话题，请确保 SLAM 正在运行！"
  exit 1
fi

echo "[1/3] 保存标准地图文件 (PGM + YAML)..."
ros2 run nav2_map_server map_saver_cli \
  -f "${SAVE_DIR}/${MAP_NAME}" \
  --ros-args -p use_sim_time:=true

echo ""
echo "[2/3] 保存 PNG 格式地图..."
ros2 run nav2_map_server map_saver_cli \
  -f "${SAVE_DIR}/${MAP_NAME}_png" \
  --fmt png \
  --ros-args -p use_sim_time:=true

echo ""
echo "[3/3] 保存 SLAM Toolbox 序列化文件..."
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${SAVE_DIR}/${MAP_NAME}_slam'}" \
  2>/dev/null || echo "[警告] 序列化保存失败（可能服务不可用）"

echo ""
echo "========================================"
echo "       保存完成！"
echo "========================================"
echo ""
echo "生成的文件："
echo ""
ls -lh "${SAVE_DIR}/${MAP_NAME}"* 2>/dev/null || true
echo ""
echo "文件说明："
echo "  - ${MAP_NAME}.pgm        : 灰度地图图像"
echo "  - ${MAP_NAME}.yaml       : 地图元数据"
echo "  - ${MAP_NAME}_png.png    : PNG 格式地图"
echo "  - ${MAP_NAME}_slam.*     : SLAM 序列化文件（可继续编辑）"
echo ""
echo "使用地图进行导航："
echo "  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=${SAVE_DIR}/${MAP_NAME}.yaml"
echo ""
