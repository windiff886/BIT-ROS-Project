#!/usr/bin/env bash
# ============================================================
# Nav2 参数批量测试脚本
# 自动遍历多组预设参数配置，依次运行测试并收集结果
#
# 使用方法：
#   ./map/batch_nav_test.sh                    # 测试所有预设配置
#   ./map/batch_nav_test.sh baseline high_speed  # 只测试指定配置
#
# 可用环境变量：
#   WAYPOINT_CFG       waypoints 配置文件
#   WORLD_NAME         Gazebo 世界名
#   MAP_FILE           地图文件
#   HEADLESS           1 无头模式（推荐批量测试时使用）
#   RUN_RVIZ           0 不启动 RViz（推荐批量测试时使用）
#   WAYPOINT_TIMEOUT   每个 waypoint 的超时时间（秒），默认 120
#   PAUSE_BETWEEN      测试间隔时间（秒），默认 10
#   SKIP_EXISTING      1 跳过已存在结果的测试，默认 0
# ============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 批量测试中断时的清理函数
batch_cleanup() {
  echo ""
  echo "[batch] 收到中断信号，正在清理所有进程..."
  set +e
  
  # 终止子脚本
  pkill -f "launch_tiago_test" 2>/dev/null || true
  
  # 终止 Gazebo/Ignition 相关进程
  pkill -f "ign gazebo" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "gzserver" 2>/dev/null || true
  pkill -f "gzclient" 2>/dev/null || true
  pkill -f "ruby.*gz" 2>/dev/null || true
  
  # 终止 ROS2 导航相关进程
  pkill -f "nav2" 2>/dev/null || true
  pkill -f "bt_navigator" 2>/dev/null || true
  pkill -f "controller_server" 2>/dev/null || true
  pkill -f "planner_server" 2>/dev/null || true
  pkill -f "behavior_server" 2>/dev/null || true
  pkill -f "amcl" 2>/dev/null || true
  pkill -f "map_server" 2>/dev/null || true
  pkill -f "lifecycle_manager" 2>/dev/null || true
  pkill -f "component_container" 2>/dev/null || true
  
  # 终止可视化和桥接
  pkill -f "rviz2" 2>/dev/null || true
  pkill -f "parameter_bridge" 2>/dev/null || true
  pkill -f "robot_state_publisher" 2>/dev/null || true
  pkill -f "static_transform_publisher" 2>/dev/null || true
  
  # 终止测试和辅助脚本
  pkill -f "nav_performance_test" 2>/dev/null || true
  pkill -f "odom_to_tf" 2>/dev/null || true
  pkill -f "laser_frame_remapper" 2>/dev/null || true
  
  # 等待进程退出
  sleep 2
  
  # 强制杀死残留的 Gazebo 进程
  pkill -9 -f "ign gazebo" 2>/dev/null || true
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -f "gzserver" 2>/dev/null || true
  
  echo "[batch] 清理完成。"
  exit 1
}
trap batch_cleanup INT TERM

# 参数配置目录
PARAMS_DIR="${ROOT_DIR}/config/nav2_params"

# 批量测试配置
# 注意：不再使用整体超时，每个 waypoint 有独立的超时限制
PAUSE_BETWEEN="${PAUSE_BETWEEN:-10}"
WAYPOINT_TIMEOUT="${WAYPOINT_TIMEOUT:-120}"  # 每个 waypoint 的超时时间（秒）
SKIP_EXISTING="${SKIP_EXISTING:-0}"
HEADLESS="${HEADLESS:-0}"      # 0=显示 Gazebo GUI，1=无头模式
RUN_RVIZ="${RUN_RVIZ:-1}"      # 1=启动 RViz，0=不启动

# 输出目录
BATCH_ID="batch_$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${OUTPUT_DIR:-${ROOT_DIR}/test_results/${BATCH_ID}}"

# 获取要测试的配置列表
if [ $# -gt 0 ]; then
  # 使用命令行参数指定的配置
  CONFIGS=("$@")
else
  # 使用所有预设配置
  CONFIGS=()
  for f in "${PARAMS_DIR}"/*.yaml; do
    if [ -f "$f" ]; then
      name=$(basename "$f" .yaml)
      CONFIGS+=("$name")
    fi
  done
fi

# 检查是否有配置
if [ ${#CONFIGS[@]} -eq 0 ]; then
  echo "[batch] 错误：没有找到任何参数配置文件！"
  echo "[batch] 请在 ${PARAMS_DIR} 目录下创建 .yaml 配置文件"
  exit 1
fi

echo ""
echo "============================================================"
echo "   Nav2 参数批量测试"
echo "============================================================"
echo "  批次 ID: ${BATCH_ID}"
echo "  配置目录: ${PARAMS_DIR}"
echo "  输出目录: ${OUTPUT_DIR}"
echo "  测试配置: ${CONFIGS[*]}"
echo "  配置数量: ${#CONFIGS[@]}"
echo "  Waypoint 超时: ${WAYPOINT_TIMEOUT}s"
echo "  测试间隔: ${PAUSE_BETWEEN}s"
echo "  无头模式: ${HEADLESS}"
echo "============================================================"
echo ""

# 创建输出目录
mkdir -p "${OUTPUT_DIR}"

# 创建批量测试摘要文件
SUMMARY_FILE="${OUTPUT_DIR}/batch_summary.yaml"
cat > "${SUMMARY_FILE}" << EOF
# Nav2 批量测试摘要
batch_id: "${BATCH_ID}"
start_time: "$(date -Iseconds)"
configs_tested: ${#CONFIGS[@]}
config_names:
EOF

for cfg in "${CONFIGS[@]}"; do
  echo "  - ${cfg}" >> "${SUMMARY_FILE}"
done

echo "results:" >> "${SUMMARY_FILE}"

# 测试结果统计
PASSED=0
FAILED=0
SKIPPED=0

# 遍历所有配置
for i in "${!CONFIGS[@]}"; do
  cfg="${CONFIGS[$i]}"
  test_num=$((i + 1))
  total=${#CONFIGS[@]}
  
  echo ""
  echo "============================================================"
  echo "[batch] 测试 ${test_num}/${total}: ${cfg}"
  echo "============================================================"
  
  # 检查配置文件
  PARAM_FILE="${PARAMS_DIR}/${cfg}.yaml"
  if [ ! -f "${PARAM_FILE}" ]; then
    echo "[batch] 警告：配置文件不存在：${PARAM_FILE}"
    echo "  - name: ${cfg}" >> "${SUMMARY_FILE}"
    echo "    status: skipped" >> "${SUMMARY_FILE}"
    echo "    reason: config_file_not_found" >> "${SUMMARY_FILE}"
    SKIPPED=$((SKIPPED + 1))
    continue
  fi
  
  # 检查是否跳过已存在的结果
  RESULT_FILE="${OUTPUT_DIR}/nav_test_${cfg}.yaml"
  if [ "${SKIP_EXISTING}" = "1" ] && [ -f "${RESULT_FILE}" ]; then
    echo "[batch] 跳过已存在结果：${cfg}"
    echo "  - name: ${cfg}" >> "${SUMMARY_FILE}"
    echo "    status: skipped" >> "${SUMMARY_FILE}"
    echo "    reason: result_exists" >> "${SUMMARY_FILE}"
    SKIPPED=$((SKIPPED + 1))
    continue
  fi
  
  # 设置环境变量
  export TEST_NAME="${cfg}"
  export NAV2_PARAMS="${PARAM_FILE}"
  export OUTPUT_DIR="${OUTPUT_DIR}"
  export HEADLESS="${HEADLESS}"
  export RUN_RVIZ="${RUN_RVIZ}"
  export WAYPOINT_TIMEOUT="${WAYPOINT_TIMEOUT}"
  
  # 记录测试开始时间
  TEST_START=$(date +%s)
  
  echo "[batch] 启动测试: ${cfg}"
  echo "[batch] 参数文件: ${PARAM_FILE}"
  echo "[batch] Waypoint 超时: ${WAYPOINT_TIMEOUT}s"
  
  # 运行测试（不设置整体超时，每个 waypoint 有独立超时）
  set +e
  "${SCRIPT_DIR}/launch_tiago_test.sh"
  EXIT_CODE=$?
  set -e
  
  # 记录测试结束时间
  TEST_END=$(date +%s)
  TEST_DURATION=$((TEST_END - TEST_START))
  
  # 记录结果
  echo "  - name: ${cfg}" >> "${SUMMARY_FILE}"
  echo "    param_file: ${PARAM_FILE}" >> "${SUMMARY_FILE}"
  echo "    duration_seconds: ${TEST_DURATION}" >> "${SUMMARY_FILE}"
  
  if [ ${EXIT_CODE} -eq 0 ]; then
    echo "[batch] ✓ 测试完成: ${cfg} (耗时 ${TEST_DURATION}s)"
    echo "    status: completed" >> "${SUMMARY_FILE}"
    echo "    exit_code: 0" >> "${SUMMARY_FILE}"
    PASSED=$((PASSED + 1))
  else
    echo "[batch] ✗ 测试异常退出: ${cfg} (退出码: ${EXIT_CODE})"
    echo "    status: error" >> "${SUMMARY_FILE}"
    echo "    exit_code: ${EXIT_CODE}" >> "${SUMMARY_FILE}"
    FAILED=$((FAILED + 1))
  fi
  
  # 清理残留进程（与 batch_cleanup 保持一致）
  echo "[batch] 清理进程..."
  pkill -f "ign gazebo" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "gzserver" 2>/dev/null || true
  pkill -f "gzclient" 2>/dev/null || true
  pkill -f "nav2" 2>/dev/null || true
  pkill -f "bt_navigator" 2>/dev/null || true
  pkill -f "controller_server" 2>/dev/null || true
  pkill -f "planner_server" 2>/dev/null || true
  pkill -f "component_container" 2>/dev/null || true
  pkill -f "rviz2" 2>/dev/null || true
  pkill -f "parameter_bridge" 2>/dev/null || true
  pkill -f "robot_state_publisher" 2>/dev/null || true
  pkill -f "nav_performance_test" 2>/dev/null || true
  pkill -f "odom_to_tf" 2>/dev/null || true
  pkill -f "laser_frame_remapper" 2>/dev/null || true
  sleep 2
  
  # 强制杀死残留的 Gazebo
  pkill -9 -f "ign gazebo" 2>/dev/null || true
  pkill -9 -f "gz sim" 2>/dev/null || true
  sleep 1
  
  # 测试间隔
  if [ ${test_num} -lt ${total} ]; then
    echo "[batch] 等待 ${PAUSE_BETWEEN}s 后开始下一个测试..."
    sleep "${PAUSE_BETWEEN}"
  fi
done

# 完成批量测试
END_TIME=$(date -Iseconds)
echo "" >> "${SUMMARY_FILE}"
echo "end_time: \"${END_TIME}\"" >> "${SUMMARY_FILE}"
echo "summary:" >> "${SUMMARY_FILE}"
echo "  passed: ${PASSED}" >> "${SUMMARY_FILE}"
echo "  failed: ${FAILED}" >> "${SUMMARY_FILE}"
echo "  skipped: ${SKIPPED}" >> "${SUMMARY_FILE}"
echo "  total: ${#CONFIGS[@]}" >> "${SUMMARY_FILE}"

echo ""
echo "============================================================"
echo "   批量测试完成"
echo "============================================================"
echo "  通过: ${PASSED}"
echo "  失败: ${FAILED}"
echo "  跳过: ${SKIPPED}"
echo "  总计: ${#CONFIGS[@]}"
echo ""
echo "  结果目录: ${OUTPUT_DIR}"
echo "  摘要文件: ${SUMMARY_FILE}"
echo "============================================================"
echo ""

# 生成对比报告（如果有 Python 脚本）
COMPARE_SCRIPT="${ROOT_DIR}/src/compare_nav_results.py"
if [ -f "${COMPARE_SCRIPT}" ]; then
  echo "[batch] 生成对比报告..."
  python3 "${COMPARE_SCRIPT}" --input "${OUTPUT_DIR}" --output "${OUTPUT_DIR}/comparison_report.html"
fi

# 返回状态码
if [ ${FAILED} -gt 0 ]; then
  exit 1
else
  exit 0
fi
