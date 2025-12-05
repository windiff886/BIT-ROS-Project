#!/usr/bin/env bash
# ============================================================
# Nav2 参数批量测试脚本
# 自动遍历多组预设参数配置和控制器，依次运行测试并收集结果
#
# 使用方法：
#   ./map/batch_nav_test.sh                              # 测试所有预设配置（默认控制器）
#   ./map/batch_nav_test.sh baseline high_speed          # 只测试指定配置
#   ./map/batch_nav_test.sh --controllers dwb rpp mppi   # 测试所有配置 + 指定控制器
#   ./map/batch_nav_test.sh --controllers dwb rpp -- baseline high_speed  # 指定控制器和配置
#   ./map/batch_nav_test.sh --controllers-only dwb rpp mppi  # 只测试控制器（使用默认参数）
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
#   TEST_CONTROLLERS   要测试的控制器列表（空格分隔），如 "dwb rpp mppi"
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
CONTROLLER_DIR="${ROOT_DIR}/config/controllers"

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

# 可用的控制器列表
AVAILABLE_CONTROLLERS=("dwb" "rpp" "mppi")

# 解析命令行参数
CONTROLLERS=()
CONFIGS=()
CONTROLLERS_ONLY=0
PARSE_MODE="configs"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --controllers)
      PARSE_MODE="controllers"
      shift
      ;;
    --controllers-only)
      PARSE_MODE="controllers"
      CONTROLLERS_ONLY=1
      shift
      ;;
    --)
      PARSE_MODE="configs"
      shift
      ;;
    -h|--help)
      echo "用法: $0 [选项] [配置名...]"
      echo ""
      echo "选项:"
      echo "  --controllers <ctrl...>      指定要测试的控制器 (dwb, rpp, mppi)"
      echo "  --controllers-only <ctrl...> 只测试控制器（使用默认参数）"
      echo "  --                           控制器和配置名之间的分隔符"
      echo "  -h, --help                   显示此帮助信息"
      echo ""
      echo "示例:"
      echo "  $0                                    # 测试所有参数配置（默认控制器）"
      echo "  $0 baseline high_speed                # 只测试指定配置"
      echo "  $0 --controllers dwb rpp mppi         # 所有配置 × 指定控制器"
      echo "  $0 --controllers dwb rpp -- baseline  # 指定控制器 × 指定配置"
      echo "  $0 --controllers-only dwb rpp mppi    # 只测试控制器（默认参数）"
      echo ""
      echo "可用控制器: ${AVAILABLE_CONTROLLERS[*]}"
      exit 0
      ;;
    *)
      if [ "$PARSE_MODE" = "controllers" ]; then
        CONTROLLERS+=("$1")
      else
        CONFIGS+=("$1")
      fi
      shift
      ;;
  esac
done

# 如果没有指定控制器，使用默认控制器（不测试多个控制器）
if [ ${#CONTROLLERS[@]} -eq 0 ]; then
  CONTROLLERS=("default")
fi

# 如果指定了 --controllers-only，只使用一个默认配置
if [ ${CONTROLLERS_ONLY} -eq 1 ]; then
  CONFIGS=("baseline")  # 使用 baseline 作为默认配置
fi

# 如果没有指定配置，获取所有配置
if [ ${#CONFIGS[@]} -eq 0 ]; then
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

# 验证控制器
for ctrl in "${CONTROLLERS[@]}"; do
  if [ "$ctrl" != "default" ]; then
    CTRL_FILE="${CONTROLLER_DIR}/${ctrl}.yaml"
    if [ ! -f "${CTRL_FILE}" ]; then
      echo "[batch] 警告：控制器配置文件不存在：${CTRL_FILE}"
      echo "[batch] 将自动创建控制器配置目录和文件..."
    fi
  fi
done

# 确保控制器配置目录存在
mkdir -p "${CONTROLLER_DIR}"

# 计算测试总数
if [ "${CONTROLLERS[0]}" = "default" ]; then
  TOTAL_TESTS=${#CONFIGS[@]}
else
  TOTAL_TESTS=$((${#CONFIGS[@]} * ${#CONTROLLERS[@]}))
fi

echo ""
echo "============================================================"
echo "   Nav2 参数批量测试"
echo "============================================================"
echo "  批次 ID: ${BATCH_ID}"
echo "  参数配置目录: ${PARAMS_DIR}"
echo "  控制器配置目录: ${CONTROLLER_DIR}"
echo "  输出目录: ${OUTPUT_DIR}"
echo "  测试参数配置: ${CONFIGS[*]}"
echo "  测试控制器: ${CONTROLLERS[*]}"
echo "  参数配置数量: ${#CONFIGS[@]}"
echo "  控制器数量: ${#CONTROLLERS[@]}"
echo "  测试总数: ${TOTAL_TESTS}"
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
total_tests: ${TOTAL_TESTS}
configs_count: ${#CONFIGS[@]}
controllers_count: ${#CONTROLLERS[@]}
config_names:
EOF

for cfg in "${CONFIGS[@]}"; do
  echo "  - ${cfg}" >> "${SUMMARY_FILE}"
done

echo "controller_names:" >> "${SUMMARY_FILE}"
for ctrl in "${CONTROLLERS[@]}"; do
  echo "  - ${ctrl}" >> "${SUMMARY_FILE}"
done

echo "results:" >> "${SUMMARY_FILE}"

# 测试结果统计
PASSED=0
FAILED=0
SKIPPED=0
CURRENT_TEST=0

# 双层循环：遍历所有控制器和配置的组合
for ctrl in "${CONTROLLERS[@]}"; do
  for cfg in "${CONFIGS[@]}"; do
    CURRENT_TEST=$((CURRENT_TEST + 1))
    
    # 生成测试名称
    if [ "$ctrl" = "default" ]; then
      TEST_LABEL="${cfg}"
    else
      TEST_LABEL="${cfg}_${ctrl}"
    fi
    
    echo ""
    echo "============================================================"
    echo "[batch] 测试 ${CURRENT_TEST}/${TOTAL_TESTS}: ${TEST_LABEL}"
    if [ "$ctrl" != "default" ]; then
      echo "[batch]   参数配置: ${cfg}"
      echo "[batch]   控制器: ${ctrl}"
    fi
    echo "============================================================"
    
    # 检查参数配置文件
    PARAM_FILE="${PARAMS_DIR}/${cfg}.yaml"
    if [ ! -f "${PARAM_FILE}" ]; then
      echo "[batch] 警告：参数配置文件不存在：${PARAM_FILE}"
      echo "  - name: ${TEST_LABEL}" >> "${SUMMARY_FILE}"
      echo "    config: ${cfg}" >> "${SUMMARY_FILE}"
      echo "    controller: ${ctrl}" >> "${SUMMARY_FILE}"
      echo "    status: skipped" >> "${SUMMARY_FILE}"
      echo "    reason: param_file_not_found" >> "${SUMMARY_FILE}"
      SKIPPED=$((SKIPPED + 1))
      continue
    fi
    
    # 检查控制器配置文件（如果不是 default）
    CTRL_FILE=""
    if [ "$ctrl" != "default" ]; then
      CTRL_FILE="${CONTROLLER_DIR}/${ctrl}.yaml"
      if [ ! -f "${CTRL_FILE}" ]; then
        echo "[batch] 警告：控制器配置文件不存在：${CTRL_FILE}"
        echo "[batch] 请先创建控制器配置文件，或运行以下命令生成模板："
        echo "        python3 ${ROOT_DIR}/src/generate_controller_config.py --controller ${ctrl} --output ${CTRL_FILE}"
        echo "  - name: ${TEST_LABEL}" >> "${SUMMARY_FILE}"
        echo "    config: ${cfg}" >> "${SUMMARY_FILE}"
        echo "    controller: ${ctrl}" >> "${SUMMARY_FILE}"
        echo "    status: skipped" >> "${SUMMARY_FILE}"
        echo "    reason: controller_file_not_found" >> "${SUMMARY_FILE}"
        SKIPPED=$((SKIPPED + 1))
        continue
      fi
    fi
    
    # 检查是否跳过已存在的结果
    RESULT_FILE="${OUTPUT_DIR}/nav_test_${TEST_LABEL}.yaml"
    if [ "${SKIP_EXISTING}" = "1" ] && [ -f "${RESULT_FILE}" ]; then
      echo "[batch] 跳过已存在结果：${TEST_LABEL}"
      echo "  - name: ${TEST_LABEL}" >> "${SUMMARY_FILE}"
      echo "    config: ${cfg}" >> "${SUMMARY_FILE}"
      echo "    controller: ${ctrl}" >> "${SUMMARY_FILE}"
      echo "    status: skipped" >> "${SUMMARY_FILE}"
      echo "    reason: result_exists" >> "${SUMMARY_FILE}"
      SKIPPED=$((SKIPPED + 1))
      continue
    fi
    
    # 设置环境变量
    export TEST_NAME="${TEST_LABEL}"
    export NAV2_PARAMS="${PARAM_FILE}"
    export OUTPUT_DIR="${OUTPUT_DIR}"
    export HEADLESS="${HEADLESS}"
    export RUN_RVIZ="${RUN_RVIZ}"
    export WAYPOINT_TIMEOUT="${WAYPOINT_TIMEOUT}"
    
    # 设置控制器配置（如果不是 default）
    if [ -n "${CTRL_FILE}" ]; then
      export NAV2_CONTROLLER_PARAMS="${CTRL_FILE}"
    else
      unset NAV2_CONTROLLER_PARAMS 2>/dev/null || true
    fi
    
    # 记录测试开始时间
    TEST_START=$(date +%s)
    
    echo "[batch] 启动测试: ${TEST_LABEL}"
    echo "[batch] 参数文件: ${PARAM_FILE}"
    if [ -n "${CTRL_FILE}" ]; then
      echo "[batch] 控制器文件: ${CTRL_FILE}"
    fi
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
  echo "  - name: ${TEST_LABEL}" >> "${SUMMARY_FILE}"
  echo "    config: ${cfg}" >> "${SUMMARY_FILE}"
  echo "    controller: ${ctrl}" >> "${SUMMARY_FILE}"
  echo "    param_file: ${PARAM_FILE}" >> "${SUMMARY_FILE}"
  if [ -n "${CTRL_FILE}" ]; then
    echo "    controller_file: ${CTRL_FILE}" >> "${SUMMARY_FILE}"
  fi
  echo "    duration_seconds: ${TEST_DURATION}" >> "${SUMMARY_FILE}"
  
  if [ ${EXIT_CODE} -eq 0 ]; then
    echo "[batch] ✓ 测试完成: ${TEST_LABEL} (耗时 ${TEST_DURATION}s)"
    echo "    status: completed" >> "${SUMMARY_FILE}"
    echo "    exit_code: 0" >> "${SUMMARY_FILE}"
    PASSED=$((PASSED + 1))
  else
    echo "[batch] ✗ 测试异常退出: ${TEST_LABEL} (退出码: ${EXIT_CODE})"
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
  if [ ${CURRENT_TEST} -lt ${TOTAL_TESTS} ]; then
    echo "[batch] 等待 ${PAUSE_BETWEEN}s 后开始下一个测试..."
    sleep "${PAUSE_BETWEEN}"
  fi
  done  # 结束 cfg 循环
done  # 结束 ctrl 循环

# 完成批量测试
END_TIME=$(date -Iseconds)
echo "" >> "${SUMMARY_FILE}"
echo "end_time: \"${END_TIME}\"" >> "${SUMMARY_FILE}"
echo "summary:" >> "${SUMMARY_FILE}"
echo "  passed: ${PASSED}" >> "${SUMMARY_FILE}"
echo "  failed: ${FAILED}" >> "${SUMMARY_FILE}"
echo "  skipped: ${SKIPPED}" >> "${SUMMARY_FILE}"
echo "  total: ${TOTAL_TESTS}" >> "${SUMMARY_FILE}"

echo ""
echo "============================================================"
echo "   批量测试完成"
echo "============================================================"
echo "  通过: ${PASSED}"
echo "  失败: ${FAILED}"
echo "  跳过: ${SKIPPED}"
echo "  总计: ${TOTAL_TESTS}"
echo ""
echo "  测试的参数配置: ${CONFIGS[*]}"
echo "  测试的控制器: ${CONTROLLERS[*]}"
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
