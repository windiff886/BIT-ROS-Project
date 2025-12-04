#!/usr/bin/env bash
# 导航性能测试脚本：启动仿真环境，自动让机器人依次前往预设目标点，并记录性能数据。
#
# 使用方法：
#   ./map/launch_tiago_test.sh                           # 使用默认配置
#   ./map/launch_tiago_test.sh config/my_route.yaml      # 指定路径文件
#   TEST_NAME=baseline ./map/launch_tiago_test.sh        # 指定测试名称
#
# 可用环境变量：
#   WAYPOINT_CFG     waypoints 配置文件路径，默认 config/my_route.yaml
#   TEST_NAME        测试名称（用于报告文件命名），默认为时间戳
#   MAP_FILE         地图 yaml 路径，默认 map/nav_maps/warehouse.yaml
#   NAV2_PARAMS      Nav2 参数文件
#   WORLD_NAME       Gazebo 世界名，默认 warehouse
#   HEADLESS         1 无头渲染，0 GUI（默认 0）
#   RUN_RVIZ         1 启动 RViz（默认 1）
#   AUTO_INIT_POSE   1 自动设置初始位姿（默认 1），0 等待手动设置
#   INIT_POSE_WAIT   等待用户设置初始位姿的时间（秒），默认 10
#   OUTPUT_DIR       测试报告输出目录，默认 report/nav_tests

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 测试配置
WAYPOINT_CFG="${WAYPOINT_CFG:-${1:-${ROOT_DIR}/config/my_route.yaml}}"
TEST_NAME="${TEST_NAME:-$(date +%Y%m%d_%H%M%S)}"
OUTPUT_DIR="${OUTPUT_DIR:-${ROOT_DIR}/test_results}"

# 仿真配置
WORLD_NAME="${WORLD_NAME:-warehouse}"
WORLD_FILE="${ROOT_DIR}/map/turtlebot4_gz_bringup/worlds/${WORLD_NAME}.sdf"
TIAGO_MODEL="${ROOT_DIR}/robot/tiago_robot/tiago_description/models/tiago/model.sdf"
TIAGO_URDF="${ROOT_DIR}/robot/tiago_robot/tiago_description/robots/tiago.urdf.xacro"
MAP_FILE="${MAP_FILE:-${ROOT_DIR}/map/nav_maps/warehouse.yaml}"
NAV2_BASE_PARAMS="${NAV2_BASE_PARAMS:-${ROOT_DIR}/robot/pal_navigation_cfg_public/pal_navigation_cfg_params/params/tiago_nav2.yaml}"
NAV2_OVERRIDE_PARAMS="${NAV2_PARAMS:-}"  # 可选的覆盖参数文件
TIAGO_NAME="${TIAGO_NAME:-tiago}"
HEADLESS="${HEADLESS:-0}"
RUN_RVIZ="${RUN_RVIZ:-1}"
ARM_TYPE="${ARM_TYPE:-no-arm}"
AUTO_INIT_POSE="${AUTO_INIT_POSE:-1}"
INIT_POSE_WAIT="${INIT_POSE_WAIT:-10}"

# 机器人初始位置
TIAGO_X="${TIAGO_X:-0}"
TIAGO_Y="${TIAGO_Y:-0}"
TIAGO_YAW="${TIAGO_YAW:-0}"

RESOURCE_DIRS=(
  "${ROOT_DIR}/map/turtlebot4_gz_bringup"
  "${ROOT_DIR}/robot/tiago_robot/tiago_description"
  "${ROOT_DIR}/robot/pmb2_robot/pmb2_description"
  "${ROOT_DIR}/robot/pal_urdf_utils"
  "${ROOT_DIR}/robot/pal_hey5/pal_hey5_description"
)

# 处理参数合并
if [ -n "${NAV2_OVERRIDE_PARAMS}" ] && [ -f "${NAV2_OVERRIDE_PARAMS}" ]; then
  echo "[test] 合并 Nav2 参数..."
  echo "[test]   基础: ${NAV2_BASE_PARAMS}"
  echo "[test]   覆盖: ${NAV2_OVERRIDE_PARAMS}"
  
  MERGED_PARAMS="/tmp/nav2_merged_${TEST_NAME}.yaml"
  python3 "${ROOT_DIR}/src/merge_nav2_params.py" \
    --base "${NAV2_BASE_PARAMS}" \
    --override "${NAV2_OVERRIDE_PARAMS}" \
    --output "${MERGED_PARAMS}"
  
  NAV2_PARAMS="${MERGED_PARAMS}"
  echo "[test]   输出: ${NAV2_PARAMS}"
else
  NAV2_PARAMS="${NAV2_BASE_PARAMS}"
fi

echo ""
echo "=============================================="
echo "   TIAGo 导航性能测试"
echo "=============================================="
echo "  测试名称: ${TEST_NAME}"
echo "  Waypoints: ${WAYPOINT_CFG}"
echo "  Nav2参数: ${NAV2_PARAMS}"
echo "  输出目录: ${OUTPUT_DIR}"
echo "  世界: ${WORLD_NAME}"
echo "  地图: ${MAP_FILE}"
echo "=============================================="
echo ""

# 创建输出目录
mkdir -p "${OUTPUT_DIR}"

# 检查文件存在（NAV2_PARAMS 可能是合并后的临时文件，单独检查）
for f in "${WORLD_FILE}" "${TIAGO_MODEL}" "${MAP_FILE}" "${WAYPOINT_CFG}"; do
  if [ ! -f "${f}" ]; then
    echo "[test] 错误：文件不存在：${f}" >&2
    exit 1
  fi
done

# 检查 Nav2 参数文件
if [ ! -f "${NAV2_PARAMS}" ]; then
  echo "[test] 错误：Nav2 参数文件不存在：${NAV2_PARAMS}" >&2
  exit 1
fi

# 统计 waypoints 数量
WAYPOINT_COUNT=$(grep -c "name: point_" "${WAYPOINT_CFG}" || echo "0")
echo "[test] 将测试 ${WAYPOINT_COUNT} 个目标点"

# ROS 环境
set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "${ROOT_DIR}/install/setup.bash" ]; then
  source "${ROOT_DIR}/install/setup.bash"
fi
set -u

# RMW 选择
if [ -z "${RMW_IMPLEMENTATION:-}" ] && [ -f "/opt/ros/humble/lib/librmw_cyclonedds_cpp.so" ]; then
  export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
fi

# Gazebo 资源
export GZ_SIM_RESOURCE_PATH="$(IFS=:; echo "${RESOURCE_DIRS[*]}:${GZ_SIM_RESOURCE_PATH:-}")"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"

# 软件渲染（与 launch_tiago_nav2.sh 保持一致）
if [ "${GZ_SOFT_RENDER:-0}" = "1" ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export QT_QUICK_BACKEND="${QT_QUICK_BACKEND:-software}"
  if [ "${HEADLESS}" = "1" ]; then
    export QT_XCB_GL_INTEGRATION="${QT_XCB_GL_INTEGRATION:-none}"
  fi
fi

PIDS=()
cleanup() {
  echo ""
  echo "[test] 正在关闭..."
  set +e
  
  # 先发送 SIGTERM 让进程退出
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  
  # 清理所有相关进程
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "gzserver" 2>/dev/null || true
  pkill -f "parameter_bridge" 2>/dev/null || true
  pkill -f "robot_state_publisher" 2>/dev/null || true
  pkill -f "nav2" 2>/dev/null || true
  pkill -f "rviz2" 2>/dev/null || true
  pkill -f "nav_performance_test" 2>/dev/null || true
  pkill -f "odom_to_tf" 2>/dev/null || true
  pkill -f "laser_frame_remapper" 2>/dev/null || true
  
  sleep 1
  
  # 强制杀死残留进程
  for pid in "${PIDS[@]}"; do
    kill -9 "${pid}" 2>/dev/null || true
  done
  
  echo "[test] 已退出。"
  exit 0
}

# 捕获 Ctrl+C (SIGINT)、SIGTERM 和正常退出
trap cleanup EXIT INT TERM

# 1) Gazebo
SIM_ARGS="${WORLD_FILE} -r -v 4"
if [ "${HEADLESS}" = "1" ]; then
  SIM_ARGS="${SIM_ARGS} -s --headless-rendering"
fi
echo "[test] 启动 Gazebo..."
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="${SIM_ARGS}" &
PIDS+=($!)

WAIT_TIME="${WAIT_FOR_GZ:-6}"
echo "[test] 等待 Gazebo (${WAIT_TIME}s)..."
sleep "${WAIT_TIME}"

# 2) Spawn TIAGo (根据 ARM_TYPE 选择模型)
TIAGO_MODEL_TO_USE="${TIAGO_MODEL}"
if [ "${ARM_TYPE}" = "no-arm" ] && [ -f "${ROOT_DIR}/robot/tiago_robot/tiago_description/models/tiago_no_arm/model.sdf" ]; then
  TIAGO_MODEL_TO_USE="${ROOT_DIR}/robot/tiago_robot/tiago_description/models/tiago_no_arm/model.sdf"
  echo "[test] 使用无臂版 TIAGo 模型"
else
  echo "[test] 使用标准 TIAGo 模型"
fi
echo "[test] 生成 TIAGo..."
ros2 run ros_gz_sim create \
  -name "${TIAGO_NAME}" \
  -file "${TIAGO_MODEL_TO_USE}" \
  -x "${TIAGO_X}" \
  -y "${TIAGO_Y}" \
  -z "${TIAGO_Z:-0.05}" \
  -Y "${TIAGO_YAW}"

sleep 2

# 3) Bridge - 注意：不桥接 /tf，由 odom_to_tf.py 处理
echo "[test] 启动 ROS-Gazebo 桥接..."
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock \
  /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
  /model/tiago/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
  /scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
  /joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model \
  >/tmp/tiago_test_bridge.log 2>&1 &
PIDS+=($!)

sleep 1

# 4) Robot State Publisher
ROBOT_DESC=""
if command -v xacro >/dev/null 2>&1 && [ -f "${TIAGO_URDF}" ]; then
  ROBOT_DESC=$(xacro "${TIAGO_URDF}" 2>/dev/null || echo "")
fi
if [ -z "${ROBOT_DESC}" ]; then
  ROBOT_DESC='<?xml version="1.0"?>
<robot name="tiago">
  <link name="base_footprint"/>
  <link name="base_link"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.0985" rpy="0 0 0"/>
  </joint>
  <link name="base_laser_link"/>
  <joint name="base_laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_laser_link"/>
    <origin xyz="0.202 0 -0.004" rpy="0 0 0"/>
  </joint>
</robot>'
fi
echo "[test] 启动 robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="${ROBOT_DESC}" \
  -p use_sim_time:=true \
  >/tmp/tiago_test_rsp.log 2>&1 &
PIDS+=($!)

sleep 1

# 5) Static TF 和 odom TF（与 launch_tiago_nav2.sh 保持一致）
echo "[test] 发布静态 TF..."

# 激光 frame 的静态变换
ros2 run tf2_ros static_transform_publisher \
  --ros-args -p use_sim_time:=true \
  -- 0 0 0 0 0 0 base_laser_link tiago/base_footprint/base_laser &
PIDS+=($!)

# odom → base_footprint TF
echo "[test] 启动里程计 TF 发布节点..."
python3 "${ROOT_DIR}/src/odom_to_tf.py" \
  --ros-args -p use_sim_time:=true \
  >/tmp/tiago_test_odom_tf.log 2>&1 &
PIDS+=($!)

# 激光扫描 frame 转换器
echo "[test] 启动激光 frame 转换器..."
python3 "${ROOT_DIR}/src/laser_frame_remapper.py" \
  --ros-args -p use_sim_time:=true \
  -p input_topic:=/scan \
  -p output_topic:=/scan_remapped \
  -p target_frame:=base_laser_link \
  >/tmp/tiago_test_laser.log 2>&1 &
PIDS+=($!)

sleep 3
echo "[test] TF 和激光转换节点已启动..."

# 6) Nav2
echo "[test] 启动 Nav2..."
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  autostart:=true \
  map:="${MAP_FILE}" \
  params_file:="${NAV2_PARAMS}" \
  >/tmp/tiago_test_nav2.log 2>&1 &
PIDS+=($!)

# 7) RViz
if [ "${RUN_RVIZ}" = "1" ]; then
  echo "[test] 启动 RViz..."
  RVIZ_CFG="${ROOT_DIR}/config/nav2_tiago.rviz"
  if [ ! -f "${RVIZ_CFG}" ]; then
    RVIZ_CFG="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
  fi
  ros2 run rviz2 rviz2 \
    -d "${RVIZ_CFG}" \
    --ros-args -p use_sim_time:=true \
    >/tmp/tiago_test_rviz.log 2>&1 &
  PIDS+=($!)
fi

# 8) 等待 Nav2 就绪并设置初始位姿
echo "[test] 等待 Nav2 就绪..."
sleep 5

if [ "${AUTO_INIT_POSE}" = "1" ]; then
  echo "[test] 自动设置初始位姿 (x=${TIAGO_X}, y=${TIAGO_Y}, yaw=${TIAGO_YAW})..."
  
  # 计算四元数
  YAW_RAD=$(python3 -c "import math; print(${TIAGO_YAW})")
  QZ=$(python3 -c "import math; print(math.sin(${YAW_RAD}/2))")
  QW=$(python3 -c "import math; print(math.cos(${YAW_RAD}/2))")
  
  # 多次发布确保 AMCL 接收到（不等待订阅者）
  for i in 1 2 3; do
    ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
      header: {frame_id: 'map'},
      pose: {
        pose: {
          position: {x: ${TIAGO_X}, y: ${TIAGO_Y}, z: 0.0},
          orientation: {x: 0.0, y: 0.0, z: ${QZ}, w: ${QW}}
        },
        covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.07]
      }
    }" -1 &  # -1 表示发布一次后立即退出，不等待订阅者
    sleep 2
  done
  
  echo "[test] 等待定位稳定..."
  sleep 2
else
  echo ""
  echo "[test] ================================================"
  echo "[test] 请在 RViz 中设置初始位姿 (2D Pose Estimate)"
  echo "[test] 等待 ${INIT_POSE_WAIT} 秒..."
  echo "[test] ================================================"
  sleep "${INIT_POSE_WAIT}"
fi

# 9) 开始导航性能测试
echo ""
echo "[test] ================================================"
echo "[test] 开始导航性能测试！"
echo "[test] 测试名称: ${TEST_NAME}"
echo "[test] Waypoints: ${WAYPOINT_CFG}"
echo "[test] 目标点数量: ${WAYPOINT_COUNT}"
echo "[test] 报告输出: ${OUTPUT_DIR}"
echo "[test] ================================================"
echo ""

# 使用性能测试脚本替代简单的 follow_waypoints
python3 "${ROOT_DIR}/src/nav_performance_test.py" \
  --config "${WAYPOINT_CFG}" \
  --output "${OUTPUT_DIR}" \
  --name "${TEST_NAME}" &
PIDS+=($!)

echo ""
echo "[test] 性能测试已启动，按 Ctrl+C 结束测试。"
echo ""
echo "测试完成后，报告将保存到："
echo "  YAML: ${OUTPUT_DIR}/nav_test_${TEST_NAME}.yaml"
echo "  CSV:  ${OUTPUT_DIR}/nav_test_${TEST_NAME}.csv"
echo ""
echo "日志文件："
echo "  Nav2    : /tmp/tiago_test_nav2.log"
echo "  RViz    : /tmp/tiago_test_rviz.log"
echo "  Bridge  : /tmp/tiago_test_bridge.log"
echo ""

wait "${PIDS[0]}"
