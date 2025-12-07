#!/usr/bin/env bash
# 一键启动 Gazebo + TIAGo + Nav2（载入已有地图），默认带 GUI 和 RViz，并可选自动发送 waypoints。
# 可用环境变量：
#   MAP_FILE         地图 yaml 路径，默认 map/nav_maps/warehouse.yaml
#   NAV2_PARAMS      Nav2 参数文件，默认 tiago_nav2.yaml
#   WORLD_NAME       Gazebo 世界名，默认 warehouse（对应 worlds/warehouse.sdf）
#   HEADLESS         1 无头渲染，0 GUI（默认 0）
#   RUN_RVIZ         1 启动 RViz（默认 1）
#   START_UI         1 启动键控 UI（默认 1）
#   RUN_WAYPOINTS    1 自动发送 config/waypoints.yaml 中的点；0 不发送（默认 0）
#   WAYPOINT_CFG     自动发送时使用的配置文件路径
#   ARM_TYPE         机械臂类型：tiago-arm（默认）或 no-arm（无臂版）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

WORLD_NAME="${WORLD_NAME:-${1:-warehouse}}"
WORLD_FILE="${ROOT_DIR}/map/turtlebot4_gz_bringup/worlds/${WORLD_NAME}.sdf"
TIAGO_MODEL="${ROOT_DIR}/robot/tiago_robot/tiago_description/models/tiago/model.sdf"
TIAGO_URDF="${ROOT_DIR}/robot/tiago_robot/tiago_description/robots/tiago.urdf.xacro"
MAP_FILE="${MAP_FILE:-${ROOT_DIR}/map/nav_maps/warehouse.yaml}"
NAV2_PARAMS="${NAV2_PARAMS:-${ROOT_DIR}/robot/pal_navigation_cfg_public/pal_navigation_cfg_params/params/tiago_nav2.yaml}"
WAYPOINT_CFG="${WAYPOINT_CFG:-${ROOT_DIR}/config/waypoints.yaml}"
TIAGO_NAME="${TIAGO_NAME:-tiago}"
HEADLESS="${HEADLESS:-0}"
RUN_RVIZ="${RUN_RVIZ:-1}"
START_UI="${START_UI:-0}"
ARM_TYPE="${ARM_TYPE:-no-arm}"

RESOURCE_DIRS=(
  "${ROOT_DIR}/map/turtlebot4_gz_bringup"
  "${ROOT_DIR}/robot/tiago_robot/tiago_description"
  "${ROOT_DIR}/robot/pmb2_robot/pmb2_description"
  "${ROOT_DIR}/robot/pal_urdf_utils"
  "${ROOT_DIR}/robot/pal_hey5/pal_hey5_description"
)

if [ -n "${CONDA_PREFIX:-}" ]; then
  echo "[nav2-start] 检测到 Conda 环境 (${CONDA_PREFIX})，若 GUI 问题请先关闭 conda。" >&2
fi

for f in "${WORLD_FILE}" "${TIAGO_MODEL}" "${MAP_FILE}" "${NAV2_PARAMS}"; do
  if [ ! -f "${f}" ]; then
    echo "[nav2-start] 文件不存在：${f}" >&2
    exit 1
  fi
done

# ROS 环境
set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
# 默认启用本地 install 覆盖层（无臂模式需要）
if [ -f "${ROOT_DIR}/install/setup.bash" ]; then
  echo "[nav2-start] 加载本地 install/ 覆盖层。"
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

# 软件渲染
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
  echo "[nav2-start] 正在关闭..."
  set +e
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -9 "${pid}" 2>/dev/null || true
  done
  echo "[nav2-start] 已退出。"
}
trap cleanup EXIT

# 1) Gazebo
SIM_ARGS="${WORLD_FILE} -r -v 4"
if [ "${HEADLESS}" = "1" ]; then
  SIM_ARGS="${SIM_ARGS} -s --headless-rendering"
fi
echo "[nav2-start] 启动 Gazebo：${WORLD_NAME}"
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="${SIM_ARGS}" &
PIDS+=($!)

WAIT_TIME="${WAIT_FOR_GZ:-15}"
echo "[nav2-start] 等待 Gazebo (${WAIT_TIME}s)..."
sleep "${WAIT_TIME}"

# 2) Spawn TIAGo (根据 ARM_TYPE 选择模型)
TIAGO_MODEL_TO_USE="${TIAGO_MODEL}"
if [ "${ARM_TYPE}" = "no-arm" ] && [ -f "${ROOT_DIR}/robot/tiago_robot/tiago_description/models/tiago_no_arm/model.sdf" ]; then
  TIAGO_MODEL_TO_USE="${ROOT_DIR}/robot/tiago_robot/tiago_description/models/tiago_no_arm/model.sdf"
  echo "[nav2-start] 使用无臂版 TIAGo 模型"
else
  echo "[nav2-start] 使用标准 TIAGo 模型"
fi
# 固定初始位置（仓库中心，与 AMCL 初始位姿一致）
TIAGO_INIT_X=0.0
TIAGO_INIT_Y=0.0
TIAGO_INIT_Z=0.05
TIAGO_INIT_YAW=0.0

echo "[nav2-start] 生成 TIAGo 于位置 (${TIAGO_INIT_X}, ${TIAGO_INIT_Y}, yaw=${TIAGO_INIT_YAW})..."
echo "[nav2-start] 执行: ros2 run ros_gz_sim create -name ${TIAGO_NAME} -file ... -x ${TIAGO_INIT_X} -y ${TIAGO_INIT_Y} -z ${TIAGO_INIT_Z} -Y ${TIAGO_INIT_YAW}"
ros2 run ros_gz_sim create \
  -name "${TIAGO_NAME}" \
  -file "${TIAGO_MODEL_TO_USE}" \
  -x "${TIAGO_INIT_X}" \
  -y "${TIAGO_INIT_Y}" \
  -z "${TIAGO_INIT_Z}" \
  -Y "${TIAGO_INIT_YAW}" \
  && echo "[nav2-start] 机器人生成成功" \
  || echo "[nav2-start] 警告：机器人生成可能失败"

sleep 2

# 3) Bridge
echo "[nav2-start] 启动 ROS<->Gazebo 桥接..."
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock \
  /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
  /model/tiago/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
  /scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
  /tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V \
  /joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model \
  >/tmp/tiago_gz_bridge.log 2>&1 &
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
echo "[nav2-start] 启动 robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="${ROBOT_DESC}" \
  -p use_sim_time:=true \
  >/tmp/tiago_rsp.log 2>&1 &
PIDS+=($!)

sleep 1

# 5) Static TF
echo "[nav2-start] 发布静态 TF..."

# 激光 frame 的静态变换
ros2 run tf2_ros static_transform_publisher \
  --ros-args -p use_sim_time:=true \
  -- 0 0 0 0 0 0 base_laser_link tiago/base_footprint/base_laser &
PIDS+=($!)

# 激光扫描 frame 转换器
echo "[nav2-start] 启动激光 frame 转换器..."
python3 "${ROOT_DIR}/src/laser_frame_remapper.py" \
  --ros-args -p use_sim_time:=true \
  -p input_topic:=/scan \
  -p output_topic:=/scan_remapped \
  -p target_frame:=base_laser_link \
  >/tmp/tiago_laser_remap.log 2>&1 &
PIDS+=($!)

sleep 3
echo "[nav2-start] TF 和激光转换节点已启动..."

# 6) Nav2 - 使用重映射的激光话题
echo "[nav2-start] 启动 Nav2 bringup..."
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  autostart:=true \
  map:="${MAP_FILE}" \
  params_file:="${NAV2_PARAMS}" \
  >/tmp/tiago_nav2.log 2>&1 &
PIDS+=($!)

# 7) 可选：自动发送 waypoints
if [ "${RUN_WAYPOINTS:-0}" = "1" ]; then
  echo "[nav2-start] 等待 Nav2 就绪后发送路径：${WAYPOINT_CFG}"
  sleep 5
  python3 "${ROOT_DIR}/src/follow_waypoints.py" --config "${WAYPOINT_CFG}" >/tmp/tiago_waypoints.log 2>&1 &
  PIDS+=($!)
fi

# 8) RViz
if [ "${RUN_RVIZ:-1}" = "1" ]; then
  echo "[nav2-start] 启动 RViz..."
  RVIZ_CFG="${ROOT_DIR}/config/nav2_tiago.rviz"
  if [ ! -f "${RVIZ_CFG}" ]; then
    RVIZ_CFG="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
  fi
  ros2 run rviz2 rviz2 \
    -d "${RVIZ_CFG}" \
    --ros-args -p use_sim_time:=true \
    >/tmp/tiago_rviz.log 2>&1 &
  PIDS+=($!)
fi

# 9) 可选：键控 UI
if [ "${START_UI:-1}" = "1" ]; then
  echo "[nav2-start] 启动键控 UI (WASD)..."
  python3 "${SCRIPT_DIR}/teleop_ui.py" >/tmp/tiago_ui.log 2>&1 &
  PIDS+=($!)
fi

echo ""
echo "======================================="
echo "[nav2-start] 已启动 Gazebo + Nav2。"
echo "  世界: ${WORLD_NAME}"
echo "  地图: ${MAP_FILE}"
echo "  参数: ${NAV2_PARAMS}"
echo "---------------------------------------"
echo "下一步："
echo "  1) 在 RViz 发布初始位姿 /initialpose（或使用命令行）。"
echo "  2) 然后在需要时运行 python3 src/follow_waypoints.py --config config/waypoints.yaml"
echo "     如果已设置 RUN_WAYPOINTS=1，会自动发送。"
echo "日志："
echo "  桥接    : /tmp/tiago_gz_bridge.log"
echo "  RSP     : /tmp/tiago_rsp.log"
echo "  OdomTF  : /tmp/tiago_odom_tf.log"
echo "  LaserMap: /tmp/tiago_laser_remap.log"
echo "  Nav2    : /tmp/tiago_nav2.log"
echo "  RViz    : /tmp/tiago_rviz.log"
echo "  UI      : /tmp/tiago_ui.log"
echo "======================================="
echo ""
echo "按 Ctrl+C 退出并清理。"
echo ""

wait "${PIDS[0]}"
