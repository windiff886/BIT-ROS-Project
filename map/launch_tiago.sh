#!/usr/bin/env bash
# 一键在 map 仿真环境里启动 Gazebo 并导入 TIAGo 模型。
# 可选参数：第一个参数为世界名称（对应 map/turtlebot4_gz_bringup/worlds 下的 .sdf 文件名去掉扩展名），默认 warehouse。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

WORLD_NAME="${1:-warehouse}"
WORLD_FILE="${ROOT_DIR}/map/turtlebot4_gz_bringup/worlds/${WORLD_NAME}.sdf"
TIAGO_MODEL="${ROOT_DIR}/robot/tiago_robot/tiago_description/models/tiago/model.sdf"
# 额外资源路径，保证模型 mesh 可被找到
RESOURCE_DIRS=(
  "${ROOT_DIR}/map/turtlebot4_gz_bringup"
  "${ROOT_DIR}/robot/tiago_robot/tiago_description"
  "${ROOT_DIR}/robot/pmb2_robot/pmb2_description"
  "${ROOT_DIR}/robot/pal_urdf_utils"
  "${ROOT_DIR}/robot/pal_hey5/pal_hey5_description"
)
TIAGO_NAME="${TIAGO_NAME:-tiago}"
HEADLESS="${HEADLESS:-0}"  # 默认启动 GUI；无头运行请设 HEADLESS=1

if [ -n "${CONDA_PREFIX:-}" ]; then
  echo "[tiago-start] 检测到当前在 Conda 环境 (${CONDA_PREFIX})，可能与系统 Qt/OpenGL 冲突。如遇 GUI 问题，请先 'conda deactivate' 后重试。" >&2
fi

if [ ! -f "${WORLD_FILE}" ]; then
  echo "[tiago-start] 找不到世界文件：${WORLD_FILE}" >&2
  exit 1
fi

if [ ! -f "${TIAGO_MODEL}" ]; then
  echo "[tiago-start] 找不到 TIAGo 模型：${TIAGO_MODEL}" >&2
  exit 1
fi

# ROS 环境（setup 脚本在 set -u 下会报未绑定变量，临时关闭 -u）
set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck source=/opt/ros/humble/setup.bash
  source /opt/ros/humble/setup.bash
fi
USE_LOCAL_INSTALL="${USE_LOCAL_INSTALL:-0}"
if [ "${USE_LOCAL_INSTALL}" = "1" ] && [ -f "${ROOT_DIR}/install/setup.bash" ]; then
  echo "[tiago-start] 使用本地 install/ 覆盖层。"
  # shellcheck source=../install/setup.bash
  source "${ROOT_DIR}/install/setup.bash"
else
  if [ -f "${ROOT_DIR}/install/setup.bash" ]; then
    echo "[tiago-start] 检测到本地 install/，默认跳过（设置 USE_LOCAL_INSTALL=1 可启用）。"
  fi
fi
set -u

# RMW 选择：若未设置且本机有 cyclonedds 库，则默认使用；否则保持系统默认
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  if [ -f "/opt/ros/humble/lib/librmw_cyclonedds_cpp.so" ]; then
    export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
  fi
else
  if [ ! -f "/opt/ros/humble/lib/lib${RMW_IMPLEMENTATION}.so" ]; then
    echo "[tiago-start] 警告：指定的 RMW '${RMW_IMPLEMENTATION}' 未安装，已清除环境变量以使用默认。"
    unset RMW_IMPLEMENTATION
  fi
fi

# 资源路径，方便 gz 解析世界与模型
export GZ_SIM_RESOURCE_PATH="$(IFS=:; echo "${RESOURCE_DIRS[*]}:${GZ_SIM_RESOURCE_PATH:-}")"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"

# GUI/渲染设置：远程/无显卡时强制软件渲染或无头模式
if [ "${GZ_SOFT_RENDER:-0}" = "1" ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export QT_QUICK_BACKEND="${QT_QUICK_BACKEND:-software}"
  # 仅在无头模式禁用 XCB GL，GUI 模式仍保留 GLX/EGL 路径
  if [ "${HEADLESS}" = "1" ]; then
    export QT_XCB_GL_INTEGRATION="${QT_XCB_GL_INTEGRATION:-none}"
  fi
fi

SIM_ARGS="${WORLD_FILE} -r -v 4"
if [ "${HEADLESS}" = "1" ]; then
  SIM_ARGS="${SIM_ARGS} -s --headless-rendering"
fi
echo "[tiago-start] 启动 Gazebo，载入世界：${WORLD_FILE}"
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="${SIM_ARGS}" &
SIM_PID=$!

cleanup() {
  set +e
  if [ -n "${BRIDGE_PID:-}" ]; then
    kill "${BRIDGE_PID}" >/dev/null 2>&1 || true
  fi
  if [ -n "${UI_PID:-}" ]; then
    kill "${UI_PID}" >/dev/null 2>&1 || true
  fi
  kill "${SIM_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

WAIT_TIME="${WAIT_FOR_GZ:-5}"
echo "[tiago-start] 等待 Gazebo 就绪（${WAIT_TIME}s）..."
sleep "${WAIT_TIME}"

echo "[tiago-start] 在仿真中生成 TIAGo（可通过 TIAGO_X/TIAGO_Y/TIAGO_Z/TIAGO_YAW 调整）"
# 记录世界名（gz_args 里用的是文件路径，服务名使用世界名）
WORLD_BASENAME="$(basename "${WORLD_FILE}" .sdf)"
ros2 run ros_gz_sim create \
  -name "${TIAGO_NAME}" \
  -file "${TIAGO_MODEL}" \
  -x "${TIAGO_X:-0}" \
  -y "${TIAGO_Y:-0}" \
  -z "${TIAGO_Z:-0.05}" \
  -Y "${TIAGO_YAW:-0}"

# 可选：开启必要的桥接，默认启动。设置 START_BRIDGE=0 可关闭。
if [ "${START_BRIDGE:-1}" = "1" ]; then
  echo "[tiago-start] 启动 ROS <-> Gazebo 桥接（clock/cmd_vel/odom），日志：/tmp/tiago_gz_bridge.log"
  set +e
  ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock \
    /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
    /model/tiago/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
    /odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
    >/tmp/tiago_gz_bridge.log 2>&1 &
  BRIDGE_PID=$!
  set -e
fi

# 可选：启动一个简单的 Tk UI 控制 /cmd_vel（默认启动，可通过 START_UI=0 关闭）
if [ "${START_UI:-1}" = "1" ]; then
  echo "[tiago-start] 启动键控 UI 控制器（/cmd_vel）。"
  python3 "${SCRIPT_DIR}/teleop_ui.py" >/tmp/tiago_ui.log 2>&1 &
  UI_PID=$!
fi

echo "[tiago-start] 仿真运行中，按 Ctrl+C 退出。"
wait "${SIM_PID}"
