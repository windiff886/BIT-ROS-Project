#!/usr/bin/env bash
# 一键启动 TIAGo 仿真 + SLAM 建图环境
# 功能：启动 Gazebo 仿真、导入机器人、启动控制 UI、启动 SLAM 建图、启动 RViz 可视化
# 用法：./launch_tiago_slam.sh [世界名称]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

WORLD_NAME="${1:-warehouse}"
WORLD_FILE="${ROOT_DIR}/map/turtlebot4_gz_bringup/worlds/${WORLD_NAME}.sdf"
TIAGO_MODEL="${ROOT_DIR}/src/robot/tiago_robot/tiago_description/models/tiago/model.sdf"
TIAGO_URDF="${ROOT_DIR}/src/robot/tiago_robot/tiago_description/robots/tiago.urdf.xacro"

# 资源路径
RESOURCE_DIRS=(
  "${ROOT_DIR}/map/turtlebot4_gz_bringup"
  "${ROOT_DIR}/src/robot/tiago_robot/tiago_description"
  "${ROOT_DIR}/src/robot/pmb2_robot/pmb2_description"
  "${ROOT_DIR}/src/robot/pal_urdf_utils"
  "${ROOT_DIR}/src/robot/pal_hey5/pal_hey5_description"
)

TIAGO_NAME="${TIAGO_NAME:-tiago}"
HEADLESS="${HEADLESS:-0}"

# 检查 Conda 环境
if [ -n "${CONDA_PREFIX:-}" ]; then
  echo "[slam-start] 检测到 Conda 环境 (${CONDA_PREFIX})，可能与 Qt/OpenGL 冲突。" >&2
fi

# 检查文件
if [ ! -f "${WORLD_FILE}" ]; then
  echo "[slam-start] 找不到世界文件：${WORLD_FILE}" >&2
  exit 1
fi

if [ ! -f "${TIAGO_MODEL}" ]; then
  echo "[slam-start] 找不到 TIAGo 模型：${TIAGO_MODEL}" >&2
  exit 1
fi

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
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  if [ -f "/opt/ros/humble/lib/librmw_cyclonedds_cpp.so" ]; then
    export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
  fi
fi

# Gazebo 资源路径
export GZ_SIM_RESOURCE_PATH="$(IFS=:; echo "${RESOURCE_DIRS[*]}:${GZ_SIM_RESOURCE_PATH:-}")"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"

# 软件渲染设置
if [ "${GZ_SOFT_RENDER:-0}" = "1" ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export QT_QUICK_BACKEND="${QT_QUICK_BACKEND:-software}"
  if [ "${HEADLESS}" = "1" ]; then
    export QT_XCB_GL_INTEGRATION="${QT_XCB_GL_INTEGRATION:-none}"
  fi
fi

# 进程 PID 数组
PIDS=()

cleanup() {
  echo ""
  echo "[slam-start] 正在关闭所有进程..."
  set +e
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  # 等待进程结束
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -9 "${pid}" 2>/dev/null || true
  done
  echo "[slam-start] 已关闭"
}
trap cleanup EXIT

# ============ 1. 启动 Gazebo 仿真 ============
SIM_ARGS="${WORLD_FILE} -r -v 4"
if [ "${HEADLESS}" = "1" ]; then
  SIM_ARGS="${SIM_ARGS} -s --headless-rendering"
fi
echo "[slam-start] 启动 Gazebo 仿真，世界：${WORLD_NAME}"
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="${SIM_ARGS}" &
PIDS+=($!)

WAIT_TIME="${WAIT_FOR_GZ:-6}"
echo "[slam-start] 等待 Gazebo 就绪（${WAIT_TIME}s）..."
sleep "${WAIT_TIME}"

# ============ 2. 生成 TIAGo 机器人 ============
echo "[slam-start] 生成 TIAGo 机器人..."
WORLD_BASENAME="$(basename "${WORLD_FILE}" .sdf)"
ros2 run ros_gz_sim create \
  -name "${TIAGO_NAME}" \
  -file "${TIAGO_MODEL}" \
  -x "${TIAGO_X:-0}" \
  -y "${TIAGO_Y:-0}" \
  -z "${TIAGO_Z:-0.05}" \
  -Y "${TIAGO_YAW:-0}"

sleep 2

# ============ 3. 启动 ROS-Gazebo 桥接 ============
echo "[slam-start] 启动 ROS <-> Gazebo 桥接..."
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

# ============ 4. 启动 Robot State Publisher (TF) ============
echo "[slam-start] 启动 Robot State Publisher..."

# 生成 URDF (如果有 xacro)
if command -v xacro &> /dev/null && [ -f "${TIAGO_URDF}" ]; then
  ROBOT_DESC=$(xacro "${TIAGO_URDF}" 2>/dev/null || echo "")
fi

# 如果 xacro 失败，使用简化的 URDF
if [ -z "${ROBOT_DESC:-}" ]; then
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
  <link name="odom"/>
</robot>'
fi

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="${ROBOT_DESC}" \
  -p use_sim_time:=true \
  >/tmp/tiago_rsp.log 2>&1 &
PIDS+=($!)

sleep 1

# ============ 5. 发布静态 TF (odom -> base_footprint) ============
echo "[slam-start] 发布静态 TF 变换..."

# base_footprint -> tiago/base_footprint/base_laser (激光雷达 frame)
# 这是 Gazebo 桥接产生的 frame 名称
ros2 run tf2_ros static_transform_publisher \
  --ros-args -p use_sim_time:=true \
  -- 0.202 0 0.0945 0 0 0 base_footprint tiago/base_footprint/base_laser &
PIDS+=($!)

# odom -> base_footprint (由 Gazebo odom 插件发布，这里作为备用)
ros2 run tf2_ros static_transform_publisher \
  --ros-args -p use_sim_time:=true \
  -- 0 0 0 0 0 0 odom base_footprint &
PIDS+=($!)

sleep 1

# ============ 6. 启动 SLAM Toolbox ============
echo "[slam-start] 启动 SLAM Toolbox 建图..."

# 创建 SLAM 配置文件
SLAM_CONFIG="/tmp/slam_toolbox_config.yaml"
cat > "${SLAM_CONFIG}" << 'EOF'
slam_toolbox:
  ros__parameters:
    # 基本参数
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_sim_time: true

    # 模式：mapping (建图) 或 localization (定位)
    mode: mapping

    # 地图更新参数
    map_update_interval: 2.0
    resolution: 0.05
    max_laser_range: 20.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0

    # 优化参数
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # 扫描匹配参数
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0

    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349

    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5

    use_response_expansion: true

    # 回环检测
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # 扫描缓存
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    # 位姿更新阈值
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3

    # 地图保存
    map_file_name: /tmp/tiago_map
    map_start_pose: [0.0, 0.0, 0.0]
    map_start_at_dock: true

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02

    enable_interactive_mode: true
EOF

ros2 run slam_toolbox sync_slam_toolbox_node \
  --ros-args \
  --params-file "${SLAM_CONFIG}" \
  -p use_sim_time:=true \
  >/tmp/tiago_slam.log 2>&1 &
PIDS+=($!)

sleep 2

# ============ 7. 启动 RViz 可视化 ============
echo "[slam-start] 启动 RViz 可视化..."

# 创建 RViz 配置 - 重点显示地图和激光扫描
RVIZ_CONFIG="/tmp/tiago_slam.rviz"
cat > "${RVIZ_CONFIG}" << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded: ~
      Splitter Ratio: 0.5
    Tree Height: 500
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 50
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 0; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.05
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Use Timestamp: false
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: false
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        {}
      Update Interval: 0
      Value: false
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Enabled: true
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Scale: 50
      Target Frame: <Fixed Frame>
      Value: TopDownOrtho (rviz_default_plugins)
      X: 0
      Y: 0
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 800
  Width: 1200
  X: 100
  Y: 100
EOF

ros2 run rviz2 rviz2 \
  -d "${RVIZ_CONFIG}" \
  --ros-args -p use_sim_time:=true \
  >/tmp/tiago_rviz.log 2>&1 &
PIDS+=($!)

sleep 2

# ============ 8. 启动控制 UI ============
echo "[slam-start] 启动键控 UI..."
python3 "${SCRIPT_DIR}/teleop_ui.py" >/tmp/tiago_ui.log 2>&1 &
PIDS+=($!)

# ============ 完成 ============
echo ""
echo "=========================================="
echo "[slam-start] SLAM 建图环境已启动！"
echo "=========================================="
echo ""
echo "控制方式："
echo "  - 使用弹出的 UI 窗口 (W/A/S/D 键) 控制机器人移动"
echo "  - 在 RViz 中查看实时建图结果"
echo ""
echo "保存地图（在另一个终端运行）："
echo "  cd ${SCRIPT_DIR}"
echo "  ./save_map.sh                     # 自动命名保存到项目 maps/ 目录"
echo "  ./save_map.sh my_map              # 指定名称"
echo "  ./save_map.sh my_map /path/to/dir # 指定名称和目录"
echo ""
echo "生成的文件格式："
echo "  - PGM + YAML : 标准导航地图格式"
echo "  - PNG        : 图像格式（方便查看）"
echo "  - .posegraph : SLAM 序列化文件（可继续编辑）"
echo ""
echo "日志文件："
echo "  - 桥接: /tmp/tiago_gz_bridge.log"
echo "  - SLAM: /tmp/tiago_slam.log"
echo "  - RViz: /tmp/tiago_rviz.log"
echo ""
echo "按 Ctrl+C 退出..."
echo ""

# 等待第一个进程（Gazebo）
wait "${PIDS[0]}"
