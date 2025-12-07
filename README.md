# 仓库管理员 (Warehouse Runner)

## 文件系统概览图（目录树）
```
BIT-ROS-Project/
├─ map/                     仿真世界、导航地图、一键脚本、批量测试
│  ├─ turtlebot4_gz_bringup/worlds/warehouse.sdf
│  ├─ nav_maps/warehouse.{pgm,yaml}
│  ├─ launch_tiago*.sh, launch_tiago_nav2.sh, launch_tiago_slam.sh
│  ├─ launch_tiago_test.sh, batch_nav_test.sh, teleop_ui.py, save_map.sh
├─ config/                  Nav2 参数与行为树、路径点、RViz 视图
│  ├─ nav2_params/*.yaml
│  ├─ behavior_trees/
│  ├─ waypoints.yaml, my_route.yaml, nav2_tiago.rviz
├─ src/                     自定义节点与工具脚本
│  ├─ laser_frame_remapper.py, odom_to_tf.py, follow_waypoints.py
│  ├─ merge_nav2_params.py, nav_performance_test.py, test_navigation_performance.py
├─ robot/                   TIAGo/PMB2 模型与官方导航参数
│  ├─ pal_navigation_cfg_public/.../tiago_nav2.yaml
│  ├─ tiago_robot/tiago_description/models/{tiago,tiago_no_arm}/
│  ├─ tiago_simulation/, tiago_navigation/, pmb2_robot/
├─ report/report.tex        项目报告
├─ visualize_waypoints.py, waypoints_visualization.png
├─ build/, install/, log/, test_results/    运行产物
└─ LICENSE, README.md
```

+ Nav2运行视频：录屏 2025年12月07日 20时12分28秒.webm
+ BT yaml配置文件： config/behavior_trees
+ 斯坦利自定义控制器路径： src/custom_controller
+ 自定义控制器说明文档： 见研习报告
+ 研习报告： report/report.pdf
+ Nav2与控制器批量测试结果路径： test_results
+ 启动Nav2： bash map/launch_tiago_nav2.sh
+ 进行Nav2参数的批量测试： bash map/batch_nav_test.sh
+ 进行多种控制器的批量测试： bash map/batch_nav_test.sh --controllers dwb rpp mppi stanley -- baseline
+ 如何使用自定义BT： 在robot/pal_navigation_cfg_public/pal_navigation_cfg_params/params/tiago_nav2.yaml路径下，将default_bt_xml_filename: "config/behavior_trees/dynamic_multi_controller.xml"解除注释，将default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"注释（83，84行），再直接启动bash map/launch_tiago_nav2.sh即可

## map 目录脚本详解

**launch_tiago_nav2.sh（导航一键启动）**
- 作用：启动 Gazebo + TIAGo + Nav2（载入地图）；可选自动发送 waypoints、RViz、键控 UI。
- 核心环境变量：`MAP_FILE` 地图 yaml；`NAV2_PARAMS` Nav2 参数；`WORLD_NAME` 世界名；`HEADLESS` 无头；`RUN_RVIZ`；`START_UI`；`RUN_WAYPOINTS`；`WAYPOINT_CFG` 路径点文件；`ARM_TYPE` tiago-arm/no-arm。
- 用法示例：`HEADLESS=1 RUN_WAYPOINTS=1 WAYPOINT_CFG=config/waypoints.yaml ./map/launch_tiago_nav2.sh`。

**launch_tiago_slam.sh（建图启动）**
- 作用：启动仿真 + 桥接 + SLAM Toolbox + RViz，用于建图。
- 环境/参数：位置参数为世界名；`HEADLESS=1` 无头；`USE_LOCAL_INSTALL=1` 使用本地 install 覆盖；`GZ_SOFT_RENDER=1` 软件渲染。
- 建图完成后执行：`./map/save_map.sh my_map` 生成 pgm/yaml/png/序列化。

**launch_tiago.sh（最小仿真）**
- 作用：仅启动 Gazebo 并生成 TIAGo，可选桥接和键控 UI。
- 环境：`WORLD_NAME`，`HEADLESS`，`START_BRIDGE`，`START_UI`。
- 示例：`HEADLESS=1 START_UI=0 ./map/launch_tiago.sh`。

**launch_tiago_test.sh（单次导航测试）**
- 作用：封装一次导航测试，支持参数/控制器多层合并（基线 + 覆盖 + 控制器）。
- 环境：`WAYPOINT_CFG`，`TEST_NAME`，`MAP_FILE`，`NAV2_PARAMS` 覆盖，`NAV2_CONTROLLER_PARAMS` 控制器覆盖，`HEADLESS`，`RUN_RVIZ`，`AUTO_INIT_POSE`，`INIT_POSE_WAIT`，`WAYPOINT_TIMEOUT`。
- 流程：合并参数 → 启动仿真/桥接/TF → Nav2 → 自动或手动初始位姿 → 跑 waypoints，日志在 `/tmp/tiago_test_*.log`。

**batch_nav_test.sh（批量测试调度）**
- 作用：遍历参数/控制器组合，循环调用 `launch_tiago_test.sh`，结果写入 `test_results/batch_*/`。
- 关键参数：`--controllers`/`--controllers-only`（dwb/rpp/mppi）；配置名列表；`HEADLESS`；`RUN_RVIZ`；`WAYPOINT_TIMEOUT`；`PAUSE_BETWEEN`；`SKIP_EXISTING`；`OUTPUT_DIR`。
- 示例：`HEADLESS=1 RUN_RVIZ=0 ./map/batch_nav_test.sh --controllers dwb rpp mppi -- baseline high_speed`。

**save_map.sh（保存建图结果）**
- 作用：保存 SLAM 地图为 pgm/yaml/png 与 SLAM 序列化文件。
- 用法：`./map/save_map.sh my_map`（默认保存到 `maps/`）。

**其他**
- `launch_tiago_nav2.sh`、`launch_tiago_test.sh` 内部会启动 `teleop_ui.py`（若 `START_UI=1`），用于键控；`batch_nav_test.sh` 会复用上述脚本并做强力清理。

