# BIT-ROS-Project

基于 ROS2 和 Nav2 的 TIAGo 机器人导航测试项目。

## 项目结构

```
├── config/                     # 配置文件
│   ├── nav2_params/           # Nav2 参数配置（多组预设）
│   │   ├── baseline.yaml      # 基准配置
│   │   ├── high_speed.yaml    # 高速配置
│   │   ├── aggressive.yaml    # 激进配置
│   │   ├── safe_narrow.yaml   # 狭窄通道安全配置
│   │   └── smooth_path.yaml   # 平滑路径配置
│   ├── controllers/           # 控制器配置
│   │   ├── dwb.yaml          # DWB 控制器
│   │   ├── rpp.yaml          # RPP 控制器
│   │   └── mppi.yaml         # MPPI 控制器
│   ├── waypoints.yaml         # 导航点配置
│   └── my_route.yaml          # 测试路线
├── map/                        # 地图和启动脚本
│   ├── batch_nav_test.sh      # 批量测试脚本
│   ├── launch_tiago_test.sh   # 单次测试脚本
│   ├── launch_tiago_nav2.sh   # 启动导航
│   └── nav_maps/              # 地图文件
├── robot/                      # 机器人描述文件
├── src/                        # 源代码
└── test_results/              # 测试结果输出
```

## 快速开始

### 1. 启动导航仿真

```bash
# 启动 TIAGo 导航（含 Gazebo 仿真 + Nav2）
./map/launch_tiago_nav2.sh
```

### 2. 运行单次导航测试

```bash
# 使用默认配置运行测试
./map/launch_tiago_test.sh

# 指定路线配置
./map/launch_tiago_test.sh config/my_route.yaml

# 指定测试名称
TEST_NAME=my_test ./map/launch_tiago_test.sh
```

## 批量测试

批量测试脚本支持同时测试**多组参数配置**和**多种控制器**。

### 基本用法

```bash
# 显示帮助
./map/batch_nav_test.sh --help

# 测试所有预设参数配置（使用默认控制器）
./map/batch_nav_test.sh

# 只测试指定的参数配置
./map/batch_nav_test.sh baseline high_speed
```

### 测试多种控制器

```bash
# 所有参数配置 × 指定控制器（笛卡尔积测试）
./map/batch_nav_test.sh --controllers dwb rpp mppi

# 只测试控制器（使用 baseline 参数）
./map/batch_nav_test.sh --controllers-only dwb rpp mppi

# 指定控制器 × 指定参数配置
./map/batch_nav_test.sh --controllers dwb rpp -- baseline high_speed
```

### 可用控制器

| 控制器 | 文件 | 特点 |
|--------|------|------|
| **DWB** | `config/controllers/dwb.yaml` | Dynamic Window B，通用、平衡性能和精度 |
| **RPP** | `config/controllers/rpp.yaml` | Regulated Pure Pursuit，简单、快速、低 CPU 消耗 |
| **MPPI** | `config/controllers/mppi.yaml` | Model Predictive Path Integral，高精度、复杂环境避障 |

### 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `HEADLESS` | `0` | 设为 `1` 启用无头模式（批量测试推荐） |
| `RUN_RVIZ` | `1` | 设为 `0` 不启动 RViz |
| `WAYPOINT_TIMEOUT` | `120` | 每个导航点的超时时间（秒） |
| `PAUSE_BETWEEN` | `10` | 测试间隔时间（秒） |
| `SKIP_EXISTING` | `0` | 设为 `1` 跳过已存在结果的测试 |

### 批量测试示例

```bash
# 无头模式批量测试（推荐用于服务器）
HEADLESS=1 RUN_RVIZ=0 ./map/batch_nav_test.sh --controllers dwb rpp mppi

# 跳过已完成的测试
SKIP_EXISTING=1 ./map/batch_nav_test.sh --controllers dwb rpp mppi

# 测试结果保存在 test_results/batch_YYYYMMDD_HHMMSS/ 目录
```

## 参数配置说明

### 预设参数配置 (`config/nav2_params/`)

- **baseline.yaml**: 保守的基准配置，用于对比
- **high_speed.yaml**: 高速配置，提高最大速度
- **aggressive.yaml**: 激进配置，更小的安全距离
- **safe_narrow.yaml**: 适合狭窄通道的安全配置
- **smooth_path.yaml**: 追求平滑轨迹的配置

### 创建自定义配置

1. 复制现有配置文件：
   ```bash
   cp config/nav2_params/baseline.yaml config/nav2_params/my_config.yaml
   ```

2. 修改参数（只需包含要修改的参数，会自动与基础配置合并）

3. 运行测试：
   ```bash
   ./map/batch_nav_test.sh my_config
   ```

## 测试结果

测试结果保存在 `test_results/` 目录：

```
test_results/
└── batch_20251205_143000/
    ├── batch_summary.yaml          # 批量测试摘要
    ├── nav_test_baseline_dwb.yaml  # 各测试详细结果
    ├── nav_test_baseline_rpp.yaml
    ├── nav_test_baseline_mppi.yaml
    └── ...
```

## 依赖

- ROS2 Humble
- Nav2
- Gazebo (Ignition)
- TIAGo 机器人仿真包
