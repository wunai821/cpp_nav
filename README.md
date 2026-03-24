# rm_sentinel_nav

面向哨兵赛场的导航工程。当前不要把“已有主链”理解成所有分层模块都已经完整落地，按模块状态看：

- `L1/IMU`：已实现
- `Sync`：已实现
- `Localization`：已实现
- `Mapping`：已实现
- `Preprocess`：部分实现
  当前已有 `range crop + self filter + ground suppression + voxel downsample` 主线，仍需继续做更强的现场调参和异常工况验证。
- `Costmap/MOT`：部分实现
  当前已有 `ObstacleLayer + InflationLayer + DynamicLayer + TrackerKf + MotManager` 主线，仍需继续补厚更强的比赛场景验证。
- `局部感知层`：部分实现
  当前已有分层预处理、静态障碍层和动态障碍层主线，但仍在继续补强语义分离和现场可视化验证。
- `Planner`：部分实现
  当前已有 `GlobalAStar + OmniDwa + GoalManager + MissionManager + RecoveryPlanner` 主线，仍需继续补强更多比赛策略细节。
- `恢复策略`：部分实现
  当前已有 `L1/L2/L3` 恢复策略链路，但仍需继续补强更多针对卡死、误匹配和复杂动态障碍的恢复动作。
- `Safety`：已实现
  当前已有独立 `SafetyManager + CommandGate + CollisionChecker + FailoverPolicy + watchdog` 主线，最终下发到底盘的命令会经过独立安全层裁决。
- `STM32 / runtime bring-up`：部分实现
  当前串口桥和一部分运行时联调已接入，但还不应视为完整实机闭环。
- `真正重定位`：部分实现
  当前已有 `RelocalizationManager`、coarse relocalization 和稳定观察窗口链路，但仍需继续补强更多实战恢复策略。
- `LIO 前端 / ESKF-LIO`：部分实现
  当前已有 `ESKF-lite / LIO-lite / 轻量回环` 主线，但还不是完整的重型后端图优化系统。
- 热身建图与正赛定位两套模式：已实现
- mock / replay / warmup mapping / runtime bring-up 工具：已实现
- Foxglove 最小 WebSocket 调试旁路：已实现

核心坐标系固定为：

- `map`
- `odom`
- `base_link`
- `laser_link`
- `imu_link`

更多架构说明见 [architecture.md](docs/architecture.md) 和 [thread_model.md](docs/thread_model.md)。

## 快速开始

### 1. 编译

```bash
cmake -S . -B build
cmake --build build
```

常用产物：

- `./rm_nav_main`
- `./mock_planner_generator`
- `./replay_player`
- `./warmup_mapping_tool`
- `./l1_driver_test`
- `./stm32_bridge_test`

如果仓库里存在 `third_party/unitree_lidar_sdk/`：

- `CMake` 会默认直接接入这份本地 Unitree L1 SDK
- `rm_nav_main` 会直接内嵌调用 SDK 读雷达和内置 IMU，不需要额外单开一个 SDK 终端
- 如需显式指定仓库内 SDK，也可以用可移植写法：

```bash
cmake -S . -B build -DUNITREE_LIDAR_SDK_ROOT=third_party/unitree_lidar_sdk
```

- 只有在 SDK 不放在仓库里时，才需要额外设置环境变量 `UNITREE_LIDAR_SDK_ROOT`

### 2. 查看版本

```bash
./rm_nav_main --version
```

### 3. 运行主程序

```bash
./rm_nav_main --config config/
```

默认配置目录是 `config/`，主程序只支持两个参数：

- `--config <dir>`
- `--version`

## 第一次拿到仓库先看什么

最值得先改的配置文件：

- [system.yaml](config/system.yaml)
- [sensors.yaml](config/sensors.yaml)
- [debug.yaml](config/debug.yaml)
- [localization.yaml](config/localization.yaml)
- [mapping.yaml](config/mapping.yaml)

最常用的模式：

- `bringup_mode: none`
  正式运行默认值；模式切换只看 FSM/`manual_mode_selector`，不额外切到可视化预设
- `bringup_mode: lidar_view`
  只在需要看真雷达时临时切换；它会拉起真雷达可视化最小链，方便调 `base_link / laser_link / footprint / current_scan / local_costmap`
- `manual_mode_selector: 0`
  只跑热身建图链：`MODE_WARMUP + MODE_SAVE`
- `manual_mode_selector: 1`
  只跑正赛主链：`MODE_COMBAT / GOTO_CENTER / HOLD_CENTER / RECOVERY`
- `manual_mode_selector: -1`
  让业务 FSM 自己切状态：若 `mapping.active_dir` 下已有 active 地图则优先走 combat，没有图则优先走 warmup
- `match_mode_enabled: true`
  比赛模式总开关；打开后建图链和正赛链都会统一等待下位机/裁判开始信号，同时启用 `HP/ammo -> 回出生点补给 -> 等待 -> 返回中心点` 的比赛行为

切运行模式时，主程序现在只需要改 [system.yaml](config/system.yaml)：

- `bringup_mode`
- `manual_mode_selector`
- `match_mode_enabled`

[mapping.yaml](config/mapping.yaml) 里的 `mapping.enabled` 和 [localization.yaml](config/localization.yaml) 里的 `localization.enabled` 现在只保留为兼容字段，不再作为 runtime 主链切模式入口。

当前仍明确标记为 roadmap / 预留接口的模块：

- `config/mission.yaml`

这个文件保留在仓库里，是为了稳定目录结构和后续扩展接口，不代表当前主链已经完全按这份配置驱动。

当前 `mapping` 也要明确一个边界：

- 已经具备热身建图闭环和地图导出/回读能力
- 热身建图默认在“轻量回环成功”时立即请求存图；若一直等不到回环，仍由 `system.auto_shutdown_ms` 保底触发 `MODE_SAVE`
- 但更稳的比赛级建图能力仍在补强中

这块后续工程拆分见 [mapping_hardening.md](docs/mapping_hardening.md)。

如果接下来要优先强化“热身只有一次机会”的建图过程，直接看：

- [mapping_hardening.md](docs/mapping_hardening.md) 里的 `第 8.6 阶段：建图稳态与轻量回环`

## 常用运行方式

### 1. 跑 mock 规划链

适合不等实机就调：

- 局部避障
- 动态障碍预测
- DWA 权重
- Center Hold

命令：

```bash
./mock_planner_generator --config config --duration-s 10 --hz 20
```

如果要指定场景：

```bash
./mock_planner_generator \
  --config config \
  --scenario-file config/mock_scenario.yaml \
  --duration-s 10 \
  --hz 20 \
  --output-dir logs/mock_planner_demo
```

常用参数：

- `--config <dir>`
- `--output-dir <dir>`
- `--scenario-file <file>`
- `--scenario cross|head_on|chase|line|mixed`
- `--duration-s <N>`
- `--hz <N>`
- `--fixed-pose`
- `--no-foxglove`
- `--foxglove-host <ip>`
- `--foxglove-port <N>`

默认输出：

- `pose.json`
- `goal.json`
- `cmd.json`
- `planner_status.json`
- `global_path.json`
- `dynamic_obstacles.json`
- `trajectory.json`
- `local_costmap.pcd`
- `static_map.pcd`
- `summary.txt`

### 2. 跑 replay

适合离线检查录包内容和单模块复现。

命令：

```bash
./replay_player logs/session.rmr
./replay_player logs/session.rmr --verbose
```

录包格式支持：

- lidar frame
- imu packet
- odom feedback
- referee state
- chassis command

协议说明见 [comm_protocol.md](docs/comm_protocol.md)。

### 3. 跑 warmup mapping 独立工具

适合热身阶段离线验证“固定路线边跑边建图”。

命令：

```bash
./warmup_mapping_tool --config config --duration-s 30 --loop-hz 8 --output-dir logs/warmup_mapping_demo
```

常用参数：

- `--config <dir>`
- `--output-dir <dir>`
- `--duration-s <N>`
- `--loop-hz <N>`

导出物：

- `global_map.pcd`
- `occupancy.bin`
- `occupancy.png`
- `map_meta.json`
- `map_validation_report.json`
  - 不只包含点数、占据栅格数、占据比例这些几何质检
  - 也包含保存前轨迹一致性检查，会直接比较 `raw external pose` 和
    `optimized pose` 在回访区域里的平移/yaw 误差均值，帮助判断轻量回环
    纠偏后轨迹是否更一致
- `summary.txt`

地图格式见 [map_format.md](docs/map_format.md)。

### 4. 用 runtime 跑热身建图

把 [mapping.yaml](config/mapping.yaml) 里的：

```yaml
mapping:
  enabled: true
```

然后运行：

```bash
./rm_nav_main --config config/
```

此时主程序会进入：

- `MODE_WARMUP`
- 固定 waypoint 巡航
- `MODE_SAVE`
- 先写 `staging`、做地图质检，再切换 `active`

### 5. 跑真雷达 bring-up

默认 [system.yaml](config/system.yaml) 是：

```yaml
bringup_mode: none
```

需要跑真雷达 bring-up 时，再临时改成：

```yaml
bringup_mode: lidar_view
```

再确认 [sensors.yaml](config/sensors.yaml) 使用真雷达：

```yaml
lidar:
  enabled: true
  source: unitree_sdk
```

启动：

```bash
sudo ./rm_nav_main --config config/
```

这个模式会优先拉起：

- 真雷达驱动
- `sync`
- `preprocess`
- `local costmap`
- 调试旁路

### 6. 跑 STM32 串口联调

```bash
./stm32_bridge_test --device /dev/ttyUSB0 --baud 115200 --duration-ms 3000
```

联调目标是先确认：

- 发心跳
- 发底盘速度命令
- 收 odom
- 收 referee

## 输出物说明

### 运行时调试目录

主程序会写：

- `logs/debug/foxglove/`
- `logs/watchdog/`
- `logs/crash/`

常见文件：

- `logs/debug/foxglove/fsm_status.json`
- `logs/debug/foxglove/fsm_event.json`
- `logs/debug/foxglove/perf_status.json`
- `logs/debug/foxglove/pose.json`
- `logs/debug/foxglove/current_scan.pcd`
- `logs/debug/foxglove/static_map.pcd`

### 热身建图导出

输出目录由 [mapping.yaml](config/mapping.yaml) 的 `mapping.active_dir` 决定，核心产物是：

- `global_map.pcd`
- `occupancy.bin`
- `occupancy.png`
- `map_meta.json`
- `map_validation_report.json`

### mock / replay / bring-up

各工具通常都会在 `logs/.../summary.txt` 留一份摘要，优先看它确认：

- 输入参数
- 运行时长
- 帧数 / 频率
- Foxglove 地址

## Foxglove 怎么接

运行 mock 或 runtime，确认 [debug.yaml](config/debug.yaml) 里：

```yaml
websocket:
  enabled: true
  host: 127.0.0.1
  port: 8765
```

然后在 Foxglove Desktop 里用 `Foxglove WebSocket` 连接：

```text
ws://127.0.0.1:8765
```

runtime 最常用 topic 见 [debug_channels.md](docs/debug_channels.md)。

## 文档索引

- [architecture.md](docs/architecture.md)
- [thread_model.md](docs/thread_model.md)
- [comm_protocol.md](docs/comm_protocol.md)
- [state_machine.md](docs/state_machine.md)
- [map_format.md](docs/map_format.md)
- [mapping_hardening.md](docs/mapping_hardening.md)
- [debug_channels.md](docs/debug_channels.md)
- [performance_budget.md](docs/performance_budget.md)
