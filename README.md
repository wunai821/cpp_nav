# rm_sentinel_nav

面向哨兵赛场的导航工程，当前主线已经具备：

- 真雷达或 mock 数据输入
- `Sync -> Localization/Mapping -> Preprocess -> Costmap/MOT -> Planner -> Safety -> STM32` 主链
- 热身建图与正赛定位两套模式
- mock / replay / warmup mapping / runtime bring-up 工具
- Foxglove 最小 WebSocket 调试旁路

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

如果本机存在 Unitree L1 SDK：

- 优先读取环境变量 `UNITREE_LIDAR_SDK_ROOT`
- 检测到 SDK 头文件和静态库后，会自动打开 `unitree_sdk` 雷达源

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

- `bringup_mode: lidar_view`
  只拉起真雷达可视化最小链，方便调 `base_link / laser_link / footprint / current_scan / local_costmap`
- `manual_mode_selector: 0`
  只跑热身建图链：`MODE_WARMUP + MODE_SAVE`
- `manual_mode_selector: 1`
  只跑正赛主链：`MODE_COMBAT / GOTO_CENTER / HOLD_CENTER / RECOVERY`
- `manual_mode_selector: -1`
  让业务 FSM 自己切状态

当前仍明确标记为 roadmap / 预留接口的模块：

- `config/costmap.yaml`
- `config/mission.yaml`
- `config/mot.yaml`

这些文件保留在仓库里，是为了稳定目录结构和后续扩展接口，不代表当前主链已经完全按这些配置文件驱动。

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
- 导出建图结果

### 5. 跑真雷达 bring-up

先把 [system.yaml](config/system.yaml) 里设成：

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

输出目录由 [mapping.yaml](config/mapping.yaml) 的 `mapping.output_dir` 决定，核心产物是：

- `global_map.pcd`
- `occupancy.bin`
- `occupancy.png`
- `map_meta.json`

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
- [debug_channels.md](docs/debug_channels.md)
- [performance_budget.md](docs/performance_budget.md)
