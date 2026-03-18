# 调试通道说明

本文档描述当前通过 Foxglove WebSocket 和本地快照输出的主要调试 topic。runtime 实现在 [runtime.cpp](../src/app/runtime.cpp)，mock 实现在 [mock_planner_generator.cpp](../tools/mock_planner_generator.cpp)。

## 连接方式

配置入口在 [debug.yaml](../config/debug.yaml)：

- `websocket.enabled`
- `websocket.host`
- `websocket.port`

本机连接示例：

```text
ws://127.0.0.1:8765
```

推荐使用：

- Foxglove Desktop
- 连接类型选择 `Foxglove WebSocket`

## runtime topic

### 状态与控制

- `/rm_nav/runtime/fsm_status`
  当前状态机状态、前一状态、模式标志位
- `/rm_nav/runtime/fsm_event`
  最近一次状态转移事件
- `/rm_nav/runtime/perf_status`
  各阶段耗时、降级标志、debug 抑制状态
- `/rm_nav/runtime/safety_event`
  最近一次安全事件
- `/rm_nav/runtime/pose`
  当前位姿
- `/rm_nav/runtime/cmd`
  当前安全层输出命令

### mapping 相关

- `/rm_nav/runtime/mapping_status`
  建图帧数、累计点数、当前 waypoint
- `/rm_nav/runtime/current_waypoint`
  当前 waypoint 的简化位姿
- `/rm_nav/runtime/partial_map_scene`
  当前已累积地图的 SceneUpdate

### 感知与几何关系

- `/rm_nav/runtime/local_costmap_scene`
  局部代价图占据栅格
- `/rm_nav/runtime/footprint_scene`
  自车 footprint / self-mask 四边形
- `/rm_nav/runtime/laser_link_scene`
  雷达安装点和朝向 marker
- `/rm_nav/runtime/base_link_scene`
  车体参考点 marker
- `/rm_nav/runtime/current_scan_scene`
  当前点云 SceneUpdate

## mock topic

### 核心规划 topic

- `/rm_nav/mock/pose`
- `/rm_nav/mock/goal`
- `/rm_nav/mock/cmd`
- `/rm_nav/mock/planner_status`
- `/rm_nav/mock/global_path`
- `/rm_nav/mock/dynamic_obstacles`
- `/rm_nav/mock/trajectory`

### 可画曲线的数值 topic

- `/rm_nav/mock/metrics/dynamic_max_risk`
- `/rm_nav/mock/metrics/dynamic_integrated_risk`
- `/rm_nav/mock/metrics/dynamic_clearance_min`
- `/rm_nav/mock/metrics/dynamic_risk_05`
- `/rm_nav/mock/metrics/dynamic_risk_10`

这些 topic 的消息格式都是：

```json
{"value": 0.123}
```

适合直接放进 Foxglove `Plot` 面板。

### mock scene topic

- `/rm_nav/mock/static_map_scene`
- `/rm_nav/mock/local_costmap_scene`

## 本地快照文件

即使不连 Foxglove，程序仍然会在 `logs/debug/foxglove/` 或 mock 输出目录里留下快照：

- `pose.json`
- `cmd.json`
- `planner_status.json`
- `fsm_status.json`
- `perf_status.json`
- `current_scan.pcd`
- `static_map.pcd`

## 推荐面板搭配

### 看主链是否真的在跑

- `Raw Messages`
  - `/rm_nav/runtime/fsm_status`
  - `/rm_nav/runtime/safety_event`
  - `/rm_nav/runtime/perf_status`

### 看雷达安装和自车过滤

- `3D`
  - `/rm_nav/runtime/current_scan_scene`
  - `/rm_nav/runtime/base_link_scene`
  - `/rm_nav/runtime/laser_link_scene`
  - `/rm_nav/runtime/footprint_scene`
  - `/rm_nav/runtime/local_costmap_scene`

### 调 DWA 动态风险

- `Plot`
  - `/rm_nav/mock/metrics/dynamic_max_risk.value`
  - `/rm_nav/mock/metrics/dynamic_integrated_risk.value`
  - `/rm_nav/mock/metrics/dynamic_clearance_min.value`
  - `/rm_nav/mock/metrics/dynamic_risk_05.value`
  - `/rm_nav/mock/metrics/dynamic_risk_10.value`

## 注意事项

- SceneUpdate 是大消息，默认限频 5Hz
- `pose/cmd/perf` 这类标量 topic 默认 20Hz
- 高负载时会优先保主链，主动压掉部分点云/scene 调试通道
