# 性能预算说明

本文档描述当前工程针对四线程 `i5-6500` 这一类机器所采用的性能预算、限频和降级策略。实现集中在 [runtime.cpp](../src/app/runtime.cpp)。

## 优先级原则

第一优先级始终是主链：

- 驱动
- sync
- localization matcher
- preprocess / costmap / MOT
- planner
- safety

第二优先级才是 debug：

- Foxglove SceneUpdate
- JSON 编码
- 日志刷盘

## 当前重点采样指标

runtime 已经会统计并输出这些指标：

- `driver_cpu_usage_milli`
- `sync_process_latency_ns`
- `sync_preintegration_latency_ns`
- `sync_deskew_latency_ns`
- `localization_latency_ns`
- `matcher_latency_ns`
- `mot_clustering_latency_ns`
- `mot_total_latency_ns`
- `planner_latency_ns`
- `safety_tick_latency_ns`
- `foxglove_publish_latency_ns`
- `debug_encode_latency_ns`
- `localization_light_mode`
- `mot_reduced_roi_active`
- `debug_scene_suppressed`

它们会被发布到：

- `/rm_nav/runtime/perf_status`

并写入：

- `logs/debug/foxglove/perf_status.json`

## 默认发布预算

配置入口在 [debug.yaml](../config/debug.yaml)。

当前默认策略：

- `publish_hz: 10`
- `pointcloud_publish_hz: 5`
- `scalar_publish_hz: 20`
- `high_load_debug_threshold_ms: 8`
- `drop_pointcloud_on_high_load: true`

含义：

- 点云和 SceneUpdate 默认最多 5Hz
- `pose/cmd/perf` 等轻量消息默认最多 20Hz
- 单次 debug 发布超过 8ms 时，认为 debug 已开始挤压主链
- 高负载时优先关掉大消息，不让 Foxglove 卡住控制链

## 当前安全/恢复相关预算

配置入口：

- [safety.yaml](../config/safety.yaml)
- [localization.yaml](../config/localization.yaml)

关键参数：

- `deadman_timeout_ms`
- `collision_check_lookahead_s`
- `collision_check_dt_s`
- `recovery_speed_scale`
- `recovery_yaw_scale`
- `slow_match_threshold_ms`
- `reduced_max_iterations`
- `reduced_scan_stride`

## 降级策略

### debug 降级

- SceneUpdate 默认低频
- debug 发布超预算时，压掉点云/scene
- 标量消息尽量保留

### localization 降级

- matcher 慢帧触发 `light_match_mode`
- 减少迭代次数
- 增大 scan stride，降低单帧点数
- 连续失败达到阈值后切到基础 relocalization 预测

### perception 降级

- MOT 在超预算时可缩 ROI
- `mot_reduced_roi_active` 会写入 perf 状态

### safety 优先级

任何时候：

- `deadman`
- `static collision`
- `dynamic collision`
- `failsafe override`

都优先于调试输出。

## 现场调优建议

### 先看有没有 debug 抢主链

如果出现：

- `debug_scene_suppressed = true`
- `foxglove_publish_latency_ns` 明显升高

优先处理：

- 降低 `pointcloud_publish_hz`
- 关闭不必要的 SceneUpdate
- 只保留 `pose/cmd/perf/safety_event`

### matcher 太慢

如果 `matcher_latency_ns` 过高：

- 降低 `reduced_max_iterations`
- 增大 `reduced_scan_stride`
- 暂时不要打开更多 map/scene 调试

### MOT 太慢

如果 `mot_clustering_latency_ns` 过高：

- 缩小聚类 ROI
- 减少不必要的动态目标调试输出

### safety tick 必须稳定

`safety_tick_latency_ns` 应该一直很小。它一旦明显抖动，优先排查：

- debug 编码
- 阻塞日志
- 不必要的锁竞争

## 当前目标

这套预算不是为了跑最复杂算法，而是为了保证：

- 主链不因 Foxglove 卡住
- 失败模式可控
- 四线程机器上能稳定拉起并持续运行
