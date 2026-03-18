# 地图格式说明

本文档描述热身建图导出的 5 个核心产物，以及正赛定位如何回读它们。实现位置在 [map_serializer.cpp](../src/mapping/map_serializer.cpp)、[map_save_manager.cpp](../src/mapping/map_save_manager.cpp) 和 [map_loader.cpp](../src/localization/map_loader.cpp)。

## 导出目录

默认由 [mapping.yaml](../config/mapping.yaml) 的 `mapping.output_dir` 控制。

每次成功导图后，`active` 目录内至少包含：

- `global_map.pcd`
- `occupancy.bin`
- `occupancy.png`
- `map_meta.json`
- `map_validation_report.json`

当前保存流程不是直接覆盖写 `active`，而是：

1. 先把新图写到 `staging`
2. 对新图生成 `map_validation_report.json`
3. 校验通过后，把旧 `active` 旋转到 `last_good`
4. 再把 `staging` 切到 `active`
5. 如果校验失败，则把 `staging` 旋转到 `failed`

## `global_map.pcd`

用途：

- 正赛定位 scan matcher 使用的全局点云图
- 热身建图阶段 3D 体素地图压缩后的导出结果

当前写出的字段：

- `x`
- `y`
- `z`
- `intensity`

格式特点：

- ASCII PCD
- `HEIGHT = 1`
- 适合直接用 PCL/CloudCompare/Foxglove 做快速查看

## `occupancy.bin`

用途：

- 正赛全局 A* 和地图加载
- 作为最轻量的 2D 栅格占据数据

当前格式：

- 纯字节数组
- 大小等于 `width * height`
- 每个 cell 一个 `uint8`

当前没有把 `width/height/resolution` 写进 `bin` 本体，所以必须和 `map_meta.json` 配套使用。

## `occupancy.png`

用途：

- 人眼快速检查热身建图结果
- 不参与主链读取，只作为可视化产物

当前特点：

- 8-bit 灰度 PNG
- 导出时使用简化 PNG 写法
- 像素值通过 occupancy 反相得到，障碍越重越黑

## `map_meta.json`

用途：

- 给 `occupancy.bin` 提供尺寸和坐标信息
- 帮助正赛定位、规划回读地图

当前字段：

- `width`
- `height`
- `resolution_m`
- `origin_x_m`
- `origin_y_m`

## `map_validation_report.json`

用途：

- 建图完成后的最小质量检查报告
- 决定新图是否允许切换为当前 `active`

当前字段：

- `passed`
- `trajectory_consistency_passed`
- `global_point_count`
- `occupied_cell_count`
- `trajectory_sample_count`
- `loop_consistency_sample_count`
- `occupied_ratio`
- `raw_loop_translation_error_mean_m`
- `optimized_loop_translation_error_mean_m`
- `raw_loop_yaw_error_mean_rad`
- `optimized_loop_yaw_error_mean_rad`
- `loop_translation_gain_m`
- `loop_yaw_gain_rad`
- `width`
- `height`
- `resolution_m`
- `summary`

与轻量回环/纠偏最相关的字段：

- `trajectory_consistency_passed`
  - 轨迹一致性质检是否通过
- `raw_loop_translation_error_mean_m`
  - 回访区域里，raw external pose 相对局部匹配目标位姿的平均平移误差
- `optimized_loop_translation_error_mean_m`
  - 回访区域里，optimized pose 相对局部匹配目标位姿的平均平移误差
- `raw_loop_yaw_error_mean_rad`
  - 回访区域里，raw external pose 的平均 yaw 误差
- `optimized_loop_yaw_error_mean_rad`
  - 回访区域里，optimized pose 的平均 yaw 误差
- `loop_translation_gain_m`
  - 平移一致性增益，`raw - optimized`，越大越好
- `loop_yaw_gain_rad`
  - yaw 一致性增益，`raw - optimized`，越大越好

如果 `loop_consistency_sample_count == 0`，说明这次建图没有形成足够的回访一致性评估样本；
报告仍然有效，但这些轨迹一致性字段更应理解为“未评价”，而不是“前端一定异常”。

## 2D 占据图的语义

2D 投影来源于热身建图链：

- 只保留 `z in [z_min_m, z_max_m]`
- 低于地面阈值或过高的点不进 2D 占据图
- 分辨率由 `mapping.occupancy_resolution_m` 控制

## 正赛模块怎么使用它

### 定位

[LocalizationEngine](../src/localization/localization_engine.cpp) 会回读：

- `global_map.pcd`
- `occupancy.bin`
- `map_meta.json`

### 规划

[PlannerCoordinator](../src/planning/planner_coordinator.cpp) 的全局 A* 主要使用静态 occupancy。

## 一次完整导图后的最小验收

至少确认：

1. `global_map.pcd` 能打开并看到主结构
2. `occupancy.png` 能看清主要墙体和障碍区域
3. `map_meta.json` 的分辨率和边界看起来合理
4. 下一次启动时，正赛定位链能读这套地图
