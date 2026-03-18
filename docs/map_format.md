# 地图格式说明

本文档描述热身建图导出的 4 个核心产物，以及正赛定位如何回读它们。实现位置在 [map_serializer.cpp](../src/mapping/map_serializer.cpp) 和 [map_loader.cpp](../src/localization/map_loader.cpp)。

## 导出目录

默认由 [mapping.yaml](../config/mapping.yaml) 的 `mapping.output_dir` 控制。

每次成功导图后，目录内至少包含：

- `global_map.pcd`
- `occupancy.bin`
- `occupancy.png`
- `map_meta.json`

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
