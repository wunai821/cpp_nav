# 建图稳健化任务包

本文档把“热身建图链已可用，但还不够比赛现场稳建图”这件事，收成一组可以直接排期执行的工程任务包。当前相关实现主要位于 [mapping](../src/mapping/)、[sync](../src/sync/)、[localization](../src/localization/) 和 [app/runtime.cpp](../src/app/runtime.cpp)。

## 当前判断

当前建图链已经具备：

- `SyncedFrame -> MappingEngine -> 3D voxel map -> 2D occupancy`
- `warmup_mapping_tool`
- runtime 内 `MODE_WARMUP -> MODE_SAVE`
- 地图导出与正赛回读

当前还不够比赛级稳健的部分主要是：

- 前端仍偏轻量，缺少更稳的 `IMU + LiDAR` 紧耦合或准紧耦合能力
- 没有真正的回环/漂移抑制链
- 建图完成后的地图质量检查还比较薄
- 地图保存、版本管理、失败回退还不够硬
- 现场建图失败后，主程序缺少明确的降级与回退路径

所以这块的目标不是“再多堆模块名”，而是把现有热身建图链从“能导出地图”推进到“能在比赛现场更稳定地导出可信地图”。

## 任务包拆分

### 包 A：前端增强

目标：

- 提升热身建图时的短时姿态稳定性
- 降低快速转向、短时遮挡、加减速带来的局部扭曲

建议范围：

- 在当前 [imu_preintegrator.cpp](../src/sync/imu_preintegrator.cpp) 基础上，补一个更稳的前端姿态预测链
- 第一版优先做 `ESKF-lite` 或 `LIO-lite`，不要一上来做复杂大一统系统
- 保持主链输出仍是当前的 `SyncedFrame / Pose3f / map->odom` 兼容接口

最小交付：

- 新前端可以替换当前 mapping pose 预测
- 有开关可回退到当前轻量前端
- 能记录前端质量状态和失败计数

验收：

1. 固定路线走同一圈，轨迹平滑度优于当前版本
2. 快速转向时局部地图拉丝减轻
3. 雷达短时失真时，姿态不会立刻跳飞

不做：

- 不先做完整 3D SLAM 框架
- 不先做大而全多传感器融合平台

### 包 B：漂移抑制与轻量回环

目标：

- 降低一圈热身后的整体漂移
- 在重复经过明显结构区域时给出轻量纠偏

建议范围：

- 先做局部一致性约束或回访区域匹配
- 先支持“固定路线热身建图”场景，不先做通用 frontier 探索回环
- 回环命中后只做温和校正，不做高风险大跳变

最小交付：

- 回访区域检测
- 轻量匹配分数
- 校正幅度上限
- 连续失败时自动关闭回环修正

验收：

1. 同一路线闭环终点与起点偏差明显下降
2. 回环失败时不会把地图整体拉坏
3. 触发与拒绝原因可以在日志中看见

不做：

- 不先做复杂 pose graph 优化平台
- 不先做全图大规模后端优化

### 包 C：地图质量检查

目标：

- 建图结束后先判图，再决定是否切到可用地图
- 避免把明显坏图直接送进正赛定位

建议检查项：

- `global_map.pcd` 点数下限
- 占据图覆盖率
- 地图边界是否异常膨胀
- 热身轨迹连续性
- 导出文件完整性
- 地图原点、分辨率、尺寸自洽性

最小交付：

- 一个 `map_validation_report.json`
- 一个明确的 `pass / warn / fail`
- runtime 的 `MODE_SAVE` 只在校验通过时切 `active`

验收：

1. 故意给一组坏图时，校验能明确拦下
2. 正常热身图不会被误判太多
3. 校验结果能被主程序和人工同时读取

不做：

- 不先做图像级复杂语义分析
- 不先做人工智能式地图打分器

### 包 D：单地图保存与失败隔离

目标：

- 保存过程可恢复
- 校验失败时有隔离目录
- 单地图模式下不引入“上一张图自动回退”的额外语义

建议目录语义：

- `maps/warmup/active`
- `maps/warmup/staging`
- `maps/warmup/failed`

最小交付：

- 保存先写 `staging`
- 校验通过后原子切换到 `active`
- 校验失败或保存失败时写入 `failed`

验收：

1. 保存中断后不破坏 `active`
2. 新图不合格时不会污染 `active`
3. 地图元信息里可看到版本和生成时间

不做：

- 不先做复杂数据库式地图仓库
- 不先做跨机器同步系统

### 包 E：失败恢复与模式回退

目标：

- 热身建图失败时，系统行为要可控
- 不把“建图失败”直接等价成“整车不可用”

建议范围：

- `MODE_WARMUP` 期间前端质量持续下降时降速
- `MODE_SAVE` 失败时不切坏图
- 如果所有地图都不可用，明确停在 `IDLE / FAILSAFE`，不要假装进入 `MODE_COMBAT`

最小交付：

- 建图失败事件码
- 地图保存失败事件码
- 地图校验失败事件码
- FSM 明确转移路径

验收：

1. 建图失败不会覆盖旧图
2. 无图时状态机停在可解释状态
3. 失败目录里能保留坏图用于排障

不做：

- 不先做复杂比赛战术层联动

## 推荐执行顺序

建议顺序：

1. 包 C：地图质量检查
2. 包 D：地图版本管理与回退
3. 包 E：失败恢复与模式回退
4. 包 A：前端增强
5. 包 B：漂移抑制与轻量回环

这样排的原因很直接：

- 先保证“坏图不会进主链”
- 再保证“失败时有回退”
- 然后再提升“建图本身更稳”

## 每包建议触达文件

### 包 A

- [imu_preintegrator.cpp](../src/sync/imu_preintegrator.cpp)
- [sensor_sync_buffer.cpp](../src/sync/sensor_sync_buffer.cpp)
- [mapping_engine.cpp](../src/mapping/mapping_engine.cpp)
- [mapping.yaml](../config/mapping.yaml)

### 包 B

- [mapping_engine.cpp](../src/mapping/mapping_engine.cpp)
- [map_builder_3d.cpp](../src/mapping/map_builder_3d.cpp)
- [runtime.cpp](../src/app/runtime.cpp)

### 包 C

- [map_serializer.cpp](../src/mapping/map_serializer.cpp)
- 新增 `map_validator.*`
- [map_format.md](./map_format.md)

### 包 D

- [map_serializer.cpp](../src/mapping/map_serializer.cpp)
- [config/mapping.yaml](../config/mapping.yaml)
- [runtime.cpp](../src/app/runtime.cpp)

### 包 E

- [nav_fsm.cpp](../src/fsm/nav_fsm.cpp)
- [runtime.cpp](../src/app/runtime.cpp)
- [state_machine.md](./state_machine.md)

## 阶段完成定义

可以把这块定义成一个单独阶段：

- `第 8.5 阶段：建图稳健化`

完成标准建议定成：

1. 热身建图失败不会覆盖 `active`
2. 新图导出后有明确质检结果
3. 失败图会被隔离到 `failed`
4. 现场固定路线重复建图的一致性明显提升
5. FSM 能正确处理建图失败、保存失败、地图不可用

## 当前建议

如果只选一件最值得马上做的事，优先做：

- 包 C + 包 D

因为它们最直接决定“这张图能不能安全进正赛主链”，而且改动风险比直接上更重的前端融合要小。

## 第 8.6 阶段：建图稳态与轻量回环

这一阶段的目标不是做完整 SLAM 后端，而是把“热身只有一次机会”的风险压下去。核心思路是：

- 只服务固定热身路线
- 只做小幅纠偏
- 只在高置信条件下接受回环修正
- 一旦条件不好就宁可不用，不强行闭环

### 8.6.1 先补建图状态缓存

当前状态：

- 已完成最小实现：`MappingEngine` 已支持关键帧缓存、按位移/yaw 触发、固定容量 FIFO 淘汰，以及每帧局部点裁剪。

目标：

- 给后续回访检测和轻量回环提供最小上下文

实现任务：

1. 在 `MappingEngine` 内增加关键帧缓存结构
2. 每隔固定距离或角度记录一个 mapping keyframe
3. 每个 keyframe 至少保存：
   - `map_to_base`
   - 下采样后的局部点云
   - 时间戳
   - 帧序号
4. 给 keyframe 缓存加上最大数量和淘汰策略

建议文件：

- [mapping_engine.hpp](../include/rm_nav/mapping/mapping_engine.hpp)
- [mapping_engine.cpp](../src/mapping/mapping_engine.cpp)

完成标准：

- 能稳定缓存一条热身路线上的关键帧
- 内存占用可控，不无限增长

### 8.6.2 做回访区域检测

当前状态：

- 已完成最小实现：当前已新增 `LoopCandidateDetector`，基于平面距离、时间间隔、航向差对历史 keyframe 做粗筛，并输出最近的回访候选。
- 当前边界：这一版只做候选检测，不做局部匹配与轨迹修正；真正的回环接受仍在后续 `8.6.3 / 8.6.4`。

目标：

- 找到“当前帧和之前某段路线很像”的候选回环点

实现任务：

1. 基于当前位置与历史 keyframe 做粗筛
2. 粗筛条件至少包含：
   - 平面距离阈值
   - 时间间隔阈值
   - 航向差阈值
3. 过滤掉过近的临近帧，避免把局部连续帧误判成回环
4. 输出回环候选 `candidate_id`

建议初值：

- 距离阈值：`0.6 ~ 1.5 m`
- 最小时间间隔：`5 ~ 10 s`
- 航向差：`< 35 deg`

建议文件：

- 新增 `loop_candidate_detector.*`
- [mapping_engine.cpp](../src/mapping/mapping_engine.cpp)
- [mapping.yaml](../config/mapping.yaml)

完成标准：

- 固定路线走一圈时，能稳定给出少量合理候选
- 不会每几帧都误触发一次

### 8.6.3 做轻量局部匹配

当前状态：

- 已完成最小实现：当前已新增 `LoopClosureMatcher`，会拿当前帧与 `LatestLoopCandidate()` 指向的 keyframe 局部点云做轻量局部匹配。
- 当前输出：已可产出匹配得分、迭代次数、平移修正量、yaw 修正量，以及本次匹配是否收敛。
- 当前边界：这一步还只做“匹配并输出结果”，暂不把修正注入建图主轨迹；真正接入 correction 留给后续 `8.6.4 / 8.6.5`。

目标：

- 对候选回环点做一次小范围局部匹配，判断是否可接受

实现任务：

1. 用当前帧或当前局部子图，与候选 keyframe 局部地图做匹配
2. 第一版直接复用现有轻量 ICP/NDT 接口，不再新造 matcher
3. 匹配输入尽量下采样，保证算力稳定
4. 输出至少包含：
   - 匹配得分
   - 迭代次数
   - 估计平移量
   - 估计 yaw 修正量

建议文件：

- [icp_matcher.cpp](../src/localization/icp_matcher.cpp)
- [ndt_matcher.cpp](../src/localization/ndt_matcher.cpp)
- 新增 `loop_closure_matcher.*`

完成标准：

- 候选命中时能给出一组稳定匹配结果
- 单次匹配耗时受控，不明显拖垮 mapping loop

### 8.6.4 做“小修正接受器”

当前状态：

- 已完成最小实现：当前已新增 `LoopCorrectionGate`，会基于匹配得分、平移修正量、yaw 修正量和连续失败计数，对轻量局部匹配结果做接受/拒绝判定。
- 当前输出：`MappingEngine` 已可产出最新的 `LoopCorrectionDecision`，明确给出 accepted/rejected、拒绝原因和连续失败计数。
- 当前边界：这一版还只做 correction 门控，不把修正真正注入建图主轨迹；真正的 correction 生效留给后续 `8.6.5`。

目标：

- 只接受“可信且幅度小”的回环修正，避免一把把图拉坏

实现任务：

1. 增加回环接受判据：
   - 分数高于阈值
   - 平移修正小于阈值
   - yaw 修正小于阈值
   - 连续失败计数不过高
2. 只允许小修正进入主链
3. 修正采用平滑注入，不做瞬时大跳变
4. 若拒绝，则记录原因，不改变当前轨迹

建议初值：

- 平移修正上限：`0.15 ~ 0.30 m`
- yaw 修正上限：`5 ~ 10 deg`

建议文件：

- 新增 `loop_correction_gate.*`
- [mapping_engine.cpp](../src/mapping/mapping_engine.cpp)

完成标准：

- 命中坏候选时不会把地图整体扭坏
- 命中好候选时终点漂移能明显下降

### 8.6.5 把修正接入 mapping pose

当前状态：

- 已完成最小实现：`MappingEngine` 当前已维护“目标 correction”和“已应用 correction”，新帧进入建图前会先叠加平滑后的 correction。
- 当前注入方式：gate 接受后不会立刻整帧跳变，而是按每帧平移/yaw 步长上限，逐步把 correction 注入后续 mapping pose。
- 当前边界：这一版仍是轻量的增量修正，不做历史轨迹回放，也不做全图重优化。

目标：

- 让轻量回环真正作用到建图轨迹，但保持系统可控

实现任务：

1. 在 `MappingEngine` 内维护一条“mapping pose correction”
2. 新帧位姿进入建图前先叠加 correction
3. 回环命中时只更新 correction，不回放全历史
4. 对 correction 更新做速率限制

建议文件：

- [mapping_engine.hpp](../include/rm_nav/mapping/mapping_engine.hpp)
- [mapping_engine.cpp](../src/mapping/mapping_engine.cpp)

完成标准：

- 地图后半段能够受回环修正收益
- 不引入大跳变和连续抖动

### 8.6.6 做失败保护

目标：

- 轻量回环一旦不靠谱，就自动降级回纯建图，不影响主流程

实现任务：

1. 增加连续回环失败计数
2. 超阈值后暂时禁用回环若干秒
3. mapping loop 超预算时自动跳过本次回环匹配
4. 输出回环状态：
   - enabled
   - candidate_found
   - accepted
   - rejected_reason
   - cooldown_active

建议文件：

- [mapping_engine.cpp](../src/mapping/mapping_engine.cpp)
- [performance_budget.md](./performance_budget.md)

完成标准：

- 回环模块出问题时，建图主流程仍能继续
- 不因回环而把整车拖死

### 8.6.7 做最小可视化和日志

目标：

- 现场能快速判断“为什么没闭环”或“为什么拒绝修正”

实现任务：

1. 在 debug 输出中增加：
   - `loop_closure_status.json`
   - `loop_candidate.json`
2. 状态至少包含：
   - 当前候选 id
   - 匹配分数
   - 平移修正
   - yaw 修正
   - 接受/拒绝原因
3. 如果 Foxglove 开启，可再加一个轻量 topic：
   - `/rm_nav/runtime/loop_closure_status`

建议文件：

- [runtime.cpp](../src/app/runtime.cpp)
- [debug_channels.md](./debug_channels.md)

完成标准：

- 不看代码也能判断回环为什么没触发
- 拒绝原因能快速区分是“没候选 / 分数不够 / 修正过大 / 冷却中”

### 8.6.8 单测与集成回归

目标：

- 保证后续调参时不把轻量回环逻辑悄悄改坏

实现任务：

1. 单测：
   - 候选检测
   - 接受门限
   - 冷却逻辑
2. 集成测试：
   - 合成一条回访轨迹
   - 验证回环命中后漂移变小
   - 验证坏候选不会被接受

建议文件：

- 新增 `test_loop_candidate_detector.cpp`
- 新增 `test_loop_correction_gate.cpp`
- 新增 `test_mapping_loop_closure.cpp`

完成标准：

- 至少覆盖“命中接受 / 命中拒绝 / 连续失败降级”三种场景

## 推荐实现顺序

建议按下面顺序做：

1. `8.6.1` 建图状态缓存
2. `8.6.2` 回访区域检测
3. `8.6.3` 轻量局部匹配
4. `8.6.4` 小修正接受器
5. `8.6.5` 接入 mapping pose
6. `8.6.6` 失败保护
7. `8.6.7` 调试可视化
8. `8.6.8` 回归测试

## 这一阶段的验收标准

通过标准建议定成：

1. 固定热身路线走一圈后，终点漂移明显小于当前版本
2. 轻量回环不会引入明显大跳变
3. 候选、匹配、接受、拒绝原因可被日志观察
4. 回环模块失败时，建图主流程仍可继续
5. 四线程机器上运行频率不被明显拖垮
