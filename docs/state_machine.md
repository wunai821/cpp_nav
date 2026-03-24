# 状态机说明

本文档描述主程序当前使用的业务型 FSM。实现位置在 [nav_fsm.cpp](../src/fsm/nav_fsm.cpp)。

## 状态枚举

当前固定状态：

- `BOOT`
- `SELF_CHECK`
- `IDLE`
- `MODE_WARMUP`
- `MODE_SAVE`
- `MODE_COMBAT`
- `GOTO_CENTER`
- `HOLD_CENTER`
- `RECENTER`
- `RECOVERY`
- `FAILSAFE`

## 每个状态负责什么

### `BOOT`

- 进程启动后的初始态
- 只做最小启动过渡

### `SELF_CHECK`

- 基础自检阶段
- 当前实现主要用于把线程和链路拉起来

### `IDLE`

- 等待进入热身建图或正赛模式
- 自动或手动模式切换都从这里分流

### `MODE_WARMUP`

- 打开建图链
- 打开 waypoint 巡航
- 使用 `MappingEngine` 累积点云和 2D 占据图

### `MODE_SAVE`

- 冻结热身建图
- 保存地图产物
- 尝试把导出的地图接到 combat pipeline

### `MODE_COMBAT`

- 地图已可用
- combat pipeline 已初始化
- 准备进入中心区任务

### `GOTO_CENTER`

- 启动全局 A* 到中心目标
- 启动局部 DWA

### `HOLD_CENTER`

- 已进入中心区
- 启动 Center Hold 行为
- 缩小速度并持续回正

### `RECENTER`

- 发生中心区偏移
- 重新靠近中心点

### `RECOVERY`

- 定位质量差或规划失败时进入
- 输出更保守的控制
- 依赖定位恢复和轻量 relocalization 重新回到 `GOTO_CENTER`

### `FAILSAFE`

- 安全刹车
- command gate 强制覆盖输出
- 等待心跳恢复或安全条件解除

## 事件枚举

当前实现的状态事件：

- `NONE`
- `SELF_CHECK_PASSED`
- `ENTER_WARMUP_MODE`
- `ENTER_COMBAT_MODE`
- `MAP_LOAD_SUCCESS`
- `MAP_SAVE_SUCCESS`
- `LOCALIZATION_DEGRADED`
- `PLANNER_FAILED`
- `CENTER_DRIFTED`
- `SAFETY_TRIGGERED`
- `HEARTBEAT_ANOMALY`
- `REFEREE_STATE_CHANGED`

## 主要转移关系

### 启动链

- `BOOT -> SELF_CHECK`
- `SELF_CHECK -> IDLE`

### 热身链

- `IDLE -> MODE_WARMUP`
  条件：`mapping_enabled = true`
- `MODE_WARMUP -> MODE_SAVE`
  条件：`save_requested = true`
- `MODE_SAVE -> MODE_COMBAT`
  条件：地图保存成功且 combat pipeline 已准备好
- `MODE_SAVE -> IDLE`
  条件：只保存，不切战斗链

### 正赛链

- `IDLE -> MODE_COMBAT`
  条件：地图已加载且 combat pipeline ready
- `MODE_COMBAT -> GOTO_CENTER`
  条件：地图可用
- `GOTO_CENTER -> HOLD_CENTER`
  条件：中心区到达
- `HOLD_CENTER -> RECENTER`
  条件：中心区漂移
- `RECENTER -> HOLD_CENTER`
  条件：重新回到中心区

### 恢复链

- `MODE_COMBAT / GOTO_CENTER / HOLD_CENTER / RECENTER -> RECOVERY`
  条件：
  - 定位退化
  - 规划失败
- `RECOVERY -> GOTO_CENTER`
  条件：定位和规划恢复，地图仍可用

### 失效保护链

- 任意状态 -> `FAILSAFE`
  条件：
  - 心跳异常
  - safety trigger
- `FAILSAFE -> MODE_WARMUP / MODE_COMBAT / IDLE`
  条件：心跳恢复且安全条件解除

## 手动模式覆盖

配置入口在 [system.yaml](../config/system.yaml)：

- `manual_mode_selector: -1`
  使用自动 FSM；若 active 地图存在则优先进入 combat，否则优先进入 warmup
- `manual_mode_selector: 0`
  只允许 `MODE_WARMUP + MODE_SAVE`
- `manual_mode_selector: 1`
  只允许 combat 主链
- `match_mode_enabled: true`
  比赛模式总开关；打开后建图链和 combat 主链都会统一等待裁判/下位机开始信号，同时启用 `HP/ammo` 触发的补给/回血流程

`mapping.enabled` 和 `localization.enabled` 现在只保留为兼容字段，不再作为 runtime 主链切模式入口。

## 运行时可视化

runtime 会发布：

- `/rm_nav/runtime/fsm_status`
- `/rm_nav/runtime/fsm_event`

对应调试快照会落在：

- `logs/debug/foxglove/fsm_status.json`
- `logs/debug/foxglove/fsm_event.json`
