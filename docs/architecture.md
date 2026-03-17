## Phase 0 Goal

This phase only freezes boundaries, names, and interfaces so later stages do not
need structural rewrites. No complex localization, mapping, planning, or debug
transport logic should land here.

## Coordinate Frames

The frame IDs are fixed for the whole project:

- `map`
- `odom`
- `base_link`
- `laser_link`
- `imu_link`

Semantics:

- `map`: globally consistent frame produced by localization or mapping.
- `odom`: locally smooth frame for short-horizon control.
- `base_link`: robot body center used by planning and safety.
- `laser_link`: L1 lidar mounting frame.
- `imu_link`: IMU mounting frame.

## Main Runtime Chain

The primary chain is fixed as:

`L1/IMU -> Sync -> Localization/Mapping -> Preprocess -> Costmap/MOT -> Planner -> Safety -> STM32`

Module ownership in Phase 0:

- `L1/IMU`: raw sensor drivers and packet/frame boundaries.
- `Sync`: timestamp alignment and fused sensor bundle output.
- `Localization/Mapping`: pose estimation and map updates.
- `Preprocess`: lidar filtering and normalization before perception.
- `Costmap/MOT`: static occupancy projection and dynamic obstacle tracking.
- `Planner`: path generation and chassis command production.
- `Safety`: command gating and safety event generation.
- `STM32`: final command delivery to the lower controller.

## Debug Side Chain

Foxglove stays on a side channel and must not sit on the control critical path.

`Any main-chain node -> MirrorTap -> DebugPublisher -> WebSocket`

Rules:

- Mirror traffic is best-effort only.
- Debug backpressure must not block the main chain.
- Debug serialization stays outside planner and safety hot paths.

## Core Data Contracts

Phase 0 freezes the following data structures by name:

- `ImuPacket`
- `LidarFrame`
- `SyncedFrame`
- `Pose3f`
- `GridMap2D`
- `DynamicObstacle`
- `Path2D`
- `ChassisCmd`
- `SafetyEvent`

These names are intentionally small and stable so later algorithm modules can be
reworked without renaming the project-wide contracts.

## Explicit Non-Goals

Phase 0 does not include:

- NDT, ICP, or other heavy localization implementation details
- Foxglove runtime integration
- a large all-mode FSM
- costmap tuning or layered obstacle semantics
