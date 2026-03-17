rm_sentinel_nav

Phase 0 currently freezes the project boundaries and interfaces:

- Coordinate frames: `map`, `odom`, `base_link`, `laser_link`, `imu_link`
- Main chain: `L1/IMU -> Sync -> Localization/Mapping -> Preprocess -> Costmap/MOT -> Planner -> Safety -> STM32`
- Debug side chain: `Any main-chain node -> MirrorTap -> DebugPublisher -> WebSocket`
- Thread domains: driver, sync, pose core, perception, planner, safety/FSM, debug

See [docs/architecture.md](/home/naiwu/rm_sentinel_nav/docs/architecture.md) and [docs/thread_model.md](/home/naiwu/rm_sentinel_nav/docs/thread_model.md).
# rm_sentinel_nav
