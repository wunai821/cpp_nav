## Phase 0 Thread Boundaries

Thread ownership is fixed before algorithm work starts.

### Driver Thread

Responsibilities:

- L1 lidar ingest
- IMU ingest
- STM32 transport

Properties:

- owns hardware-facing IO
- publishes packets or frames into lock-safe queues

### Sync Thread

Responsibilities:

- timestamp alignment
- `SyncedFrame` assembly

Properties:

- consumes driver output only
- does not run localization or perception logic

### Pose Core Thread

Responsibilities:

- localization
- mapping
- pose graph facing outputs in later phases

Properties:

- consumes `SyncedFrame`
- produces `Pose3f` and map-facing updates

### Perception Thread

Responsibilities:

- preprocess
- costmap projection
- MOT

Properties:

- consumes synced sensor data and pose snapshots
- owns obstacle and occupancy outputs

### Planner Thread

Responsibilities:

- path generation
- trajectory selection
- chassis command proposal

### Safety/FSM Thread

Responsibilities:

- command gating
- safety event aggregation
- runtime mode ownership

### Debug Thread

Responsibilities:

- mirror taps
- debug sample packaging
- WebSocket publishing

Rules:

- Debug thread is isolated from control timing.
- Cross-thread communication must happen through explicit data contracts.
- No module may silently spawn extra worker threads in Phase 0.
