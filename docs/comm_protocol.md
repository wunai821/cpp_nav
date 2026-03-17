## Stage 2 Communication Contract

This project now has a minimal transport contract for lower-controller IO and
offline reproduction.

### STM32 Framing

Packet layout:

- `SOF0 = 0xA5`
- `SOF1 = 0x5A`
- `version = 1`
- `type = uint8`
- `payload_size = uint16`
- `sequence = uint16`
- `header_crc8 = uint8`
- `payload`
- `frame_crc16 = uint16`

Message types:

- `1`: heartbeat
- `2`: chassis command
- `3`: odom feedback
- `4`: referee state

Current implementation is intentionally small and aimed at bring-up:

- serial only, no CAN path yet
- command path supports heartbeat and velocity command output
- receive path supports odom and referee state
- `stm32_bridge_test` provides a PTY-backed mock loop for offline validation

### Recorder Format

Recorder file header:

- magic: `RMREC01\n`
- version: `uint32 = 1`

Each record stores:

- `channel = uint8`
- `stamp_ns = int64`
- `payload_size = uint32`
- `payload bytes`

Supported channels:

- `1`: lidar frame
- `2`: imu packet
- `3`: odom state
- `4`: referee state
- `5`: chassis command

### Bring-Up Tools

Available binaries:

- `./stm32_bridge_test`
- `./l1_driver_test`
- `./l1_record_tool`
- `./imu_driver_test`
- `./imu_record_tool`
- `./replay_player`

Current limitation:

- L1 and IMU hardware parsers are still placeholders; Stage 2 code provides
  synthetic drivers plus a stable recorder/replayer so the data path can be
  verified before real packet formats are wired in.
