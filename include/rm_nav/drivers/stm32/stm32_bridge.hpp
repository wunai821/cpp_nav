#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/health_report.hpp"
#include "rm_nav/data/odom_state.hpp"
#include "rm_nav/data/referee_state.hpp"
#include "rm_nav/drivers/serial/serial_config.hpp"
#include "rm_nav/drivers/serial/serial_port.hpp"
#include "rm_nav/protocol/protocol_codec.hpp"

namespace rm_nav::drivers::stm32 {

struct Stm32BridgeConfig {
  serial::SerialConfig serial{};
  std::uint8_t heartbeat_state{1};
};

class Stm32Bridge {
 public:
  Stm32Bridge() = default;
  ~Stm32Bridge();

  common::Status Configure(const Stm32BridgeConfig& config);
  void Close();
  bool IsOpen() const;

  common::Status SendHeartbeat();
  common::Status SendChassisCmd(const data::ChassisCmd& cmd);
  bool WaitForRx(std::chrono::milliseconds timeout) const;
  common::Status SpinOnce();
  std::optional<data::OdomState> TakeOdomState();
  std::optional<data::RefereeState> TakeRefereeState();
  data::HealthReport health_report() const { return health_report_; }

 private:
  Stm32BridgeConfig config_{};
  data::HealthReport health_report_{};
  std::optional<data::OdomState> latest_odom_{};
  std::optional<data::RefereeState> latest_referee_{};
  serial::SerialPort serial_port_{};
  protocol::ProtocolCodec codec_{};
};

}  // namespace rm_nav::drivers::stm32
