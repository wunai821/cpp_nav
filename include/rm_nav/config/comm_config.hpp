#pragma once

#include <string>

namespace rm_nav::config {

struct CommConfig {
  bool stm32_enabled{true};
  std::string stm32_port{"/dev/ttyUSB0"};
  int stm32_baud_rate{115200};
  std::string stm32_command_frame{"base_link"};
  std::string stm32_feedback_yaw_mode{"base_link"};
  bool stm32_test_cmd_enabled{false};
  double stm32_test_cmd_vx_mps{1.0};
  double stm32_test_cmd_vy_mps{1.0};
  double stm32_test_cmd_wz_radps{0.0};
  bool stm32_test_cmd_brake{false};
};

}  // namespace rm_nav::config
