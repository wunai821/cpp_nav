#pragma once

#include <string>

namespace rm_nav::config {

struct CommConfig {
  bool stm32_enabled{true};
  std::string stm32_port{"/dev/ttyUSB0"};
  int stm32_baud_rate{115200};
};

}  // namespace rm_nav::config
