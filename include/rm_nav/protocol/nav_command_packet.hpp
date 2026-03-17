#pragma once

#include "rm_nav/data/chassis_cmd.hpp"

namespace rm_nav::protocol {

struct NavCommandPacket {
  data::ChassisCmd cmd{};
};

}  // namespace rm_nav::protocol
