#pragma once

#include "rm_nav/data/odom_state.hpp"

namespace rm_nav::protocol {

struct OdomFeedbackPacket {
  data::OdomState odom{};
};

}  // namespace rm_nav::protocol
