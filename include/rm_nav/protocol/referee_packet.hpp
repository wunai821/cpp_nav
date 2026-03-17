#pragma once

#include "rm_nav/data/referee_state.hpp"

namespace rm_nav::protocol {

struct RefereePacket {
  data::RefereeState state{};
};

}  // namespace rm_nav::protocol
