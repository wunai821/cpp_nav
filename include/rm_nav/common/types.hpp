#pragma once

#include <cstdint>

namespace rm_nav::common {

struct Vec2f {
  float x{0.0F};
  float y{0.0F};
};

struct Vec3f {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

using ObjectId = std::uint32_t;

}  // namespace rm_nav::common
