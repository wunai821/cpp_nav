#pragma once

namespace rm_nav::data {

struct PointXYZI {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  float relative_time_s{0.0F};
};

}  // namespace rm_nav::data
