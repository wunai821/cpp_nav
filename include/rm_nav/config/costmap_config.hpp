#pragma once

#include <cstdint>

namespace rm_nav::config {

struct CostmapConfig {
  std::uint32_t width{80};
  std::uint32_t height{80};
  double resolution_m{0.10};
  double obstacle_layer_height_m{0.05};
  double inflation_radius_m{0.30};
  double dynamic_obstacle_inflation_m{0.45};
};

}  // namespace rm_nav::config
