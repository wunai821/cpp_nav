#include "rm_nav/perception/inflation_layer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "rm_nav/perception/local_costmap_builder.hpp"

namespace rm_nav::perception {
namespace {

constexpr std::uint8_t kOccupiedCost = 100U;
constexpr std::uint8_t kInflatedCost = 80U;

}  // namespace

common::Status InflationLayer::Configure(const LocalCostmapConfig& config) {
  inflation_radius_m_ = std::max(0.0F, config.inflation_radius_m);
  configured_ = true;
  return common::Status::Ok();
}

common::Status InflationLayer::Apply(data::GridMap2D* costmap) const {
  if (costmap == nullptr) {
    return common::Status::InvalidArgument("inflation layer costmap is null");
  }
  if (!configured_) {
    return common::Status::NotReady("inflation layer is not configured");
  }
  if (inflation_radius_m_ <= 0.0F || costmap->occupancy.empty()) {
    return common::Status::Ok();
  }

  std::vector<std::uint8_t> inflated = costmap->occupancy;
  const int radius_cells =
      std::max(0, static_cast<int>(std::ceil(inflation_radius_m_ / costmap->resolution_m)));
  for (std::uint32_t gy = 0; gy < costmap->height; ++gy) {
    for (std::uint32_t gx = 0; gx < costmap->width; ++gx) {
      const auto index = static_cast<std::size_t>(gy) * costmap->width + gx;
      if (costmap->occupancy[index] < kOccupiedCost) {
        continue;
      }
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const int nx = static_cast<int>(gx) + dx;
          const int ny = static_cast<int>(gy) + dy;
          if (nx < 0 || ny < 0 || nx >= static_cast<int>(costmap->width) ||
              ny >= static_cast<int>(costmap->height)) {
            continue;
          }
          const float distance = std::sqrt(static_cast<float>(dx * dx + dy * dy)) *
                                 costmap->resolution_m;
          if (distance > inflation_radius_m_) {
            continue;
          }
          auto& cell = inflated[static_cast<std::size_t>(ny) * costmap->width +
                                static_cast<std::size_t>(nx)];
          cell = std::max(cell, kInflatedCost);
        }
      }
    }
  }
  costmap->occupancy = std::move(inflated);
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
