#include "rm_nav/perception/obstacle_layer.hpp"

#include <algorithm>
#include <cmath>

#include "rm_nav/perception/local_costmap_builder.hpp"

namespace rm_nav::perception {
namespace {

constexpr std::uint8_t kOccupiedCost = 100U;

int CenterX(const data::GridMap2D& costmap) {
  return static_cast<int>(costmap.width / 2U);
}

int CenterY(const data::GridMap2D& costmap) {
  return static_cast<int>(costmap.height / 2U);
}

bool ToGrid(const data::GridMap2D& costmap, float local_x, float local_y, int* gx, int* gy) {
  if (gx == nullptr || gy == nullptr || costmap.resolution_m <= 0.0F) {
    return false;
  }
  *gx = CenterX(costmap) + static_cast<int>(std::round(local_x / costmap.resolution_m));
  *gy = CenterY(costmap) + static_cast<int>(std::round(local_y / costmap.resolution_m));
  return *gx >= 0 && *gy >= 0 && *gx < static_cast<int>(costmap.width) &&
         *gy < static_cast<int>(costmap.height);
}

}  // namespace

common::Status ObstacleLayer::Configure(const LocalCostmapConfig& config) {
  obstacle_layer_height_m_ = config.obstacle_layer_height_m;
  configured_ = true;
  return common::Status::Ok();
}

common::Status ObstacleLayer::Apply(const data::LidarFrame& filtered_frame,
                                    data::GridMap2D* costmap) const {
  if (costmap == nullptr) {
    return common::Status::InvalidArgument("obstacle layer costmap is null");
  }
  if (!configured_) {
    return common::Status::NotReady("obstacle layer is not configured");
  }

  for (const auto& point : filtered_frame.points) {
    if (point.z < obstacle_layer_height_m_) {
      continue;
    }
    int gx = 0;
    int gy = 0;
    if (!ToGrid(*costmap, point.x, point.y, &gx, &gy)) {
      continue;
    }
    auto& cell =
        costmap->occupancy[static_cast<std::size_t>(gy) * costmap->width + static_cast<std::size_t>(gx)];
    cell = std::max(cell, kOccupiedCost);
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
