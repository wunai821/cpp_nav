#include "rm_nav/perception/dynamic_layer.hpp"

#include <algorithm>
#include <cmath>

#include "rm_nav/perception/local_costmap_builder.hpp"

namespace rm_nav::perception {
namespace {

constexpr std::uint8_t kDynamicCost = 100U;
constexpr std::uint8_t kDynamicPredictedCost = 90U;

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

void MarkDisk(data::GridMap2D* costmap, float local_x, float local_y, float radius_m,
              std::uint8_t cost) {
  if (costmap == nullptr || radius_m < 0.0F) {
    return;
  }
  int center_gx = 0;
  int center_gy = 0;
  if (!ToGrid(*costmap, local_x, local_y, &center_gx, &center_gy)) {
    return;
  }
  const int radius_cells =
      std::max(0, static_cast<int>(std::ceil(radius_m / costmap->resolution_m)));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const float distance = std::sqrt(static_cast<float>(dx * dx + dy * dy)) *
                             costmap->resolution_m;
      if (distance > radius_m) {
        continue;
      }
      const int gx = center_gx + dx;
      const int gy = center_gy + dy;
      if (gx < 0 || gy < 0 || gx >= static_cast<int>(costmap->width) ||
          gy >= static_cast<int>(costmap->height)) {
        continue;
      }
      auto& cell =
          costmap->occupancy[static_cast<std::size_t>(gy) * costmap->width + static_cast<std::size_t>(gx)];
      cell = std::max(cell, cost);
    }
  }
}

common::Vec2f WorldToLocal(const data::Pose3f& pose, float world_x, float world_y) {
  const float dx = world_x - pose.position.x;
  const float dy = world_y - pose.position.y;
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  common::Vec2f local;
  local.x = cos_yaw * dx + sin_yaw * dy;
  local.y = -sin_yaw * dx + cos_yaw * dy;
  return local;
}

}  // namespace

common::Status DynamicLayer::Configure(const LocalCostmapConfig& config) {
  dynamic_obstacle_inflation_m_ = std::max(0.0F, config.dynamic_obstacle_inflation_m);
  configured_ = true;
  return common::Status::Ok();
}

common::Status DynamicLayer::Apply(const data::Pose3f& pose,
                                   const std::vector<data::DynamicObstacle>& obstacles,
                                   const data::GridMap2D& reference_grid,
                                   data::GridMap2D* dynamic_layer) const {
  if (dynamic_layer == nullptr) {
    return common::Status::InvalidArgument("dynamic layer output is null");
  }
  if (!configured_) {
    return common::Status::NotReady("dynamic layer is not configured");
  }

  *dynamic_layer = reference_grid;
  dynamic_layer->occupancy.assign(reference_grid.occupancy.size(), 0U);
  for (const auto& obstacle : obstacles) {
    if (!obstacle.pose.is_valid || obstacle.confidence < 0.1F) {
      continue;
    }
    const auto local = WorldToLocal(pose, obstacle.pose.position.x, obstacle.pose.position.y);
    const float current_radius =
        std::max(0.15F, obstacle.radius_m + dynamic_obstacle_inflation_m_);
    const float predicted_radius =
        std::max(current_radius,
                 std::max(obstacle.radius_m, obstacle.predicted_radius_m) +
                     dynamic_obstacle_inflation_m_);
    MarkDisk(dynamic_layer, local.x, local.y, current_radius, kDynamicCost);
    if (obstacle.predicted_pose_05s.is_valid) {
      const auto predicted = WorldToLocal(
          pose, obstacle.predicted_pose_05s.position.x, obstacle.predicted_pose_05s.position.y);
      MarkDisk(dynamic_layer, predicted.x, predicted.y, predicted_radius,
               kDynamicPredictedCost);
    }
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
