#include "rm_nav/perception/local_costmap_builder.hpp"

#include <algorithm>
#include <cmath>

namespace rm_nav::perception {
namespace {

constexpr std::uint8_t kOccupiedCost = 100U;
constexpr std::uint8_t kInflatedCost = 80U;
constexpr std::uint8_t kDynamicCost = 100U;

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

void MarkCell(data::GridMap2D* costmap, int gx, int gy, std::uint8_t cost) {
  if (costmap == nullptr || gx < 0 || gy < 0 || gx >= static_cast<int>(costmap->width) ||
      gy >= static_cast<int>(costmap->height)) {
    return;
  }
  auto& cell =
      costmap->occupancy[static_cast<std::size_t>(gy) * costmap->width + static_cast<std::size_t>(gx)];
  cell = std::max(cell, cost);
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
      MarkCell(costmap, center_gx + dx, center_gy + dy, cost);
    }
  }
}

void InflateOccupiedCells(data::GridMap2D* costmap, float inflation_radius_m) {
  if (costmap == nullptr || inflation_radius_m <= 0.0F || costmap->occupancy.empty()) {
    return;
  }
  std::vector<std::uint8_t> inflated = costmap->occupancy;
  const int radius_cells =
      std::max(0, static_cast<int>(std::ceil(inflation_radius_m / costmap->resolution_m)));
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
          if (distance > inflation_radius_m) {
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

common::Status LocalCostmapBuilder::Configure(const LocalCostmapConfig& config) {
  if (config.width == 0U || config.height == 0U || config.resolution_m <= 0.0F ||
      config.inflation_radius_m < 0.0F || config.dynamic_obstacle_inflation_m < 0.0F) {
    return common::Status::InvalidArgument("invalid local costmap config");
  }
  config_ = config;
  return common::Status::Ok();
}

common::Status LocalCostmapBuilder::Build(const data::LidarFrame& filtered_frame,
                                          const data::Pose3f& pose,
                                          data::GridMap2D* costmap) {
  const std::vector<data::DynamicObstacle> no_obstacles;
  return Build(filtered_frame, pose, no_obstacles, costmap);
}

common::Status LocalCostmapBuilder::Build(const data::LidarFrame& filtered_frame,
                                          const data::Pose3f& pose,
                                          const std::vector<data::DynamicObstacle>& obstacles,
                                          data::GridMap2D* costmap) {
  if (costmap == nullptr) {
    return common::Status::InvalidArgument("costmap output is null");
  }

  costmap->stamp = filtered_frame.stamp;
  costmap->resolution_m = config_.resolution_m;
  costmap->width = config_.width;
  costmap->height = config_.height;
  costmap->origin = pose;
  costmap->occupancy.assign(static_cast<std::size_t>(config_.width) * config_.height, 0U);

  for (const auto& point : filtered_frame.points) {
    if (point.z < config_.obstacle_layer_height_m) {
      continue;
    }
    int gx = 0;
    int gy = 0;
    if (!ToGrid(*costmap, point.x, point.y, &gx, &gy)) {
      continue;
    }
    MarkCell(costmap, gx, gy, kOccupiedCost);
  }

  InflateOccupiedCells(costmap, config_.inflation_radius_m);

  for (const auto& obstacle : obstacles) {
    if (!obstacle.pose.is_valid || obstacle.confidence < 0.1F) {
      continue;
    }
    const auto local = WorldToLocal(pose, obstacle.pose.position.x, obstacle.pose.position.y);
    const float fused_radius =
        std::max(0.15F, obstacle.radius_m + config_.dynamic_obstacle_inflation_m);
    MarkDisk(costmap, local.x, local.y, fused_radius, kDynamicCost);
    const auto predicted = WorldToLocal(
        pose, obstacle.predicted_pose_05s.position.x, obstacle.predicted_pose_05s.position.y);
    MarkDisk(costmap, predicted.x, predicted.y, fused_radius * 0.85F, kDynamicCost);
  }
  return common::Status::Ok();
}

common::Status LocalCostmapBuilder::BuildAndPublish(const data::LidarFrame& filtered_frame,
                                                    const data::Pose3f& pose) {
  const std::vector<data::DynamicObstacle> no_obstacles;
  return BuildAndPublish(filtered_frame, pose, no_obstacles);
}

common::Status LocalCostmapBuilder::BuildAndPublish(
    const data::LidarFrame& filtered_frame, const data::Pose3f& pose,
    const std::vector<data::DynamicObstacle>& obstacles) {
  data::LocalCostmap costmap;
  const auto status = Build(filtered_frame, pose, obstacles, &costmap);
  if (!status.ok()) {
    return status;
  }
  latest_costmap_.Publish(std::move(costmap));
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
