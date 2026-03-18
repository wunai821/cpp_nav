#include "rm_nav/mapping/map_projector_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::mapping {
namespace {

data::PointXYZI TransformPoint(const data::PointXYZI& point, const data::Pose3f& pose) {
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  data::PointXYZI transformed = point;
  transformed.x = pose.position.x + cos_yaw * point.x - sin_yaw * point.y;
  transformed.y = pose.position.y + sin_yaw * point.x + cos_yaw * point.y;
  transformed.z = pose.position.z + point.z;
  return transformed;
}

std::size_t GridIndex(std::uint32_t width, int gx, int gy) {
  return static_cast<std::size_t>(gy) * width + static_cast<std::size_t>(gx);
}

void UpdateLogOdds(std::vector<float>* log_odds, std::vector<std::uint16_t>* observations,
                   std::uint32_t width, int gx, int gy, float delta,
                   float min_log_odds, float max_log_odds) {
  if (log_odds == nullptr || observations == nullptr || gx < 0 || gy < 0) {
    return;
  }
  const std::size_t index = GridIndex(width, gx, gy);
  (*log_odds)[index] = std::clamp((*log_odds)[index] + delta, min_log_odds, max_log_odds);
  ++(*observations)[index];
}

template <typename UpdateFreeFn>
void RayCastFreeUpdate(int x0, int y0, int x1, int y1, const UpdateFreeFn& update_free) {
  int x = x0;
  int y = y0;
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  while (x != x1 || y != y1) {
    if (!(x == x0 && y == y0)) {
      update_free(x, y);
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
}

float ProbabilityFromLogOdds(float log_odds) {
  const float odds = std::exp(log_odds);
  return odds / (1.0F + odds);
}

void InflateOccupiedCells(data::GridMap2D* occupancy, float inflation_radius_m) {
  if (occupancy == nullptr || inflation_radius_m <= 0.0F || occupancy->occupancy.empty()) {
    return;
  }

  auto inflated = occupancy->occupancy;
  const int inflation_cells = std::max(
      0, static_cast<int>(std::ceil(inflation_radius_m / occupancy->resolution_m)));
  for (std::uint32_t y = 0; y < occupancy->height; ++y) {
    for (std::uint32_t x = 0; x < occupancy->width; ++x) {
      if (occupancy->occupancy[GridIndex(occupancy->width, static_cast<int>(x),
                                         static_cast<int>(y))] != 100U) {
        continue;
      }
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          const int nx = static_cast<int>(x) + dx;
          const int ny = static_cast<int>(y) + dy;
          if (nx < 0 || ny < 0 || nx >= static_cast<int>(occupancy->width) ||
              ny >= static_cast<int>(occupancy->height)) {
            continue;
          }
          const float distance =
              std::sqrt(static_cast<float>(dx * dx + dy * dy)) * occupancy->resolution_m;
          if (distance > inflation_radius_m) {
            continue;
          }
          auto& cell = inflated[GridIndex(occupancy->width, nx, ny)];
          if (cell == 100U) {
            continue;
          }
          if (cell == 255U) {
            cell = 80U;
          } else {
            cell = std::max<std::uint8_t>(cell, 80U);
          }
        }
      }
    }
  }
  occupancy->occupancy = std::move(inflated);
}

}  // namespace

common::Status MapProjector2D::Configure(const config::MappingConfig& config) {
  if (config.occupancy_resolution_m <= 0.0 || config.z_max_m <= config.z_min_m) {
    return common::Status::InvalidArgument("invalid mapping projector config");
  }
  z_min_m_ = static_cast<float>(config.z_min_m);
  z_max_m_ = static_cast<float>(config.z_max_m);
  resolution_m_ = static_cast<float>(config.occupancy_resolution_m);
  padding_m_ = static_cast<float>(std::max(0.0, config.occupancy_padding_m));
  hit_log_odds_ = static_cast<float>(config.occupancy_hit_log_odds);
  miss_log_odds_ = static_cast<float>(config.occupancy_miss_log_odds);
  min_log_odds_ = static_cast<float>(config.occupancy_min_log_odds);
  max_log_odds_ = static_cast<float>(config.occupancy_max_log_odds);
  free_threshold_log_odds_ =
      static_cast<float>(config.occupancy_free_threshold_log_odds);
  occupied_threshold_log_odds_ =
      static_cast<float>(config.occupancy_occupied_threshold_log_odds);
  inflation_radius_m_ = static_cast<float>(std::max(0.0, config.occupancy_inflation_radius_m));
  return common::Status::Ok();
}

common::Status MapProjector2D::Project(const std::vector<data::PointXYZI>& global_points,
                                       const std::vector<MappingKeyframe>& keyframes,
                                       data::GridMap2D* occupancy) const {
  if (occupancy == nullptr) {
    return common::Status::InvalidArgument("occupancy output is null");
  }

  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();
  bool have_bounds = false;
  for (const auto& keyframe : keyframes) {
    for (const auto& local_point : keyframe.local_points) {
      auto point = TransformPoint(local_point, keyframe.map_to_base);
      if (point.z < z_min_m_ || point.z > z_max_m_) {
        continue;
      }
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
      have_bounds = true;
    }
    if (keyframe.map_to_base.is_valid) {
      min_x = std::min(min_x, keyframe.map_to_base.position.x);
      min_y = std::min(min_y, keyframe.map_to_base.position.y);
      max_x = std::max(max_x, keyframe.map_to_base.position.x);
      max_y = std::max(max_y, keyframe.map_to_base.position.y);
      have_bounds = true;
    }
  }
  if (!have_bounds) {
    for (const auto& point : global_points) {
      if (point.z < z_min_m_ || point.z > z_max_m_) {
        continue;
      }
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
      have_bounds = true;
    }
  }
  if (!have_bounds) {
    return common::Status::NotReady("no points available for 2d projection");
  }

  min_x -= padding_m_;
  min_y -= padding_m_;
  max_x += padding_m_;
  max_y += padding_m_;

  occupancy->stamp = common::Now();
  occupancy->resolution_m = resolution_m_;
  occupancy->width = static_cast<std::uint32_t>(
      std::max(1.0F, std::ceil((max_x - min_x) / resolution_m_) + 1.0F));
  occupancy->height = static_cast<std::uint32_t>(
      std::max(1.0F, std::ceil((max_y - min_y) / resolution_m_) + 1.0F));
  occupancy->origin.reference_frame = tf::kMapFrame;
  occupancy->origin.child_frame = tf::kMapFrame;
  occupancy->origin.position.x = min_x;
  occupancy->origin.position.y = min_y;
  occupancy->origin.is_valid = true;
  const std::size_t cell_count =
      static_cast<std::size_t>(occupancy->width) * occupancy->height;
  occupancy->occupancy.assign(cell_count, 255U);
  occupancy->occupancy_log_odds.assign(cell_count, 0.0F);
  std::vector<std::uint16_t> observations(cell_count, 0U);

  if (!keyframes.empty()) {
    for (const auto& keyframe : keyframes) {
      if (!keyframe.map_to_base.is_valid) {
        continue;
      }
      const int sensor_gx = static_cast<int>(
          std::floor((keyframe.map_to_base.position.x - min_x) / resolution_m_));
      const int sensor_gy = static_cast<int>(
          std::floor((keyframe.map_to_base.position.y - min_y) / resolution_m_));
      if (sensor_gx < 0 || sensor_gy < 0 || sensor_gx >= static_cast<int>(occupancy->width) ||
          sensor_gy >= static_cast<int>(occupancy->height)) {
        continue;
      }

      for (const auto& local_point : keyframe.local_points) {
        auto point = TransformPoint(local_point, keyframe.map_to_base);
        if (point.z < z_min_m_ || point.z > z_max_m_) {
          continue;
        }
        const int gx = static_cast<int>(std::floor((point.x - min_x) / resolution_m_));
        const int gy = static_cast<int>(std::floor((point.y - min_y) / resolution_m_));
        if (gx < 0 || gy < 0 || gx >= static_cast<int>(occupancy->width) ||
            gy >= static_cast<int>(occupancy->height)) {
          continue;
        }

        RayCastFreeUpdate(sensor_gx, sensor_gy, gx, gy, [&](int free_x, int free_y) {
          if (free_x < 0 || free_y < 0 || free_x >= static_cast<int>(occupancy->width) ||
              free_y >= static_cast<int>(occupancy->height)) {
            return;
          }
          UpdateLogOdds(&occupancy->occupancy_log_odds, &observations, occupancy->width, free_x,
                        free_y, -miss_log_odds_, min_log_odds_, max_log_odds_);
        });
        UpdateLogOdds(&occupancy->occupancy_log_odds, &observations, occupancy->width, gx, gy,
                      hit_log_odds_, min_log_odds_, max_log_odds_);
      }
    }
  } else {
    for (const auto& point : global_points) {
      if (point.z < z_min_m_ || point.z > z_max_m_) {
        continue;
      }
      const int gx = static_cast<int>(std::floor((point.x - min_x) / resolution_m_));
      const int gy = static_cast<int>(std::floor((point.y - min_y) / resolution_m_));
      if (gx < 0 || gy < 0 || gx >= static_cast<int>(occupancy->width) ||
          gy >= static_cast<int>(occupancy->height)) {
        continue;
      }
      UpdateLogOdds(&occupancy->occupancy_log_odds, &observations, occupancy->width, gx, gy,
                    hit_log_odds_, min_log_odds_, max_log_odds_);
    }
  }

  for (std::size_t index = 0; index < cell_count; ++index) {
    if (observations[index] == 0U) {
      occupancy->occupancy[index] = 255U;
      continue;
    }
    const float log_odds = occupancy->occupancy_log_odds[index];
    if (log_odds >= occupied_threshold_log_odds_) {
      occupancy->occupancy[index] = 100U;
    } else if (log_odds <= free_threshold_log_odds_) {
      occupancy->occupancy[index] = 0U;
    } else {
      occupancy->occupancy[index] = static_cast<std::uint8_t>(
          std::clamp(std::lround(ProbabilityFromLogOdds(log_odds) * 100.0F), 1L, 99L));
    }
  }
  InflateOccupiedCells(occupancy, inflation_radius_m_);
  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
