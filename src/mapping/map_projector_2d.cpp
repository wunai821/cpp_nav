#include "rm_nav/mapping/map_projector_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "rm_nav/common/time.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::mapping {

common::Status MapProjector2D::Configure(const config::MappingConfig& config) {
  if (config.occupancy_resolution_m <= 0.0 || config.z_max_m <= config.z_min_m) {
    return common::Status::InvalidArgument("invalid mapping projector config");
  }
  z_min_m_ = static_cast<float>(config.z_min_m);
  z_max_m_ = static_cast<float>(config.z_max_m);
  resolution_m_ = static_cast<float>(config.occupancy_resolution_m);
  padding_m_ = static_cast<float>(std::max(0.0, config.occupancy_padding_m));
  return common::Status::Ok();
}

common::Status MapProjector2D::Project(const std::vector<data::PointXYZI>& global_points,
                                       data::GridMap2D* occupancy) const {
  if (occupancy == nullptr) {
    return common::Status::InvalidArgument("occupancy output is null");
  }

  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();
  std::vector<const data::PointXYZI*> projected_points;
  projected_points.reserve(global_points.size());
  for (const auto& point : global_points) {
    if (point.z < z_min_m_ || point.z > z_max_m_) {
      continue;
    }
    projected_points.push_back(&point);
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }
  if (projected_points.empty()) {
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
  occupancy->occupancy.assign(
      static_cast<std::size_t>(occupancy->width) * occupancy->height, 0U);

  for (const auto* point : projected_points) {
    const int gx = static_cast<int>(std::floor((point->x - min_x) / resolution_m_));
    const int gy = static_cast<int>(std::floor((point->y - min_y) / resolution_m_));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(occupancy->width) ||
        gy >= static_cast<int>(occupancy->height)) {
      continue;
    }
    occupancy->occupancy[static_cast<std::size_t>(gy) * occupancy->width +
                         static_cast<std::size_t>(gx)] = 100U;
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
