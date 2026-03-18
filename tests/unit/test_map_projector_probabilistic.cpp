#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/mapping/map_projector_2d.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

rm_nav::mapping::MappingKeyframe MakeKeyframe() {
  rm_nav::mapping::MappingKeyframe keyframe;
  keyframe.stamp = rm_nav::common::Now();
  keyframe.frame_index = 1U;
  keyframe.map_to_base.stamp = keyframe.stamp;
  keyframe.map_to_base.reference_frame = rm_nav::tf::kMapFrame;
  keyframe.map_to_base.child_frame = rm_nav::tf::kBaseLinkFrame;
  keyframe.map_to_base.is_valid = true;

  rm_nav::data::PointXYZI hit_point;
  hit_point.x = 1.0F;
  hit_point.y = 0.0F;
  hit_point.z = 0.6F;
  hit_point.intensity = 1.0F;
  keyframe.local_points.push_back(hit_point);
  return keyframe;
}

std::uint8_t CellAt(const rm_nav::data::GridMap2D& grid, float x, float y) {
  const int gx = static_cast<int>(std::floor((x - grid.origin.position.x) / grid.resolution_m));
  const int gy = static_cast<int>(std::floor((y - grid.origin.position.y) / grid.resolution_m));
  assert(gx >= 0 && gy >= 0);
  assert(gx < static_cast<int>(grid.width) && gy < static_cast<int>(grid.height));
  return grid.occupancy[static_cast<std::size_t>(gy) * grid.width + static_cast<std::size_t>(gx)];
}

}  // namespace

int main() {
  rm_nav::config::MappingConfig config;
  config.z_min_m = 0.2;
  config.z_max_m = 1.2;
  config.occupancy_resolution_m = 0.1;
  config.occupancy_padding_m = 0.5;
  config.occupancy_hit_log_odds = 1.0;
  config.occupancy_miss_log_odds = 0.6;
  config.occupancy_min_log_odds = -2.0;
  config.occupancy_max_log_odds = 3.5;
  config.occupancy_free_threshold_log_odds = -0.2;
  config.occupancy_occupied_threshold_log_odds = 0.6;
  config.occupancy_inflation_radius_m = 0.15;

  rm_nav::mapping::MapProjector2D projector;
  assert(projector.Configure(config).ok());

  rm_nav::data::GridMap2D grid;
  const std::vector<rm_nav::data::PointXYZI> global_points;
  const std::vector<rm_nav::mapping::MappingKeyframe> keyframes = {MakeKeyframe()};
  assert(projector.Project(global_points, keyframes, &grid).ok());

  const auto hit = CellAt(grid, 1.0F, 0.0F);
  const auto inflated = CellAt(grid, 1.0F, 0.1F);
  const auto unknown = CellAt(grid, -0.4F, 0.4F);
  bool saw_free = false;
  for (float x = 0.1F; x <= 0.8F; x += 0.1F) {
    if (CellAt(grid, x, 0.0F) == 0U) {
      saw_free = true;
      break;
    }
  }

  assert(hit == 100U);
  assert(saw_free);
  assert(inflated >= 80U);
  assert(unknown == 255U);
  assert(!grid.occupancy_log_odds.empty());

  std::cout << "test_map_projector_probabilistic passed\n";
  return 0;
}
