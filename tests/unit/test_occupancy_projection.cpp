#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/mapping/map_projector_2d.hpp"
#include "rm_nav/mapping/map_serializer.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

rm_nav::mapping::MappingKeyframe MakeKeyframe(std::uint32_t frame_index) {
  rm_nav::mapping::MappingKeyframe keyframe;
  keyframe.stamp = rm_nav::common::FromNanoseconds(1000000000LL +
                                                   static_cast<rm_nav::common::TimeNs>(frame_index) *
                                                       1000000LL);
  keyframe.frame_index = frame_index;
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

std::size_t CellIndex(const rm_nav::data::GridMap2D& grid, float x, float y) {
  const int gx = static_cast<int>(std::floor((x - grid.origin.position.x) / grid.resolution_m));
  const int gy = static_cast<int>(std::floor((y - grid.origin.position.y) / grid.resolution_m));
  assert(gx >= 0 && gy >= 0);
  assert(gx < static_cast<int>(grid.width) && gy < static_cast<int>(grid.height));
  return static_cast<std::size_t>(gy) * grid.width + static_cast<std::size_t>(gx);
}

std::uint8_t CellAt(const rm_nav::data::GridMap2D& grid, float x, float y) {
  return grid.occupancy[CellIndex(grid, x, y)];
}

float LogOddsAt(const rm_nav::data::GridMap2D& grid, float x, float y) {
  return grid.occupancy_log_odds[CellIndex(grid, x, y)];
}

}  // namespace

int main() {
  rm_nav::config::MappingConfig config;
  config.z_min_m = 0.2;
  config.z_max_m = 1.2;
  config.occupancy_resolution_m = 0.1;
  config.occupancy_padding_m = 0.5;
  config.occupancy_hit_log_odds = 1.0;
  config.occupancy_miss_log_odds = 0.7;
  config.occupancy_min_log_odds = -1.5;
  config.occupancy_max_log_odds = 1.5;
  config.occupancy_free_threshold_log_odds = -0.3;
  config.occupancy_occupied_threshold_log_odds = 0.8;
  config.occupancy_inflation_radius_m = 0.15;

  rm_nav::mapping::MapProjector2D projector;
  assert(projector.Configure(config).ok());

  rm_nav::data::GridMap2D grid;
  const std::vector<rm_nav::data::PointXYZI> global_points = {{1.0F, 0.0F, 0.6F, 1.0F, 0.0F}};
  const std::vector<rm_nav::mapping::MappingKeyframe> keyframes = {MakeKeyframe(1U),
                                                                   MakeKeyframe(2U)};
  assert(projector.Project(global_points, keyframes, &grid).ok());

  const auto occupied = CellAt(grid, 1.0F, 0.0F);
  const auto occupied_log_odds = LogOddsAt(grid, 1.0F, 0.0F);
  const auto free_cell = CellAt(grid, 0.5F, 0.0F);
  const auto free_log_odds = LogOddsAt(grid, 0.5F, 0.0F);
  const auto unknown = CellAt(grid, -0.4F, 0.4F);
  const auto unknown_log_odds = LogOddsAt(grid, -0.4F, 0.4F);
  const auto inflated = CellAt(grid, 1.0F, 0.1F);
  const auto far_from_inflation = CellAt(grid, 1.0F, 0.3F);

  assert(occupied == 100U);
  assert(std::fabs(occupied_log_odds - 1.5F) < 1.0e-4F);
  assert(free_cell == 0U);
  assert(free_log_odds <= -0.3F);
  assert(unknown == 255U);
  assert(std::fabs(unknown_log_odds) < 1.0e-6F);
  assert(inflated >= 80U && inflated < 100U);
  assert(far_from_inflation == 255U);

  const auto output_dir =
      std::filesystem::current_path() / "logs" / "test_occupancy_projection_output";
  std::filesystem::remove_all(output_dir);
  rm_nav::mapping::MapSerializer serializer;
  rm_nav::mapping::MapArtifactPaths artifacts;
  assert(serializer.Save(output_dir, global_points, grid, &artifacts).ok());

  assert(std::filesystem::exists(output_dir / "occupancy.bin"));
  assert(std::filesystem::exists(output_dir / "occupancy.png"));
  assert(std::filesystem::exists(output_dir / "map_meta.json"));

  std::ifstream input(output_dir / "occupancy.bin", std::ios::binary);
  std::vector<std::uint8_t> saved((std::istreambuf_iterator<char>(input)),
                                  std::istreambuf_iterator<char>());
  assert(saved.size() == grid.occupancy.size());
  assert(saved[CellIndex(grid, 1.0F, 0.0F)] == occupied);
  assert(saved[CellIndex(grid, 0.5F, 0.0F)] == free_cell);
  assert(saved[CellIndex(grid, -0.4F, 0.4F)] == unknown);

  std::cout << "test_occupancy_projection passed\n";
  return 0;
}
