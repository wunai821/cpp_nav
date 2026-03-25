#include <cassert>
#include <iostream>

#include "rm_nav/perception/preprocess_pipeline.hpp"
#include "rm_nav/tf/frame_ids.hpp"

int main() {
  rm_nav::perception::PreprocessPipeline pipeline;
  rm_nav::perception::PreprocessConfig config;
  config.min_range_m = 0.5F;
  config.max_range_m = 5.0F;
  config.min_height_m = -0.2F;
  config.max_height_m = 1.0F;
  config.ground_z_max_m = -0.05F;
  config.voxel_size_m = 0.2F;
  config.self_mask_enabled = true;
  config.self_mask_x_min_m = 0.8F;
  config.self_mask_x_max_m = 1.2F;
  config.self_mask_y_min_m = -0.2F;
  config.self_mask_y_max_m = 0.2F;
  config.lidar_mount.z_m = 0.2F;
  assert(pipeline.Configure(config).ok());

  rm_nav::data::SyncedFrame frame;
  frame.lidar.is_deskewed = true;
  frame.lidar.points.push_back({0.1F, 0.0F, 0.1F, 1.0F, 0.0F});
  frame.lidar.points.push_back({6.0F, 0.0F, 0.1F, 1.0F, 0.0F});
  frame.lidar.points.push_back({1.0F, 0.0F, -0.1F, 1.0F, 0.0F});
  frame.lidar.points.push_back({1.0F, 0.0F, 1.2F, 1.0F, 0.0F});
  frame.lidar.points.push_back({1.0F, 0.0F, 0.2F, 1.0F, 0.0F});
  frame.lidar.points.push_back({1.0F, 1.0F, 0.2F, 1.0F, 0.0F});
  frame.lidar.points.push_back({1.05F, 1.02F, 0.22F, 1.0F, 0.0F});
  frame.lidar.points.push_back({2.0F, 0.5F, 0.3F, 1.0F, 0.0F});
  frame.lidar.points.push_back({1.6F, 0.4F, -0.12F, 1.0F, 0.0F});

  rm_nav::data::LidarFrame filtered;
  assert(pipeline.Run(frame, &filtered).ok());
  assert(filtered.is_deskewed);
  assert(filtered.frame_id == rm_nav::tf::kBaseLinkFrame);
  assert(filtered.points.size() == 3U);
  bool found_height_corrected_obstacle = false;
  for (const auto& point : filtered.points) {
    if (point.x > 1.5F && point.y > 0.3F && point.z > 0.05F) {
      found_height_corrected_obstacle = true;
    }
  }
  assert(found_height_corrected_obstacle);

  std::cout << "test_preprocess_pipeline passed\n";
  return 0;
}
