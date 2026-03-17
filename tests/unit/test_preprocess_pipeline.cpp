#include <cassert>
#include <iostream>

#include "rm_nav/perception/preprocess_pipeline.hpp"

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

  rm_nav::data::LidarFrame filtered;
  assert(pipeline.Run(frame, &filtered).ok());
  assert(filtered.is_deskewed);
  assert(filtered.points.size() == 2U);

  std::cout << "test_preprocess_pipeline passed\n";
  return 0;
}
