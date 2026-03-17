#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/mapping/mapping_engine.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

rm_nav::data::Pose3f MakePose(float x, float y, float yaw) {
  rm_nav::data::Pose3f pose;
  pose.stamp = rm_nav::common::Now();
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.position.x = x;
  pose.position.y = y;
  pose.rpy.z = yaw;
  pose.is_valid = true;
  return pose;
}

rm_nav::data::PointXYZI ToLocal(const rm_nav::data::PointXYZI& world_point,
                                const rm_nav::data::Pose3f& pose) {
  const float dx = world_point.x - pose.position.x;
  const float dy = world_point.y - pose.position.y;
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  rm_nav::data::PointXYZI local = world_point;
  local.x = cos_yaw * dx + sin_yaw * dy;
  local.y = -sin_yaw * dx + cos_yaw * dy;
  return local;
}

rm_nav::data::SyncedFrame MakeFrame(const std::vector<rm_nav::data::PointXYZI>& world_points,
                                    const rm_nav::data::Pose3f& pose,
                                    std::uint32_t frame_index) {
  rm_nav::data::SyncedFrame frame;
  frame.stamp = pose.stamp;
  frame.lidar.stamp = pose.stamp;
  frame.lidar.scan_begin_stamp = pose.stamp;
  frame.lidar.scan_end_stamp = pose.stamp;
  frame.lidar.frame_index = frame_index;
  frame.lidar.is_deskewed = true;
  for (const auto& point : world_points) {
    frame.lidar.points.push_back(ToLocal(point, pose));
  }
  return frame;
}

}  // namespace

int main() {
  const std::filesystem::path output_dir("logs/test_mapping_pipeline_output");
  std::filesystem::create_directories(output_dir);

  rm_nav::config::MappingConfig config;
  config.output_dir = output_dir.string();
  config.voxel_size_m = 0.1;
  config.z_min_m = 0.2;
  config.z_max_m = 1.2;
  config.occupancy_resolution_m = 0.1;

  rm_nav::mapping::MappingEngine engine;
  assert(engine.Initialize(config).ok());

  std::vector<rm_nav::data::PointXYZI> world_points = {
      {1.0F, 1.0F, 0.5F, 1.0F, 0.0F}, {1.2F, 1.0F, 0.6F, 1.0F, 0.0F},
      {1.4F, 1.1F, 0.7F, 1.0F, 0.0F}, {2.0F, 1.8F, 0.9F, 1.0F, 0.0F},
      {2.2F, 1.9F, 0.1F, 1.0F, 0.0F}};

  const auto pose0 = MakePose(0.0F, 0.0F, 0.0F);
  const auto pose1 = MakePose(0.2F, 0.1F, 0.05F);
  const auto pose2 = MakePose(0.4F, 0.2F, 0.08F);
  assert(engine.Update(MakeFrame(world_points, pose0, 0U), pose0).ok());
  assert(engine.Update(MakeFrame(world_points, pose1, 1U), pose1).ok());
  assert(engine.Update(MakeFrame(world_points, pose2, 2U), pose2).ok());

  rm_nav::localization::StaticMap saved_map;
  assert(engine.SaveMap(output_dir.string(), &saved_map).ok());
  assert(saved_map.global_map_loaded);
  assert(saved_map.occupancy_loaded);
  assert(!saved_map.global_points.empty());

  assert(std::filesystem::exists(output_dir / "global_map.pcd"));
  assert(std::filesystem::exists(output_dir / "occupancy.bin"));
  assert(std::filesystem::exists(output_dir / "occupancy.png"));
  assert(std::filesystem::exists(output_dir / "map_meta.json"));

  rm_nav::localization::MapLoader loader;
  rm_nav::config::LocalizationConfig localization_config;
  localization_config.global_map_pcd_path = "global_map.pcd";
  localization_config.occupancy_path = "occupancy.bin";
  localization_config.map_meta_path = "map_meta.json";
  rm_nav::localization::StaticMap loaded_map;
  assert(loader.Load(output_dir.string(), localization_config, &loaded_map).ok());
  assert(loaded_map.global_map_loaded);
  assert(loaded_map.occupancy_loaded);
  assert(!loaded_map.global_points.empty());
  assert(loaded_map.occupancy.width > 0U);
  assert(loaded_map.occupancy.height > 0U);

  std::cout << "test_mapping_pipeline passed\n";
  return 0;
}
