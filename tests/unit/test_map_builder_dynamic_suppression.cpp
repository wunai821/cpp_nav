#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/mapping/map_builder_3d.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

rm_nav::data::Pose3f MakePose() {
  rm_nav::data::Pose3f pose;
  pose.stamp = rm_nav::common::Now();
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.is_valid = true;
  return pose;
}

rm_nav::data::SyncedFrame MakeFrame(std::uint32_t frame_index, bool include_far_point) {
  rm_nav::data::SyncedFrame frame;
  frame.stamp = rm_nav::common::Now();
  frame.lidar.stamp = frame.stamp;
  frame.lidar.scan_begin_stamp = frame.stamp;
  frame.lidar.scan_end_stamp = frame.stamp;
  frame.lidar.frame_index = frame_index;
  frame.lidar.is_deskewed = true;

  rm_nav::data::PointXYZI near_point;
  near_point.x = 0.30F;
  near_point.y = 0.00F;
  near_point.z = 0.50F;
  near_point.intensity = 1.0F;
  frame.lidar.points.push_back(near_point);

  if (include_far_point) {
    rm_nav::data::PointXYZI far_point;
    far_point.x = 2.0F;
    far_point.y = 0.0F;
    far_point.z = 0.50F;
    far_point.intensity = 1.0F;
    frame.lidar.points.push_back(far_point);
  }

  return frame;
}

}  // namespace

int main() {
  rm_nav::config::MappingConfig config;
  config.voxel_size_m = 0.1;
  config.z_min_m = 0.2;
  config.z_max_m = 1.2;
  config.dynamic_suppression_enabled = true;
  config.dynamic_near_field_radius_m = 1.0;
  config.dynamic_consistency_frames = 2;
  config.dynamic_pending_ttl_frames = 2;

  rm_nav::mapping::MapBuilder3D builder;
  assert(builder.Configure(config).ok());

  const auto pose = MakePose();
  assert(builder.Update(MakeFrame(1U, true), pose).ok());
  assert(builder.GlobalPoints().empty());

  assert(builder.Update(MakeFrame(2U, true), pose).ok());
  const auto stable_points = builder.GlobalPoints();
  assert(stable_points.size() == 1U);
  assert(std::fabs(stable_points.front().x - 2.0F) < 0.15F);
  assert(std::fabs(stable_points.front().y) < 0.15F);

  assert(builder.Update(MakeFrame(3U, false), pose).ok());
  const auto after_near_only = builder.GlobalPoints();
  assert(after_near_only.size() == 1U);

  std::vector<rm_nav::data::DynamicObstacle> obstacles;
  rm_nav::data::DynamicObstacle obstacle;
  obstacle.pose = MakePose();
  obstacle.pose.position.x = 2.0F;
  obstacle.pose.position.y = 0.0F;
  obstacle.predicted_pose_05s = obstacle.pose;
  obstacle.predicted_pose_10s = obstacle.pose;
  obstacle.radius_m = 0.2F;
  obstacle.confidence = 0.9F;
  obstacles.push_back(obstacle);

  rm_nav::mapping::MapBuilder3D masked_builder;
  assert(masked_builder.Configure(config).ok());
  assert(masked_builder.Update(MakeFrame(10U, true), pose, obstacles).ok());
  assert(masked_builder.Update(MakeFrame(11U, true), pose, obstacles).ok());
  assert(masked_builder.GlobalPoints().empty());

  std::cout << "test_map_builder_dynamic_suppression passed\n";
  return 0;
}
