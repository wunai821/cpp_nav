#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "rm_nav/common/time.hpp"
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

rm_nav::data::SyncedFrame MakeFrame(std::uint32_t frame_index, std::size_t point_count) {
  rm_nav::data::SyncedFrame frame;
  frame.stamp = rm_nav::common::Now();
  frame.lidar.stamp = frame.stamp;
  frame.lidar.scan_begin_stamp = frame.stamp;
  frame.lidar.scan_end_stamp = frame.stamp;
  frame.lidar.frame_index = frame_index;
  frame.lidar.is_deskewed = true;
  for (std::size_t index = 0; index < point_count; ++index) {
    rm_nav::data::PointXYZI point;
    point.x = 0.05F * static_cast<float>(index);
    point.y = 0.02F * static_cast<float>(index);
    point.z = 0.6F;
    point.intensity = 1.0F;
    frame.lidar.points.push_back(point);
  }
  return frame;
}

}  // namespace

int main() {
  rm_nav::config::MappingConfig config;
  config.voxel_size_m = 0.1;
  config.z_min_m = 0.2;
  config.z_max_m = 1.2;
  config.occupancy_resolution_m = 0.1;
  config.keyframe_translation_threshold_m = 0.5;
  config.keyframe_yaw_threshold_rad = 0.2;
  config.max_keyframes = 2;
  config.keyframe_max_points = 3;

  rm_nav::mapping::MappingEngine engine;
  assert(engine.Initialize(config).ok());

  assert(engine.Update(MakeFrame(1U, 6U), MakePose(0.0F, 0.0F, 0.0F)).ok());
  assert(engine.Keyframes().size() == 1U);
  assert(engine.Keyframes().front().frame_index == 1U);
  assert(engine.Keyframes().front().local_points.size() == 3U);

  assert(engine.Update(MakeFrame(2U, 6U), MakePose(0.1F, 0.1F, 0.05F)).ok());
  assert(engine.Keyframes().size() == 1U);

  assert(engine.Update(MakeFrame(3U, 6U), MakePose(0.7F, 0.1F, 0.05F)).ok());
  assert(engine.Keyframes().size() == 2U);
  assert(engine.Keyframes().back().frame_index == 3U);
  assert(!engine.LatestLoopCandidate().found);

  assert(engine.Update(MakeFrame(4U, 6U), MakePose(0.72F, 0.1F, 0.35F)).ok());
  assert(engine.Keyframes().size() == 2U);
  assert(engine.Keyframes().front().frame_index == 3U);
  assert(engine.Keyframes().back().frame_index == 4U);

  std::cout << "test_mapping_keyframe_cache passed\n";
  return 0;
}
