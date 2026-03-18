#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/mapping/mapping_engine.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

rm_nav::data::Pose3f MakePose(rm_nav::common::TimePoint stamp, float x, float y, float yaw) {
  rm_nav::data::Pose3f pose;
  pose.stamp = stamp;
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.position.x = x;
  pose.position.y = y;
  pose.rpy.z = yaw;
  pose.is_valid = true;
  return pose;
}

rm_nav::data::PointXYZI ToLocal(const rm_nav::data::PointXYZI& world_point, float x, float y,
                                float yaw, float relative_time_s) {
  const float dx = world_point.x - x;
  const float dy = world_point.y - y;
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  rm_nav::data::PointXYZI local = world_point;
  local.x = cos_yaw * dx + sin_yaw * dy;
  local.y = -sin_yaw * dx + cos_yaw * dy;
  local.relative_time_s = relative_time_s;
  return local;
}

std::vector<rm_nav::data::PointXYZI> MakeWorldPoints() {
  std::vector<rm_nav::data::PointXYZI> points;
  for (int index = 0; index < 64; ++index) {
    rm_nav::data::PointXYZI point;
    point.x = 0.8F + 0.05F * static_cast<float>(index % 8);
    point.y = -1.2F + 0.10F * static_cast<float>(index / 8);
    point.z = 0.6F;
    point.intensity = 1.0F;
    points.push_back(point);
  }
  return points;
}

rm_nav::data::SyncedFrame MakeFrame(const std::vector<rm_nav::data::PointXYZI>& world_points,
                                    rm_nav::common::TimePoint stamp, std::uint32_t frame_index,
                                    float true_x, float true_y, float true_yaw) {
  rm_nav::data::SyncedFrame frame;
  frame.stamp = stamp;
  frame.lidar.stamp = stamp;
  frame.lidar.scan_begin_stamp = stamp;
  frame.lidar.scan_end_stamp = stamp;
  frame.lidar.frame_index = frame_index;
  frame.lidar.frame_id = rm_nav::tf::kLaserFrame;
  frame.lidar.is_deskewed = true;
  for (std::size_t index = 0; index < world_points.size(); ++index) {
    frame.lidar.points.push_back(
        ToLocal(world_points[index], true_x, true_y, true_yaw, static_cast<float>(index) * 0.001F));
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
  config.pose_source = "scan_to_map_icp";
  config.frontend_match_max_iterations = 12;
  config.frontend_correspondence_distance_m = 0.8;
  config.frontend_min_match_score = 0.2;
  config.frontend_match_max_points = 96;
  config.frontend_min_map_points = 20;
  config.frontend_submap_radius_m = 1.4;
  config.frontend_submap_max_points = 40;
  config.keyframe_translation_threshold_m = 0.4;
  config.keyframe_yaw_threshold_rad = 0.2;
  config.loop_correction_apply_translation_step_m = 0.0;
  config.loop_correction_apply_yaw_step_rad = 0.0;

  rm_nav::mapping::MappingEngine engine;
  assert(engine.Initialize(config).ok());

  const auto world_points = MakeWorldPoints();
  const auto t0 = rm_nav::common::Now();
  const auto raw0 = MakePose(t0, 0.0F, 0.0F, 0.0F);
  assert(engine.Update(MakeFrame(world_points, t0, 1U, 0.0F, 0.0F, 0.0F), raw0).ok());

  const auto raw1 = MakePose(t0 + std::chrono::seconds(1), 0.0F, 0.0F, 0.0F);
  assert(engine.Update(MakeFrame(world_points, t0 + std::chrono::seconds(1), 2U, 0.0F, 0.0F, 0.0F),
                       raw1)
             .ok());

  const auto raw2 = MakePose(t0 + std::chrono::seconds(2), 0.25F, 0.0F, 0.0F);
  assert(engine.Update(MakeFrame(world_points, t0 + std::chrono::seconds(2), 3U, 0.0F, 0.0F, 0.0F),
                       raw2)
             .ok());

  const auto result = engine.LatestResult();
  assert(result.pose_source == "scan_to_map_icp");
  assert(result.frontend_converged);
  assert(!result.frontend_fallback);
  assert(result.frontend_reference_points >= 20U);
  assert(result.frontend_reference_points <= 40U);
  assert(std::fabs(result.external_pose.position.x - raw2.position.x) < 1.0e-4F);
  assert(std::fabs(result.predicted_pose.position.x - raw2.position.x) < 1.0e-4F);
  assert(std::fabs(result.map_to_base.position.x) < 0.12F);
  assert(std::fabs(result.map_to_base.position.x) < std::fabs(raw2.position.x));

  std::cout << "test_mapping_frontend_scan_to_map passed\n";
  return 0;
}
