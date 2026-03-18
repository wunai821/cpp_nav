#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

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

rm_nav::data::PointXYZI MakePoint(float x, float y, float z = 0.5F, float intensity = 1.0F) {
  rm_nav::data::PointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

rm_nav::data::DynamicObstacle MakeObstacle(float x, float y, float radius_m) {
  rm_nav::data::DynamicObstacle obstacle;
  obstacle.pose = MakePose();
  obstacle.pose.position.x = x;
  obstacle.pose.position.y = y;
  obstacle.predicted_pose_05s = obstacle.pose;
  obstacle.predicted_pose_10s = obstacle.pose;
  obstacle.radius_m = radius_m;
  obstacle.confidence = 0.95F;
  return obstacle;
}

rm_nav::data::SyncedFrame MakeFrame(std::uint32_t frame_index,
                                    const std::vector<rm_nav::data::PointXYZI>& points) {
  rm_nav::data::SyncedFrame frame;
  frame.stamp = rm_nav::common::Now();
  frame.lidar.stamp = frame.stamp;
  frame.lidar.scan_begin_stamp = frame.stamp;
  frame.lidar.scan_end_stamp = frame.stamp;
  frame.lidar.frame_index = frame_index;
  frame.lidar.is_deskewed = true;
  frame.lidar.points = points;
  return frame;
}

bool ContainsPointNear(const std::vector<rm_nav::data::PointXYZI>& points, float x, float y,
                       float tolerance_m = 0.15F) {
  for (const auto& point : points) {
    if (std::fabs(point.x - x) <= tolerance_m && std::fabs(point.y - y) <= tolerance_m) {
      return true;
    }
  }
  return false;
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
  config.dynamic_known_obstacle_mask_enabled = true;
  config.dynamic_known_obstacle_margin_m = 0.25;
  config.dynamic_known_obstacle_min_confidence = 0.35;

  rm_nav::mapping::MapBuilder3D builder;
  assert(builder.Configure(config).ok());

  const auto pose = MakePose();
  const auto masked_obstacle = MakeObstacle(2.8F, 0.8F, 0.2F);
  const std::vector<rm_nav::data::DynamicObstacle> known_obstacles{masked_obstacle};

  assert(builder.Update(
             MakeFrame(1U, {MakePoint(3.0F, 0.0F), MakePoint(1.5F, -1.2F), MakePoint(0.4F, 0.0F),
                            MakePoint(2.8F, 0.8F)}),
             pose, known_obstacles)
             .ok());
  auto debug = builder.LatestDynamicSuppressionDebug();
  assert(debug.accepted_points.empty());
  assert(debug.pending_points.size() == 2U);
  assert(debug.rejected_points.size() == 2U);
  assert(debug.rejected_near_field == 1U);
  assert(debug.rejected_known_obstacle_mask == 1U);
  assert(ContainsPointNear(debug.pending_points, 3.0F, 0.0F));
  assert(ContainsPointNear(debug.pending_points, 1.5F, -1.2F));

  assert(builder.Update(
             MakeFrame(2U, {MakePoint(3.0F, 0.0F), MakePoint(1.8F, -0.7F), MakePoint(0.4F, 0.0F),
                            MakePoint(2.8F, 0.8F)}),
             pose, known_obstacles)
             .ok());
  debug = builder.LatestDynamicSuppressionDebug();
  assert(debug.accepted_points.size() == 1U);
  assert(ContainsPointNear(debug.accepted_points, 3.0F, 0.0F));
  assert(debug.pending_points.size() == 2U);
  assert(ContainsPointNear(debug.pending_points, 1.5F, -1.2F));
  assert(ContainsPointNear(debug.pending_points, 1.8F, -0.7F));
  assert(debug.rejected_near_field == 1U);
  assert(debug.rejected_known_obstacle_mask == 1U);

  assert(builder.Update(
             MakeFrame(3U, {MakePoint(2.1F, -0.2F), MakePoint(0.4F, 0.0F), MakePoint(2.8F, 0.8F)}),
             pose, known_obstacles)
             .ok());
  assert(builder.Update(
             MakeFrame(4U, {MakePoint(2.4F, 0.3F), MakePoint(0.4F, 0.0F), MakePoint(2.8F, 0.8F)}),
             pose, known_obstacles)
             .ok());

  debug = builder.LatestDynamicSuppressionDebug();
  assert(debug.stale_pending_evictions >= 1U);
  assert(debug.pending_points.size() >= 2U);
  assert(debug.rejected_points.size() == 2U);

  const auto global_points = builder.GlobalPoints();
  assert(global_points.size() == 1U);
  assert(ContainsPointNear(global_points, 3.0F, 0.0F));
  assert(!ContainsPointNear(global_points, 1.5F, -1.2F));
  assert(!ContainsPointNear(global_points, 1.8F, -0.7F));
  assert(!ContainsPointNear(global_points, 2.1F, -0.2F));
  assert(!ContainsPointNear(global_points, 2.4F, 0.3F));
  assert(!ContainsPointNear(global_points, 2.8F, 0.8F));

  std::cout << "test_dynamic_suppression_validation passed\n";
  return 0;
}
