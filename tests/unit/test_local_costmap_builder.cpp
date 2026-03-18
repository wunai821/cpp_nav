#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iostream>

#include "rm_nav/perception/local_costmap_builder.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

std::uint8_t CellAt(const rm_nav::data::GridMap2D& costmap, float local_x, float local_y) {
  const int center_x = static_cast<int>(costmap.width / 2U);
  const int center_y = static_cast<int>(costmap.height / 2U);
  const int gx = center_x + static_cast<int>(local_x / costmap.resolution_m + 0.5F);
  const int gy = center_y + static_cast<int>(local_y / costmap.resolution_m + 0.5F);
  assert(gx >= 0 && gy >= 0);
  assert(gx < static_cast<int>(costmap.width));
  assert(gy < static_cast<int>(costmap.height));
  return costmap.occupancy[static_cast<std::size_t>(gy) * costmap.width +
                           static_cast<std::size_t>(gx)];
}

}  // namespace

int main() {
  rm_nav::perception::LocalCostmapBuilder builder;
  assert(builder.Configure({}).ok());

  rm_nav::data::Pose3f pose;
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.is_valid = true;

  rm_nav::data::LidarFrame frame;
  frame.frame_id = rm_nav::tf::kLaserFrame;
  frame.stamp = rm_nav::common::Now();

  rm_nav::data::PointXYZI obstacle_point;
  obstacle_point.x = 1.0F;
  obstacle_point.y = 0.0F;
  obstacle_point.z = 0.20F;
  frame.points.push_back(obstacle_point);

  rm_nav::data::PointXYZI low_point;
  low_point.x = -2.0F;
  low_point.y = -2.0F;
  low_point.z = 0.0F;
  frame.points.push_back(low_point);

  rm_nav::data::DynamicObstacle dynamic_obstacle;
  dynamic_obstacle.pose = pose;
  dynamic_obstacle.pose.position.x = 0.5F;
  dynamic_obstacle.pose.position.y = 0.5F;
  dynamic_obstacle.pose.is_valid = true;
  dynamic_obstacle.predicted_pose_05s = dynamic_obstacle.pose;
  dynamic_obstacle.predicted_pose_05s.position.x = 0.8F;
  dynamic_obstacle.predicted_pose_05s.position.y = 0.5F;
  dynamic_obstacle.predicted_pose_05s.is_valid = true;
  dynamic_obstacle.predicted_pose_10s = dynamic_obstacle.predicted_pose_05s;
  dynamic_obstacle.radius_m = 0.20F;
  dynamic_obstacle.confidence = 0.95F;

  rm_nav::data::GridMap2D costmap;
  assert(builder.Build(frame, pose, {dynamic_obstacle}, &costmap).ok());
  assert(costmap.width == 80U);
  assert(costmap.height == 80U);

  const auto static_cost = CellAt(costmap, 1.0F, 0.0F);
  const auto inflation_cost = CellAt(costmap, 1.2F, 0.0F);
  const auto dynamic_cost = CellAt(costmap, 0.5F, 0.5F);
  const auto dynamic_predicted_cost = CellAt(costmap, 0.8F, 0.5F);

  assert(static_cost >= 100U);
  assert(inflation_cost >= 80U);
  assert(dynamic_cost >= 100U);
  assert(dynamic_predicted_cost >= 100U);

  const auto inflated_cells =
      std::count_if(costmap.occupancy.begin(), costmap.occupancy.end(),
                    [](std::uint8_t cost) { return cost >= 80U; });
  const auto occupied_cells =
      std::count_if(costmap.occupancy.begin(), costmap.occupancy.end(),
                    [](std::uint8_t cost) { return cost >= 100U; });
  assert(inflated_cells > occupied_cells);

  std::cout << "test_local_costmap_builder passed\n";
  return 0;
}
