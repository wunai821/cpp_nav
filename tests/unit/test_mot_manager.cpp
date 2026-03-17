#include <cassert>
#include <iostream>

#include "rm_nav/perception/mot_manager.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

rm_nav::data::LidarFrame MakeClusterFrame(float offset_x) {
  rm_nav::data::LidarFrame frame;
  frame.stamp = rm_nav::common::Now();
  frame.frame_id = rm_nav::tf::kLaserFrame;
  const float base_x = 2.0F + offset_x;
  for (int index = 0; index < 4; ++index) {
    rm_nav::data::PointXYZI point;
    point.x = base_x + 0.05F * static_cast<float>(index % 2);
    point.y = 1.0F + 0.05F * static_cast<float>(index / 2);
    point.z = 0.2F;
    frame.points.push_back(point);
  }
  for (int index = 0; index < 4; ++index) {
    rm_nav::data::PointXYZI point;
    point.x = 4.0F + 0.05F * static_cast<float>(index % 2);
    point.y = -0.5F + 0.05F * static_cast<float>(index / 2);
    point.z = 0.25F;
    frame.points.push_back(point);
  }
  return frame;
}

}  // namespace

int main() {
  rm_nav::perception::MotManager mot;
  assert(mot.Configure({}).ok());

  rm_nav::data::Pose3f pose;
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.is_valid = true;

  std::vector<rm_nav::data::DynamicObstacle> obstacles;
  assert(mot.Update(MakeClusterFrame(0.0F), pose, &obstacles).ok());
  assert(obstacles.size() == 2U);

  assert(mot.Update(MakeClusterFrame(0.25F), pose, &obstacles).ok());
  assert(obstacles.size() == 2U);
  bool found_confirmed_moving = false;
  for (const auto& obstacle : obstacles) {
    if (obstacle.is_confirmed && obstacle.velocity.x > 0.05F) {
      found_confirmed_moving = true;
    }
  }
  assert(found_confirmed_moving);

  rm_nav::data::LidarFrame empty_frame;
  empty_frame.stamp = rm_nav::common::Now();
  for (int index = 0; index < 5; ++index) {
    assert(mot.Update(empty_frame, pose, &obstacles).ok());
  }
  assert(obstacles.empty());

  std::cout << "test_mot_manager passed\n";
  return 0;
}
