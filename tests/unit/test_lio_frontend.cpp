#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/mapping/lio_frontend.hpp"

namespace {

rm_nav::data::Pose3f MakePose(float x, float y, float yaw) {
  rm_nav::data::Pose3f pose;
  pose.stamp = rm_nav::common::Now();
  pose.position.x = x;
  pose.position.y = y;
  pose.rpy.z = yaw;
  pose.is_valid = true;
  return pose;
}

}  // namespace

int main() {
  rm_nav::config::MappingConfig config;
  config.frontend_motion_compensation_enabled = true;
  config.frontend_motion_compensation_min_yaw_rad = 0.01;
  config.frontend_imu_yaw_weight = 1.0;
  config.frontend_imu_position_weight = 0.0;

  rm_nav::mapping::LioFrontend frontend;
  assert(frontend.Configure(config).ok());

  rm_nav::mapping::LioFrontendPrediction prediction;
  rm_nav::data::PreintegratedImuBlock preint;
  preint.is_valid = true;
  preint.sample_count = 4U;
  preint.delta_rpy.z = 0.20F;
  preint.duration = std::chrono::milliseconds(100);
  assert(frontend.PredictPose(MakePose(0.0F, 0.0F, 0.0F), MakePose(0.0F, 0.0F, 0.0F),
                              MakePose(0.0F, 0.0F, 0.0F), preint, &prediction)
             .ok());
  assert(prediction.used_imu_prediction);
  assert(prediction.motion_compensation_recommended);
  assert(std::fabs(prediction.predicted_pose.rpy.z - 0.20F) < 0.02F);

  rm_nav::data::SyncedFrame frame;
  frame.preint = preint;
  rm_nav::data::PointXYZI early_point;
  early_point.x = 1.0F;
  early_point.relative_time_s = 0.0F;
  rm_nav::data::PointXYZI late_point;
  late_point.x = 1.0F;
  late_point.relative_time_s = 0.10F;
  frame.lidar.points = {early_point, late_point};

  rm_nav::data::SyncedFrame prepared;
  assert(frontend.PrepareFrame(frame, &prepared).ok());
  assert(std::fabs(prepared.lidar.points[0].x - 1.0F) < 1.0e-4F);
  assert(std::fabs(prepared.lidar.points[1].y) > 0.05F);

  std::cout << "test_lio_frontend passed\n";
  return 0;
}
