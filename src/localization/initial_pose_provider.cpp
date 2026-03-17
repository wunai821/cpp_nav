#include "rm_nav/localization/initial_pose_provider.hpp"

#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::localization {

data::Pose3f InitialPoseProvider::MakeInitialPose(common::TimePoint stamp) const {
  data::Pose3f pose;
  pose.stamp = stamp;
  pose.reference_frame = tf::kMapFrame;
  pose.child_frame = tf::kBaseLinkFrame;
  pose.position.x = static_cast<float>(config_.x_m);
  pose.position.y = static_cast<float>(config_.y_m);
  pose.rpy.z = static_cast<float>(config_.theta_rad);
  pose.is_valid = true;
  return pose;
}

}  // namespace rm_nav::localization
