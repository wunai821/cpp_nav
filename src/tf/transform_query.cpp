#include "rm_nav/tf/transform_query.hpp"

#include <cmath>

#include "rm_nav/data/tf_types.hpp"

namespace rm_nav::tf {

data::Pose3f MakeTransform(std::string_view parent, std::string_view child, float x_m,
                           float y_m, float yaw_rad, common::TimePoint stamp) {
  data::Pose3f pose;
  pose.stamp = stamp;
  pose.reference_frame = parent;
  pose.child_frame = child;
  pose.position.x = x_m;
  pose.position.y = y_m;
  pose.rpy.z = yaw_rad;
  pose.is_valid = true;
  return pose;
}

data::Pose3f Inverse(const data::Pose3f& transform) {
  const float yaw = transform.rpy.z;
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);

  data::Pose3f inverse;
  inverse.stamp = transform.stamp;
  inverse.reference_frame = transform.child_frame;
  inverse.child_frame = transform.reference_frame;
  inverse.position.x = -(cos_yaw * transform.position.x + sin_yaw * transform.position.y);
  inverse.position.y = -(-sin_yaw * transform.position.x + cos_yaw * transform.position.y);
  inverse.rpy.z = -yaw;
  inverse.is_valid = transform.is_valid;
  return inverse;
}

data::Pose3f Compose(const data::Pose3f& parent_to_mid,
                     const data::Pose3f& mid_to_child) {
  const float yaw = parent_to_mid.rpy.z;
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);

  data::Pose3f result;
  result.stamp = mid_to_child.stamp;
  result.reference_frame = parent_to_mid.reference_frame;
  result.child_frame = mid_to_child.child_frame;
  result.position.x =
      parent_to_mid.position.x + cos_yaw * mid_to_child.position.x -
      sin_yaw * mid_to_child.position.y;
  result.position.y =
      parent_to_mid.position.y + sin_yaw * mid_to_child.position.x +
      cos_yaw * mid_to_child.position.y;
  result.rpy.z = parent_to_mid.rpy.z + mid_to_child.rpy.z;
  result.is_valid = parent_to_mid.is_valid && mid_to_child.is_valid;
  return result;
}

common::Status LookupTransform(const TfTreeLite& tree, std::string_view parent,
                               std::string_view child, common::TimePoint stamp,
                               data::Pose3f* transform) {
  if (transform == nullptr) {
    return common::Status::InvalidArgument("transform output is null");
  }
  const auto result = tree.Lookup(parent, child, stamp);
  if (!result.has_value()) {
    return common::Status::NotReady("transform is not available");
  }
  *transform = *result;
  return common::Status::Ok();
}

}  // namespace rm_nav::tf
