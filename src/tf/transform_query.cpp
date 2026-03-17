#include "rm_nav/tf/transform_query.hpp"

#include <array>
#include <cmath>

#include "rm_nav/data/tf_types.hpp"

namespace rm_nav::tf {
namespace {

using Mat3 = std::array<std::array<float, 3>, 3>;

Mat3 RotationFromRpy(const common::Vec3f& rpy) {
  const float cr = std::cos(rpy.x);
  const float sr = std::sin(rpy.x);
  const float cp = std::cos(rpy.y);
  const float sp = std::sin(rpy.y);
  const float cy = std::cos(rpy.z);
  const float sy = std::sin(rpy.z);

  return {{
      {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr}},
      {{sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr}},
      {{-sp, cp * sr, cp * cr}},
  }};
}

Mat3 Transpose(const Mat3& matrix) {
  Mat3 transposed{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      transposed[row][col] = matrix[col][row];
    }
  }
  return transposed;
}

Mat3 Multiply(const Mat3& lhs, const Mat3& rhs) {
  Mat3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      float value = 0.0F;
      for (std::size_t k = 0; k < 3; ++k) {
        value += lhs[row][k] * rhs[k][col];
      }
      result[row][col] = value;
    }
  }
  return result;
}

common::Vec3f Multiply(const Mat3& matrix, const common::Vec3f& vector) {
  common::Vec3f result;
  result.x = matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z;
  result.y = matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z;
  result.z = matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z;
  return result;
}

common::Vec3f ExtractRpy(const Mat3& rotation) {
  common::Vec3f rpy;
  rpy.y = std::asin(-rotation[2][0]);
  const float cos_pitch = std::cos(rpy.y);
  if (std::fabs(cos_pitch) > 1.0e-6F) {
    rpy.x = std::atan2(rotation[2][1], rotation[2][2]);
    rpy.z = std::atan2(rotation[1][0], rotation[0][0]);
  } else {
    rpy.x = 0.0F;
    rpy.z = std::atan2(-rotation[0][1], rotation[1][1]);
  }
  return rpy;
}

}  // namespace

data::Pose3f MakeTransform(std::string_view parent, std::string_view child, float x_m,
                           float y_m, float z_m, float roll_rad, float pitch_rad,
                           float yaw_rad, common::TimePoint stamp) {
  data::Pose3f pose;
  pose.stamp = stamp;
  pose.reference_frame = parent;
  pose.child_frame = child;
  pose.position.x = x_m;
  pose.position.y = y_m;
  pose.position.z = z_m;
  pose.rpy.x = roll_rad;
  pose.rpy.y = pitch_rad;
  pose.rpy.z = yaw_rad;
  pose.is_valid = true;
  return pose;
}

data::Pose3f MakeTransform(std::string_view parent, std::string_view child, float x_m,
                           float y_m, float yaw_rad, common::TimePoint stamp) {
  return MakeTransform(parent, child, x_m, y_m, 0.0F, 0.0F, 0.0F, yaw_rad, stamp);
}

data::Pose3f Inverse(const data::Pose3f& transform) {
  const Mat3 rotation = RotationFromRpy(transform.rpy);
  const Mat3 inverse_rotation = Transpose(rotation);
  const common::Vec3f rotated_translation = Multiply(inverse_rotation, transform.position);

  data::Pose3f inverse;
  inverse.stamp = transform.stamp;
  inverse.reference_frame = transform.child_frame;
  inverse.child_frame = transform.reference_frame;
  inverse.position.x = -rotated_translation.x;
  inverse.position.y = -rotated_translation.y;
  inverse.position.z = -rotated_translation.z;
  inverse.rpy = ExtractRpy(inverse_rotation);
  inverse.is_valid = transform.is_valid;
  return inverse;
}

data::Pose3f Compose(const data::Pose3f& parent_to_mid,
                     const data::Pose3f& mid_to_child) {
  const Mat3 parent_rotation = RotationFromRpy(parent_to_mid.rpy);
  const Mat3 child_rotation = RotationFromRpy(mid_to_child.rpy);
  const common::Vec3f rotated_translation = Multiply(parent_rotation, mid_to_child.position);

  data::Pose3f result;
  result.stamp = mid_to_child.stamp;
  result.reference_frame = parent_to_mid.reference_frame;
  result.child_frame = mid_to_child.child_frame;
  result.position.x = parent_to_mid.position.x + rotated_translation.x;
  result.position.y = parent_to_mid.position.y + rotated_translation.y;
  result.position.z = parent_to_mid.position.z + rotated_translation.z;
  result.rpy = ExtractRpy(Multiply(parent_rotation, child_rotation));
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
