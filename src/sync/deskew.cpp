#include "rm_nav/sync/deskew.hpp"

#include <cmath>

namespace rm_nav::sync {
namespace {

rm_nav::data::PointXYZI RotateYaw(const rm_nav::data::PointXYZI& point, float yaw_rad) {
  const float cos_yaw = std::cos(yaw_rad);
  const float sin_yaw = std::sin(yaw_rad);
  rm_nav::data::PointXYZI rotated = point;
  rotated.x = cos_yaw * point.x - sin_yaw * point.y;
  rotated.y = sin_yaw * point.x + cos_yaw * point.y;
  return rotated;
}

}  // namespace

common::Status Deskew::Apply(const data::LidarFrame& input,
                             const data::PreintegratedImuBlock& preint,
                             data::LidarFrame* output) const {
  if (output == nullptr) {
    return common::Status::InvalidArgument("deskew output is null");
  }

  *output = input;
  output->points.clear();
  output->points.reserve(input.points.size());

  const float scan_duration_s =
      static_cast<float>(common::ToNanoseconds(input.scan_end_stamp - input.scan_begin_stamp)) /
      1.0e9F;
  if (!preint.is_valid || scan_duration_s <= 0.0F) {
    output->points = input.points;
    output->is_deskewed = false;
    return common::Status::Ok();
  }

  for (const auto& point : input.points) {
    const float alpha = point.relative_time_s <= 0.0F
                            ? 0.0F
                            : std::min(1.0F, point.relative_time_s / scan_duration_s);
    const float yaw_correction = -preint.delta_rpy.z * alpha;
    const rm_nav::common::Vec3f translation = {
        -preint.delta_position.x * alpha,
        -preint.delta_position.y * alpha,
        -preint.delta_position.z * alpha,
    };
    auto corrected = RotateYaw(point, yaw_correction);
    corrected.x += translation.x;
    corrected.y += translation.y;
    corrected.z += translation.z;
    output->points.push_back(corrected);
  }

  output->stamp = input.scan_end_stamp;
  output->is_deskewed = true;
  return common::Status::Ok();
}

}  // namespace rm_nav::sync
