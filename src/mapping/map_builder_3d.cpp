#include "rm_nav/mapping/map_builder_3d.hpp"

#include <cmath>

namespace rm_nav::mapping {
namespace {

data::PointXYZI TransformPoint(const data::PointXYZI& point, const data::Pose3f& pose) {
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  data::PointXYZI transformed = point;
  transformed.x = pose.position.x + cos_yaw * point.x - sin_yaw * point.y;
  transformed.y = pose.position.y + sin_yaw * point.x + cos_yaw * point.y;
  transformed.z = pose.position.z + point.z;
  return transformed;
}

}  // namespace

common::Status MapBuilder3D::Configure(const config::MappingConfig& config) {
  frame_count_ = 0;
  voxel_map_.Clear();
  synthetic_structure_z_m_ =
      0.5F * (static_cast<float>(config.z_min_m) + static_cast<float>(config.z_max_m));
  return voxel_map_.Configure(static_cast<float>(config.voxel_size_m));
}

common::Status MapBuilder3D::Update(const data::SyncedFrame& frame,
                                    const data::Pose3f& map_to_base) {
  if (!map_to_base.is_valid) {
    return common::Status::InvalidArgument("mapping pose is invalid");
  }
  std::vector<data::PointXYZI> transformed_points;
  transformed_points.reserve(frame.lidar.points.size());
  for (const auto& point : frame.lidar.points) {
    auto transformed = TransformPoint(point, map_to_base);
    if (std::fabs(transformed.z) < 0.05F) {
      transformed.z = synthetic_structure_z_m_;
    }
    transformed_points.push_back(transformed);
  }
  voxel_map_.Insert(transformed_points);
  ++frame_count_;
  return common::Status::Ok();
}

void MapBuilder3D::Reset() {
  frame_count_ = 0;
  voxel_map_.Clear();
}

std::vector<data::PointXYZI> MapBuilder3D::GlobalPoints() const {
  return voxel_map_.ExtractPoints();
}

}  // namespace rm_nav::mapping
