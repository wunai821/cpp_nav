#include "rm_nav/mapping/voxel_map.hpp"

#include <algorithm>
#include <cmath>

namespace rm_nav::mapping {

common::Status VoxelMap::Configure(float voxel_size_m) {
  if (voxel_size_m <= 0.0F) {
    return common::Status::InvalidArgument("voxel size must be positive");
  }
  voxel_size_m_ = voxel_size_m;
  return common::Status::Ok();
}

void VoxelMap::Clear() { voxels_.clear(); }

std::size_t VoxelMap::VoxelKeyHash::operator()(const VoxelKey& key) const {
  const std::size_t hx = static_cast<std::size_t>(key.x) * 73856093U;
  const std::size_t hy = static_cast<std::size_t>(key.y) * 19349663U;
  const std::size_t hz = static_cast<std::size_t>(key.z) * 83492791U;
  return hx ^ hy ^ hz;
}

VoxelMap::VoxelKey VoxelMap::ToKey(const data::PointXYZI& point) const {
  return {static_cast<int>(std::floor(point.x / voxel_size_m_)),
          static_cast<int>(std::floor(point.y / voxel_size_m_)),
          static_cast<int>(std::floor(point.z / voxel_size_m_))};
}

void VoxelMap::Insert(const data::PointXYZI& point) {
  auto& voxel = voxels_[ToKey(point)];
  voxel.sum_x += point.x;
  voxel.sum_y += point.y;
  voxel.sum_z += point.z;
  voxel.sum_intensity += point.intensity;
  ++voxel.count;
}

void VoxelMap::Insert(const std::vector<data::PointXYZI>& points) {
  for (const auto& point : points) {
    Insert(point);
  }
}

std::vector<data::PointXYZI> VoxelMap::ExtractPoints() const {
  std::vector<data::PointXYZI> points;
  points.reserve(voxels_.size());
  for (const auto& entry : voxels_) {
    const auto& acc = entry.second;
    if (acc.count == 0U) {
      continue;
    }
    data::PointXYZI point;
    const float inv_count = 1.0F / static_cast<float>(acc.count);
    point.x = acc.sum_x * inv_count;
    point.y = acc.sum_y * inv_count;
    point.z = acc.sum_z * inv_count;
    point.intensity = acc.sum_intensity * inv_count;
    points.push_back(point);
  }
  std::sort(points.begin(), points.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.x != rhs.x) {
      return lhs.x < rhs.x;
    }
    if (lhs.y != rhs.y) {
      return lhs.y < rhs.y;
    }
    return lhs.z < rhs.z;
  });
  return points;
}

}  // namespace rm_nav::mapping
