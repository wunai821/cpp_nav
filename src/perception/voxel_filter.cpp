#include "rm_nav/perception/voxel_filter.hpp"

#include <cmath>
#include <cstdint>
#include <unordered_set>

#include "rm_nav/perception/preprocess_pipeline.hpp"

namespace rm_nav::perception {
namespace {

std::uint64_t VoxelKey(int vx, int vy, int vz) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(vx)) << 42U) ^
         (static_cast<std::uint64_t>(static_cast<std::uint32_t>(vy)) << 21U) ^
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(vz));
}

}  // namespace

common::Status VoxelFilter::Configure(const PreprocessConfig& config) {
  voxel_size_m_ = config.voxel_size_m;
  configured_ = true;
  return common::Status::Ok();
}

common::Status VoxelFilter::Apply(const std::vector<data::PointXYZI>& input,
                                  std::vector<data::PointXYZI>* output) const {
  if (output == nullptr) {
    return common::Status::InvalidArgument("voxel filter output is null");
  }
  if (!configured_) {
    return common::Status::NotReady("voxel filter is not configured");
  }

  output->clear();
  output->reserve(input.size());
  std::unordered_set<std::uint64_t> occupied_voxels;
  occupied_voxels.reserve(input.size());
  for (const auto& point : input) {
    const int vx = static_cast<int>(std::floor(point.x / voxel_size_m_));
    const int vy = static_cast<int>(std::floor(point.y / voxel_size_m_));
    const int vz = static_cast<int>(std::floor(point.z / voxel_size_m_));
    const auto key = VoxelKey(vx, vy, vz);
    if (!occupied_voxels.insert(key).second) {
      continue;
    }
    output->push_back(point);
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
