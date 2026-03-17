#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/point_types.hpp"

namespace rm_nav::mapping {

class VoxelMap {
 public:
  common::Status Configure(float voxel_size_m);
  void Clear();
  void Insert(const data::PointXYZI& point);
  void Insert(const std::vector<data::PointXYZI>& points);
  std::vector<data::PointXYZI> ExtractPoints() const;
  std::size_t voxel_count() const { return voxels_.size(); }
  float voxel_size_m() const { return voxel_size_m_; }

 private:
  struct VoxelKey {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const;
  };

  struct VoxelAccumulator {
    float sum_x{0.0F};
    float sum_y{0.0F};
    float sum_z{0.0F};
    float sum_intensity{0.0F};
    std::uint32_t count{0};
  };

  VoxelKey ToKey(const data::PointXYZI& point) const;

  float voxel_size_m_{0.1F};
  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels_{};
};

}  // namespace rm_nav::mapping
