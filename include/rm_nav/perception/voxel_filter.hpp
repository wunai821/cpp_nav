#pragma once

#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/point_types.hpp"

namespace rm_nav::perception {

struct PreprocessConfig;

class VoxelFilter {
 public:
  common::Status Configure(const PreprocessConfig& config);
  common::Status Apply(const std::vector<data::PointXYZI>& input,
                       std::vector<data::PointXYZI>* output) const;

 private:
  float voxel_size_m_{0.15F};
  bool configured_{false};
};

}  // namespace rm_nav::perception
