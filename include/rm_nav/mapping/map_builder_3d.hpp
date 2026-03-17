#pragma once

#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/mapping/voxel_map.hpp"

namespace rm_nav::mapping {

class MapBuilder3D {
 public:
  common::Status Configure(const config::MappingConfig& config);
  common::Status Update(const data::SyncedFrame& frame, const data::Pose3f& map_to_base);
  void Reset();
  std::vector<data::PointXYZI> GlobalPoints() const;
  std::size_t frame_count() const { return frame_count_; }

 private:
  VoxelMap voxel_map_{};
  std::size_t frame_count_{0};
  float synthetic_structure_z_m_{0.6F};
};

}  // namespace rm_nav::mapping
