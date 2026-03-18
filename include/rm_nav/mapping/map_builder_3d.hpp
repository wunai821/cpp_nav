#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/mapping/voxel_map.hpp"

namespace rm_nav::mapping {

class MapBuilder3D {
 public:
  struct DynamicSuppressionDebugSnapshot {
    std::size_t frame_index{0};
    bool suppression_enabled{true};
    std::size_t stale_pending_evictions{0};
    std::size_t rejected_near_field{0};
    std::size_t rejected_known_obstacle_mask{0};
    std::size_t rejected_duplicate_voxel{0};
    std::vector<data::PointXYZI> pending_points;
    std::vector<data::PointXYZI> accepted_points;
    std::vector<data::PointXYZI> rejected_points;
  };

  common::Status Configure(const config::MappingConfig& config);
  common::Status Update(const data::SyncedFrame& frame, const data::Pose3f& map_to_base);
  common::Status Update(const data::SyncedFrame& frame, const data::Pose3f& map_to_base,
                        const std::vector<data::DynamicObstacle>& known_dynamic_obstacles);
  void Reset();
  std::vector<data::PointXYZI> GlobalPoints() const;
  std::size_t frame_count() const { return frame_count_; }
  DynamicSuppressionDebugSnapshot LatestDynamicSuppressionDebug() const {
    return latest_dynamic_debug_;
  }

 private:
  struct PendingVoxelKey {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const PendingVoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct PendingVoxelKeyHash {
    std::size_t operator()(const PendingVoxelKey& key) const;
  };

  struct PendingVoxel {
    data::PointXYZI latest_point{};
    int observation_count{0};
    std::size_t last_seen_frame{0};
  };

  PendingVoxelKey ToPendingVoxelKey(const data::PointXYZI& point) const;
  std::size_t FlushStalePendingVoxels();
  bool IsPointMaskedByKnownDynamicObstacle(
      const data::PointXYZI& transformed_point,
      const std::vector<data::DynamicObstacle>& known_dynamic_obstacles) const;

  VoxelMap voxel_map_{};
  std::unordered_map<PendingVoxelKey, PendingVoxel, PendingVoxelKeyHash> pending_voxels_{};
  std::size_t frame_count_{0};
  float synthetic_structure_z_m_{0.6F};
  float voxel_size_m_{0.1F};
  bool dynamic_suppression_enabled_{true};
  float dynamic_near_field_radius_m_{1.0F};
  int dynamic_consistency_frames_{2};
  int dynamic_pending_ttl_frames_{3};
  bool dynamic_known_obstacle_mask_enabled_{true};
  float dynamic_known_obstacle_margin_m_{0.25F};
  float dynamic_known_obstacle_min_confidence_{0.35F};
  DynamicSuppressionDebugSnapshot latest_dynamic_debug_{};
};

}  // namespace rm_nav::mapping
