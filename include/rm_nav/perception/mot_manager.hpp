#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "rm_nav/common/double_buffer.hpp"
#include "rm_nav/common/status.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/local_map.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/perception/euclidean_cluster.hpp"
#include "rm_nav/perception/tracker_kf.hpp"

namespace rm_nav::perception {

struct MotConfig {
  float cluster_tolerance_m{0.45F};
  std::size_t min_cluster_points{3};
  std::size_t max_cluster_points{128};
  float roi_radius_m{6.0F};
  float reduced_roi_radius_m{3.5F};
  float association_distance_m{1.0F};
  std::uint32_t max_missed_frames{4};
  float process_noise{0.08F};
  float measurement_noise{0.15F};
  float initial_confidence{0.45F};
  float confirmation_confidence{0.6F};
  int clustering_budget_ms{3};
};

struct MotPerfSnapshot {
  common::TimeNs clustering_latency_ns{0};
  common::TimeNs total_update_latency_ns{0};
  float active_roi_radius_m{0.0F};
  bool reduced_roi_active{false};
};

class MotManager {
 public:
  common::Status Configure(const MotConfig& config);
  common::Status Update(
      const data::LidarFrame& filtered_frame, const data::Pose3f& pose,
      std::vector<data::DynamicObstacle>* obstacles);
  common::Status UpdateAndPublish(const data::LidarFrame& filtered_frame,
                                  const data::Pose3f& pose);
  data::DynamicObstacleSet LatestObstacles() const {
    return latest_obstacles_.ReadSnapshot();
  }
  MotPerfSnapshot LatestPerf() const { return latest_perf_.ReadSnapshot(); }

 private:
  MotConfig config_{};
  EuclideanCluster clusterer_{};
  TrackerKf tracker_{};
  common::DoubleBuffer<data::DynamicObstacleSet> latest_obstacles_{};
  common::DoubleBuffer<MotPerfSnapshot> latest_perf_{};
  bool reduce_roi_next_cycle_{false};
  bool configured_{false};
};

}  // namespace rm_nav::perception
