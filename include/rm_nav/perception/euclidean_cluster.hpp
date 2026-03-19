#pragma once

#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/common/types.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::perception {

struct MotConfig;

struct Cluster {
  common::Vec3f centroid{};
  float radius_m{0.0F};
  std::size_t point_count{0};
};

class EuclideanCluster {
 public:
  common::Status Configure(const MotConfig& config);
  common::Status Build(const data::LidarFrame& filtered_frame, const data::Pose3f& pose,
                       bool reduced_roi_active, std::vector<Cluster>* clusters) const;

 private:
  common::Vec3f TransformPointToWorld(const data::PointXYZI& point,
                                      const data::Pose3f& pose) const;

  float cluster_tolerance_m_{0.45F};
  std::size_t min_cluster_points_{3U};
  std::size_t max_cluster_points_{128U};
  float roi_radius_m_{6.0F};
  float reduced_roi_radius_m_{3.5F};
  bool configured_{false};
};

}  // namespace rm_nav::perception
