#include "rm_nav/perception/euclidean_cluster.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

#include "rm_nav/perception/mot_manager.hpp"

namespace rm_nav::perception {
namespace {

float DistanceSq(const common::Vec3f& a, const common::Vec3f& b) {
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  return dx * dx + dy * dy;
}

}  // namespace

common::Status EuclideanCluster::Configure(const MotConfig& config) {
  cluster_tolerance_m_ = config.cluster_tolerance_m;
  min_cluster_points_ = config.min_cluster_points;
  max_cluster_points_ = config.max_cluster_points;
  roi_radius_m_ = config.roi_radius_m;
  reduced_roi_radius_m_ = config.reduced_roi_radius_m;
  configured_ = true;
  return common::Status::Ok();
}

common::Vec3f EuclideanCluster::TransformPointToWorld(const data::PointXYZI& point,
                                                      const data::Pose3f& pose) const {
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  common::Vec3f world;
  world.x = pose.position.x + cos_yaw * point.x - sin_yaw * point.y;
  world.y = pose.position.y + sin_yaw * point.x + cos_yaw * point.y;
  world.z = pose.position.z + point.z;
  return world;
}

common::Status EuclideanCluster::Build(const data::LidarFrame& filtered_frame,
                                       const data::Pose3f& pose,
                                       bool reduced_roi_active,
                                       std::vector<Cluster>* clusters) const {
  if (clusters == nullptr) {
    return common::Status::InvalidArgument("cluster output is null");
  }
  if (!configured_) {
    return common::Status::NotReady("euclidean cluster is not configured");
  }

  clusters->clear();
  if (filtered_frame.points.empty()) {
    return common::Status::Ok();
  }

  const float tolerance_sq = cluster_tolerance_m_ * cluster_tolerance_m_;
  const float active_roi_radius = reduced_roi_active ? reduced_roi_radius_m_ : roi_radius_m_;
  std::vector<common::Vec3f> world_points;
  world_points.reserve(filtered_frame.points.size());
  for (const auto& point : filtered_frame.points) {
    const float range_xy = std::sqrt(point.x * point.x + point.y * point.y);
    if (range_xy > active_roi_radius) {
      continue;
    }
    world_points.push_back(TransformPointToWorld(point, pose));
  }
  if (world_points.empty()) {
    return common::Status::Ok();
  }

  std::vector<bool> visited(world_points.size(), false);
  for (std::size_t seed = 0; seed < world_points.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }
    std::queue<std::size_t> queue;
    std::vector<std::size_t> members;
    queue.push(seed);
    visited[seed] = true;

    while (!queue.empty()) {
      const std::size_t current = queue.front();
      queue.pop();
      members.push_back(current);
      for (std::size_t index = 0; index < world_points.size(); ++index) {
        if (visited[index] || DistanceSq(world_points[current], world_points[index]) > tolerance_sq) {
          continue;
        }
        visited[index] = true;
        queue.push(index);
      }
    }

    if (members.size() < min_cluster_points_ || members.size() > max_cluster_points_) {
      continue;
    }

    Cluster cluster;
    common::Vec3f min_point = world_points[members.front()];
    common::Vec3f max_point = world_points[members.front()];
    for (const auto member : members) {
      const auto& point = world_points[member];
      cluster.centroid.x += point.x;
      cluster.centroid.y += point.y;
      cluster.centroid.z += point.z;
      min_point.x = std::min(min_point.x, point.x);
      min_point.y = std::min(min_point.y, point.y);
      max_point.x = std::max(max_point.x, point.x);
      max_point.y = std::max(max_point.y, point.y);
    }
    const float inv = 1.0F / static_cast<float>(members.size());
    cluster.centroid.x *= inv;
    cluster.centroid.y *= inv;
    cluster.centroid.z *= inv;
    const float dx = max_point.x - min_point.x;
    const float dy = max_point.y - min_point.y;
    cluster.radius_m = std::max(0.20F, 0.5F * std::sqrt(dx * dx + dy * dy));
    cluster.point_count = members.size();
    clusters->push_back(cluster);
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
