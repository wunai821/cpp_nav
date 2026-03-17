#include "rm_nav/perception/mot_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "rm_nav/common/time.hpp"

namespace rm_nav::perception {
namespace {

float DistanceSq(const common::Vec3f& a, const common::Vec3f& b) {
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  return dx * dx + dy * dy;
}

void InitializeCovariance(std::array<float, 16>* covariance) {
  covariance->fill(0.0F);
  (*covariance)[0] = 0.5F;
  (*covariance)[5] = 0.5F;
  (*covariance)[10] = 1.0F;
  (*covariance)[15] = 1.0F;
}

}  // namespace

common::Status MotManager::Configure(const MotConfig& config) {
  if (config.cluster_tolerance_m <= 0.0F || config.min_cluster_points == 0U ||
      config.association_distance_m <= 0.0F || config.max_missed_frames == 0U) {
    return common::Status::InvalidArgument("invalid mot config");
  }
  config_ = config;
  tracks_.clear();
  next_track_id_ = 1;
  configured_ = true;
  return common::Status::Ok();
}

common::Vec3f MotManager::TransformPointToWorld(const data::PointXYZI& point,
                                                const data::Pose3f& pose) const {
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  common::Vec3f world;
  world.x = pose.position.x + cos_yaw * point.x - sin_yaw * point.y;
  world.y = pose.position.y + sin_yaw * point.x + cos_yaw * point.y;
  world.z = pose.position.z + point.z;
  return world;
}

std::vector<MotManager::Cluster> MotManager::BuildClusters(
    const data::LidarFrame& filtered_frame, const data::Pose3f& pose) const {
  std::vector<Cluster> clusters;
  if (filtered_frame.points.empty()) {
    return clusters;
  }

  const float tolerance_sq = config_.cluster_tolerance_m * config_.cluster_tolerance_m;
  const float active_roi_radius =
      reduce_roi_next_cycle_ ? config_.reduced_roi_radius_m : config_.roi_radius_m;
  std::vector<common::Vec3f> world_points;
  world_points.reserve(filtered_frame.points.size());
  for (const auto& point : filtered_frame.points) {
    const float range_xy = std::sqrt(point.x * point.x + point.y * point.y);
    if (range_xy > active_roi_radius) {
      continue;
    }
    world_points.push_back(TransformPointToWorld(point, pose));
  }
  const std::size_t point_count = world_points.size();
  if (point_count == 0U) {
    return clusters;
  }

  std::vector<bool> visited(point_count, false);
  for (std::size_t seed = 0; seed < point_count; ++seed) {
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
      for (std::size_t index = 0; index < point_count; ++index) {
        if (visited[index] ||
            DistanceSq(world_points[current], world_points[index]) > tolerance_sq) {
          continue;
        }
        visited[index] = true;
        queue.push(index);
      }
    }

    if (members.size() < config_.min_cluster_points ||
        members.size() > config_.max_cluster_points) {
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
    clusters.push_back(cluster);
  }

  return clusters;
}

void MotManager::PredictTrack(common::TimePoint stamp, Track* track) const {
  if (track == nullptr) {
    return;
  }
  const float dt =
      std::max(0.0F, static_cast<float>(common::ToNanoseconds(stamp - track->stamp)) / 1.0e9F);
  track->state[0] += track->state[2] * dt;
  track->state[1] += track->state[3] * dt;
  track->stamp = stamp;
  track->covariance[0] += config_.process_noise + dt;
  track->covariance[5] += config_.process_noise + dt;
  track->covariance[10] += config_.process_noise;
  track->covariance[15] += config_.process_noise;
}

void MotManager::UpdateTrack(const Cluster& cluster, common::TimePoint stamp, Track* track) const {
  if (track == nullptr) {
    return;
  }
  const float dt =
      std::max(1.0e-3F, static_cast<float>(common::ToNanoseconds(stamp - track->stamp)) / 1.0e9F);
  const float pred_x = track->state[0];
  const float pred_y = track->state[1];
  const float innovation_x = cluster.centroid.x - pred_x;
  const float innovation_y = cluster.centroid.y - pred_y;
  const float px_gain = track->covariance[0] / (track->covariance[0] + config_.measurement_noise);
  const float py_gain = track->covariance[5] / (track->covariance[5] + config_.measurement_noise);

  track->state[0] = pred_x + px_gain * innovation_x;
  track->state[1] = pred_y + py_gain * innovation_y;
  track->state[2] = 0.6F * track->state[2] + 0.4F * (innovation_x / dt);
  track->state[3] = 0.6F * track->state[3] + 0.4F * (innovation_y / dt);
  track->covariance[0] *= (1.0F - px_gain);
  track->covariance[5] *= (1.0F - py_gain);
  track->radius_m = 0.7F * track->radius_m + 0.3F * cluster.radius_m;
  track->confidence = std::min(1.0F, track->confidence + 0.18F);
  track->age += 1U;
  track->missed_frames = 0U;
  track->stamp = stamp;
  track->confirmed = track->confidence >= config_.confirmation_confidence;
}

data::DynamicObstacle MotManager::ToDynamicObstacle(const Track& track) const {
  data::DynamicObstacle obstacle;
  obstacle.id = track.id;
  obstacle.pose.stamp = track.stamp;
  obstacle.pose.reference_frame = tf::kMapFrame;
  obstacle.pose.child_frame = tf::kBaseLinkFrame;
  obstacle.pose.position.x = track.state[0];
  obstacle.pose.position.y = track.state[1];
  obstacle.pose.is_valid = true;
  obstacle.velocity.x = track.state[2];
  obstacle.velocity.y = track.state[3];
  obstacle.radius_m = track.radius_m;
  obstacle.confidence = track.confidence;
  obstacle.age = track.age;
  obstacle.missed_frames = track.missed_frames;
  obstacle.is_confirmed = track.confirmed;

  obstacle.predicted_pose_05s = obstacle.pose;
  obstacle.predicted_pose_05s.position.x += obstacle.velocity.x * 0.5F;
  obstacle.predicted_pose_05s.position.y += obstacle.velocity.y * 0.5F;
  obstacle.predicted_pose_10s = obstacle.pose;
  obstacle.predicted_pose_10s.position.x += obstacle.velocity.x * 1.0F;
  obstacle.predicted_pose_10s.position.y += obstacle.velocity.y * 1.0F;
  return obstacle;
}

common::Status MotManager::Update(const data::LidarFrame& filtered_frame,
                                  const data::Pose3f& pose,
                                  std::vector<data::DynamicObstacle>* obstacles) {
  if (obstacles == nullptr) {
    return common::Status::InvalidArgument("obstacle output is null");
  }
  if (!configured_) {
    Configure({});
  }

  const auto begin_ns = common::NowNs();
  obstacles->clear();
  const auto stamp = filtered_frame.stamp;
  const auto clustering_begin_ns = common::NowNs();
  const auto clusters = BuildClusters(filtered_frame, pose);
  const auto clustering_latency_ns = common::NowNs() - clustering_begin_ns;

  for (auto& track : tracks_) {
    PredictTrack(stamp, &track);
  }

  std::vector<bool> cluster_used(clusters.size(), false);
  for (auto& track : tracks_) {
    float best_dist_sq = config_.association_distance_m * config_.association_distance_m;
    std::size_t best_index = clusters.size();
    for (std::size_t index = 0; index < clusters.size(); ++index) {
      if (cluster_used[index]) {
        continue;
      }
      const float dx = clusters[index].centroid.x - track.state[0];
      const float dy = clusters[index].centroid.y - track.state[1];
      const float dist_sq = dx * dx + dy * dy;
      if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best_index = index;
      }
    }

    if (best_index < clusters.size()) {
      UpdateTrack(clusters[best_index], stamp, &track);
      cluster_used[best_index] = true;
    } else {
      track.missed_frames += 1U;
      track.confidence = std::max(0.0F, track.confidence - 0.2F);
    }
  }

  for (std::size_t index = 0; index < clusters.size(); ++index) {
    if (cluster_used[index]) {
      continue;
    }
    Track track;
    track.id = next_track_id_++;
    track.state[0] = clusters[index].centroid.x;
    track.state[1] = clusters[index].centroid.y;
    track.radius_m = clusters[index].radius_m;
    track.confidence = config_.initial_confidence;
    track.age = 1U;
    track.stamp = stamp;
    InitializeCovariance(&track.covariance);
    tracks_.push_back(track);
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [this](const Track& track) {
                                 return track.missed_frames > config_.max_missed_frames ||
                                        track.confidence <= 0.05F;
                               }),
                tracks_.end());

  for (const auto& track : tracks_) {
    data::DynamicObstacle obstacle = ToDynamicObstacle(track);
    if (!obstacle.is_confirmed && obstacle.confidence < config_.initial_confidence) {
      continue;
    }
    obstacles->push_back(obstacle);
  }
  MotPerfSnapshot perf;
  perf.clustering_latency_ns = clustering_latency_ns;
  perf.total_update_latency_ns = common::NowNs() - begin_ns;
  perf.active_roi_radius_m =
      reduce_roi_next_cycle_ ? config_.reduced_roi_radius_m : config_.roi_radius_m;
  perf.reduced_roi_active = reduce_roi_next_cycle_;
  latest_perf_.Publish(perf);
  reduce_roi_next_cycle_ =
      perf.total_update_latency_ns >
      static_cast<common::TimeNs>(config_.clustering_budget_ms) * 1000000LL;
  return common::Status::Ok();
}

common::Status MotManager::UpdateAndPublish(const data::LidarFrame& filtered_frame,
                                            const data::Pose3f& pose) {
  data::DynamicObstacleSet obstacle_set;
  obstacle_set.stamp = filtered_frame.stamp;
  const auto status = Update(filtered_frame, pose, &obstacle_set.obstacles);
  if (!status.ok()) {
    return status;
  }
  latest_obstacles_.Publish(std::move(obstacle_set));
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
