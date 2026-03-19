#include "rm_nav/perception/tracker_kf.hpp"

#include <algorithm>
#include <cmath>

#include "rm_nav/tf/frame_ids.hpp"
#include "rm_nav/perception/mot_manager.hpp"

namespace rm_nav::perception {
namespace {

float Clamp01(float value) {
  return std::max(0.0F, std::min(1.0F, value));
}

void InitializeCovariance(std::array<float, 16>* covariance) {
  covariance->fill(0.0F);
  (*covariance)[0] = 0.5F;
  (*covariance)[5] = 0.5F;
  (*covariance)[10] = 1.0F;
  (*covariance)[15] = 1.0F;
}

float ComputePredictedRadius(float radius_m, float vx, float vy) {
  const float speed = std::sqrt(vx * vx + vy * vy);
  return radius_m + std::min(0.35F, 0.08F + 0.25F * speed);
}

float ComputeRiskScore(float confidence, float vx, float vy, std::uint32_t age,
                       std::uint32_t missed_frames, bool confirmed) {
  const float speed = std::sqrt(vx * vx + vy * vy);
  const float speed_term = Clamp01(speed / 1.2F);
  const float age_term = Clamp01(static_cast<float>(age) / 4.0F);
  const float confirmed_term = confirmed ? 1.0F : 0.35F;
  const float confidence_term = Clamp01(confidence);
  const float missed_penalty = Clamp01(static_cast<float>(missed_frames) / 3.0F);
  return Clamp01(0.45F * confidence_term + 0.25F * speed_term + 0.20F * confirmed_term +
                 0.10F * age_term - 0.20F * missed_penalty);
}

data::DynamicObstacleRiskLevel ClassifyRiskLevel(float risk_score) {
  if (risk_score >= 0.72F) {
    return data::DynamicObstacleRiskLevel::kHigh;
  }
  if (risk_score >= 0.42F) {
    return data::DynamicObstacleRiskLevel::kMedium;
  }
  return data::DynamicObstacleRiskLevel::kLow;
}

}  // namespace

common::Status TrackerKf::Configure(const MotConfig& config) {
  association_distance_m_ = config.association_distance_m;
  max_missed_frames_ = config.max_missed_frames;
  process_noise_ = config.process_noise;
  measurement_noise_ = config.measurement_noise;
  initial_confidence_ = config.initial_confidence;
  confirmation_confidence_ = config.confirmation_confidence;
  Reset();
  configured_ = true;
  return common::Status::Ok();
}

void TrackerKf::Reset() {
  tracks_.clear();
  next_track_id_ = 1;
}

void TrackerKf::PredictTrack(common::TimePoint stamp, Track* track) const {
  if (track == nullptr) {
    return;
  }
  const float dt =
      std::max(0.0F, static_cast<float>(common::ToNanoseconds(stamp - track->stamp)) / 1.0e9F);
  track->state[0] += track->state[2] * dt;
  track->state[1] += track->state[3] * dt;
  track->stamp = stamp;
  track->covariance[0] += process_noise_ + dt;
  track->covariance[5] += process_noise_ + dt;
  track->covariance[10] += process_noise_;
  track->covariance[15] += process_noise_;
}

void TrackerKf::UpdateTrack(const Cluster& cluster, common::TimePoint stamp, Track* track) const {
  if (track == nullptr) {
    return;
  }
  const float dt =
      std::max(1.0e-3F, static_cast<float>(common::ToNanoseconds(stamp - track->stamp)) / 1.0e9F);
  const float pred_x = track->state[0];
  const float pred_y = track->state[1];
  const float innovation_x = cluster.centroid.x - pred_x;
  const float innovation_y = cluster.centroid.y - pred_y;
  const float px_gain = track->covariance[0] / (track->covariance[0] + measurement_noise_);
  const float py_gain = track->covariance[5] / (track->covariance[5] + measurement_noise_);

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
  track->confirmed = track->confidence >= confirmation_confidence_;
}

data::DynamicObstacle TrackerKf::ToDynamicObstacle(const Track& track) const {
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
  obstacle.predicted_radius_m =
      ComputePredictedRadius(track.radius_m, track.state[2], track.state[3]);
  obstacle.confidence = track.confidence;
  obstacle.risk_score =
      ComputeRiskScore(track.confidence, track.state[2], track.state[3], track.age,
                       track.missed_frames, track.confirmed);
  obstacle.age = track.age;
  obstacle.missed_frames = track.missed_frames;
  obstacle.is_confirmed = track.confirmed;
  obstacle.risk_level = ClassifyRiskLevel(obstacle.risk_score);
  obstacle.predicted_pose_05s = obstacle.pose;
  obstacle.predicted_pose_05s.position.x += obstacle.velocity.x * 0.5F;
  obstacle.predicted_pose_05s.position.y += obstacle.velocity.y * 0.5F;
  obstacle.predicted_pose_10s = obstacle.pose;
  obstacle.predicted_pose_10s.position.x += obstacle.velocity.x * 1.0F;
  obstacle.predicted_pose_10s.position.y += obstacle.velocity.y * 1.0F;
  return obstacle;
}

common::Status TrackerKf::Update(const std::vector<Cluster>& clusters, common::TimePoint stamp,
                                 std::vector<data::DynamicObstacle>* obstacles) {
  if (obstacles == nullptr) {
    return common::Status::InvalidArgument("tracker obstacle output is null");
  }
  if (!configured_) {
    return common::Status::NotReady("tracker is not configured");
  }

  obstacles->clear();
  for (auto& track : tracks_) {
    PredictTrack(stamp, &track);
  }

  std::vector<bool> cluster_used(clusters.size(), false);
  for (auto& track : tracks_) {
    float best_dist_sq = association_distance_m_ * association_distance_m_;
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
    track.confidence = initial_confidence_;
    track.age = 1U;
    track.stamp = stamp;
    InitializeCovariance(&track.covariance);
    tracks_.push_back(track);
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [this](const Track& track) {
                                 return track.missed_frames > max_missed_frames_ ||
                                        track.confidence <= 0.05F;
                               }),
                tracks_.end());

  for (const auto& track : tracks_) {
    auto obstacle = ToDynamicObstacle(track);
    if (!obstacle.is_confirmed && obstacle.confidence < initial_confidence_) {
      continue;
    }
    obstacles->push_back(obstacle);
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
