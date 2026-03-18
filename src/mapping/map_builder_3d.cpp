#include "rm_nav/mapping/map_builder_3d.hpp"

#include <cmath>
#include <unordered_set>
#include <utility>

namespace rm_nav::mapping {
namespace {

data::PointXYZI TransformPoint(const data::PointXYZI& point, const data::Pose3f& pose) {
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  data::PointXYZI transformed = point;
  transformed.x = pose.position.x + cos_yaw * point.x - sin_yaw * point.y;
  transformed.y = pose.position.y + sin_yaw * point.x + cos_yaw * point.y;
  transformed.z = pose.position.z + point.z;
  return transformed;
}

}  // namespace

common::Status MapBuilder3D::Configure(const config::MappingConfig& config) {
  frame_count_ = 0;
  voxel_map_.Clear();
  pending_voxels_.clear();
  latest_dynamic_debug_ = {};
  voxel_size_m_ = static_cast<float>(config.voxel_size_m);
  synthetic_structure_z_m_ =
      0.5F * (static_cast<float>(config.z_min_m) + static_cast<float>(config.z_max_m));
  dynamic_suppression_enabled_ = config.dynamic_suppression_enabled;
  dynamic_near_field_radius_m_ = static_cast<float>(config.dynamic_near_field_radius_m);
  dynamic_consistency_frames_ = std::max(1, config.dynamic_consistency_frames);
  dynamic_pending_ttl_frames_ = std::max(1, config.dynamic_pending_ttl_frames);
  dynamic_known_obstacle_mask_enabled_ = config.dynamic_known_obstacle_mask_enabled;
  dynamic_known_obstacle_margin_m_ =
      static_cast<float>(config.dynamic_known_obstacle_margin_m);
  dynamic_known_obstacle_min_confidence_ =
      static_cast<float>(config.dynamic_known_obstacle_min_confidence);
  return voxel_map_.Configure(static_cast<float>(config.voxel_size_m));
}

common::Status MapBuilder3D::Update(const data::SyncedFrame& frame,
                                    const data::Pose3f& map_to_base) {
  const std::vector<data::DynamicObstacle> no_dynamic_obstacles;
  return Update(frame, map_to_base, no_dynamic_obstacles);
}

common::Status MapBuilder3D::Update(
    const data::SyncedFrame& frame, const data::Pose3f& map_to_base,
    const std::vector<data::DynamicObstacle>& known_dynamic_obstacles) {
  if (!map_to_base.is_valid) {
    return common::Status::InvalidArgument("mapping pose is invalid");
  }
  ++frame_count_;
  DynamicSuppressionDebugSnapshot dynamic_debug;
  dynamic_debug.frame_index = frame_count_;
  dynamic_debug.suppression_enabled = dynamic_suppression_enabled_;
  dynamic_debug.stale_pending_evictions = FlushStalePendingVoxels();
  std::vector<data::PointXYZI> transformed_points;
  transformed_points.reserve(frame.lidar.points.size());
  std::unordered_set<PendingVoxelKey, PendingVoxelKeyHash> frame_seen_voxels;
  frame_seen_voxels.reserve(frame.lidar.points.size());
  for (const auto& point : frame.lidar.points) {
    auto transformed = TransformPoint(point, map_to_base);
    if (std::fabs(transformed.z) < 0.05F) {
      transformed.z = synthetic_structure_z_m_;
    }
    if (dynamic_suppression_enabled_ &&
        std::hypot(point.x, point.y) < dynamic_near_field_radius_m_) {
      dynamic_debug.rejected_points.push_back(transformed);
      ++dynamic_debug.rejected_near_field;
      continue;
    }
    if (IsPointMaskedByKnownDynamicObstacle(transformed, known_dynamic_obstacles)) {
      dynamic_debug.rejected_points.push_back(transformed);
      ++dynamic_debug.rejected_known_obstacle_mask;
      continue;
    }
    if (!dynamic_suppression_enabled_ || dynamic_consistency_frames_ <= 1) {
      transformed_points.push_back(transformed);
      dynamic_debug.accepted_points.push_back(transformed);
      continue;
    }

    const auto key = ToPendingVoxelKey(transformed);
    if (!frame_seen_voxels.insert(key).second) {
      dynamic_debug.rejected_points.push_back(transformed);
      ++dynamic_debug.rejected_duplicate_voxel;
      continue;
    }

    auto& pending = pending_voxels_[key];
    pending.latest_point = transformed;
    if (pending.last_seen_frame != frame_count_) {
      ++pending.observation_count;
      pending.last_seen_frame = frame_count_;
    }
    if (pending.observation_count >= dynamic_consistency_frames_) {
      transformed_points.push_back(pending.latest_point);
      dynamic_debug.accepted_points.push_back(pending.latest_point);
      pending_voxels_.erase(key);
    }
  }
  dynamic_debug.pending_points.reserve(pending_voxels_.size());
  for (const auto& entry : pending_voxels_) {
    dynamic_debug.pending_points.push_back(entry.second.latest_point);
  }
  voxel_map_.Insert(transformed_points);
  latest_dynamic_debug_ = std::move(dynamic_debug);
  return common::Status::Ok();
}

void MapBuilder3D::Reset() {
  frame_count_ = 0;
  voxel_map_.Clear();
  pending_voxels_.clear();
  latest_dynamic_debug_ = {};
}

std::vector<data::PointXYZI> MapBuilder3D::GlobalPoints() const {
  return voxel_map_.ExtractPoints();
}

std::size_t MapBuilder3D::PendingVoxelKeyHash::operator()(const PendingVoxelKey& key) const {
  const std::size_t hx = static_cast<std::size_t>(key.x) * 73856093U;
  const std::size_t hy = static_cast<std::size_t>(key.y) * 19349663U;
  const std::size_t hz = static_cast<std::size_t>(key.z) * 83492791U;
  return hx ^ hy ^ hz;
}

MapBuilder3D::PendingVoxelKey MapBuilder3D::ToPendingVoxelKey(
    const data::PointXYZI& point) const {
  return {static_cast<int>(std::floor(point.x / voxel_size_m_)),
          static_cast<int>(std::floor(point.y / voxel_size_m_)),
          static_cast<int>(std::floor(point.z / voxel_size_m_))};
}

std::size_t MapBuilder3D::FlushStalePendingVoxels() {
  if (!dynamic_suppression_enabled_ || dynamic_consistency_frames_ <= 1) {
    const std::size_t cleared = pending_voxels_.size();
    pending_voxels_.clear();
    return cleared;
  }

  std::size_t evicted = 0;
  for (auto it = pending_voxels_.begin(); it != pending_voxels_.end();) {
    if (frame_count_ >
        it->second.last_seen_frame + static_cast<std::size_t>(dynamic_pending_ttl_frames_)) {
      ++evicted;
      it = pending_voxels_.erase(it);
    } else {
      ++it;
    }
  }
  return evicted;
}

bool MapBuilder3D::IsPointMaskedByKnownDynamicObstacle(
    const data::PointXYZI& transformed_point,
    const std::vector<data::DynamicObstacle>& known_dynamic_obstacles) const {
  if (!dynamic_known_obstacle_mask_enabled_) {
    return false;
  }

  for (const auto& obstacle : known_dynamic_obstacles) {
    if (!obstacle.pose.is_valid ||
        obstacle.confidence < dynamic_known_obstacle_min_confidence_) {
      continue;
    }

    const float mask_radius =
        std::max(0.1F, obstacle.radius_m + dynamic_known_obstacle_margin_m_);
    const float mask_radius_sq = mask_radius * mask_radius;
    const auto in_mask = [&](const data::Pose3f& obstacle_pose) {
      const float dx = transformed_point.x - obstacle_pose.position.x;
      const float dy = transformed_point.y - obstacle_pose.position.y;
      return dx * dx + dy * dy <= mask_radius_sq;
    };
    if (in_mask(obstacle.pose) || in_mask(obstacle.predicted_pose_05s) ||
        in_mask(obstacle.predicted_pose_10s)) {
      return true;
    }
  }

  return false;
}

}  // namespace rm_nav::mapping
