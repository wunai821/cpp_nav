#include "rm_nav/localization/icp_matcher.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::localization {
namespace {

rm_nav::data::PointXYZI TransformPoint(const rm_nav::data::PointXYZI& point,
                                       const rm_nav::data::Pose3f& pose) {
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  rm_nav::data::PointXYZI transformed = point;
  transformed.x = pose.position.x + cos_yaw * point.x - sin_yaw * point.y;
  transformed.y = pose.position.y + sin_yaw * point.x + cos_yaw * point.y;
  transformed.z = pose.position.z + point.z;
  return transformed;
}

float NormalizeAngle(float angle) {
  while (angle > static_cast<float>(M_PI)) {
    angle -= static_cast<float>(2.0 * M_PI);
  }
  while (angle < static_cast<float>(-M_PI)) {
    angle += static_cast<float>(2.0 * M_PI);
  }
  return angle;
}

}  // namespace

common::Status IcpMatcher::Configure(const ScanMatchConfig& config) {
  if (config.max_iterations <= 0 || config.correspondence_distance_m <= 0.0F ||
      config.min_match_score <= 0.0F) {
    return common::Status::InvalidArgument("invalid icp matcher config");
  }
  config_ = config;
  return common::Status::Ok();
}

common::Status IcpMatcher::Match(const StaticMap& map, const data::LidarFrame& scan,
                                 const data::Pose3f& initial_guess,
                                 ScanMatchResult* result) const {
  if (result == nullptr) {
    return common::Status::InvalidArgument("scan match result output is null");
  }
  if (map.global_points.empty() || scan.points.empty()) {
    return common::Status::NotReady("scan matcher needs map points and scan points");
  }

  *result = {};
  data::Pose3f current = initial_guess;
  current.reference_frame = tf::kMapFrame;
  current.child_frame = tf::kBaseLinkFrame;
  current.is_valid = true;

  float best_score = 0.0F;
  std::size_t best_correspondences = 0U;

  for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
    float sum_dx = 0.0F;
    float sum_dy = 0.0F;
    float sum_dyaw = 0.0F;
    float sum_error = 0.0F;
    std::size_t correspondences = 0U;

    for (const auto& scan_point : scan.points) {
      const auto transformed = TransformPoint(scan_point, current);
      float best_dist_sq = std::numeric_limits<float>::max();
      const data::PointXYZI* best_match = nullptr;
      for (const auto& map_point : map.global_points) {
        const float dx = map_point.x - transformed.x;
        const float dy = map_point.y - transformed.y;
        const float dist_sq = dx * dx + dy * dy;
        if (dist_sq < best_dist_sq) {
          best_dist_sq = dist_sq;
          best_match = &map_point;
        }
      }
      if (best_match == nullptr ||
          best_dist_sq >
              config_.correspondence_distance_m * config_.correspondence_distance_m) {
        continue;
      }

      const float ex = best_match->x - transformed.x;
      const float ey = best_match->y - transformed.y;
      sum_dx += ex;
      sum_dy += ey;
      const float radius_sq = scan_point.x * scan_point.x + scan_point.y * scan_point.y;
      if (radius_sq > 1.0e-4F) {
        sum_dyaw += (-scan_point.y * ex + scan_point.x * ey) / radius_sq;
      }
      sum_error += std::sqrt(best_dist_sq);
      ++correspondences;
    }

    result->iterations = iteration + 1;
    if (correspondences == 0U) {
      break;
    }

    const float inv = 1.0F / static_cast<float>(correspondences);
    current.position.x += sum_dx * inv;
    current.position.y += sum_dy * inv;
    current.rpy.z = NormalizeAngle(current.rpy.z + sum_dyaw * inv * 0.5F);

    best_correspondences = correspondences;
    const float mean_error = sum_error * inv;
    best_score = 1.0F / (1.0F + mean_error);
    if (std::fabs(sum_dx * inv) < 1.0e-3F && std::fabs(sum_dy * inv) < 1.0e-3F &&
        std::fabs(sum_dyaw * inv) < 1.0e-3F) {
      break;
    }
  }

  result->matched_pose = current;
  result->matched_pose.reference_frame = tf::kMapFrame;
  result->matched_pose.child_frame = tf::kBaseLinkFrame;
  result->matched_pose.is_valid = best_correspondences > 0U;
  result->score = best_score;
  result->correspondence_count = best_correspondences;
  result->converged =
      best_correspondences >= std::max<std::size_t>(8U, scan.points.size() / 8U) &&
      best_score >= config_.min_match_score;
  return common::Status::Ok();
}

}  // namespace rm_nav::localization
