#include "rm_nav/safety/collision_checker.hpp"

#include <algorithm>
#include <cmath>

namespace rm_nav::safety {
namespace {

float NormalizeAngle(float angle) {
  constexpr float kPi = 3.14159265358979323846F;
  while (angle > kPi) {
    angle -= 2.0F * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0F * kPi;
  }
  return angle;
}

std::pair<float, float> RotateOffset(float x, float y, float yaw) {
  return {std::cos(yaw) * x - std::sin(yaw) * y, std::sin(yaw) * x + std::cos(yaw) * y};
}

}  // namespace

CollisionChecker::FootprintSamples CollisionChecker::BuildFootprintSamples(
    float local_x, float local_y, float local_yaw, const config::SafetyConfig& config) const {
  const float half_length = std::max(0.0F, static_cast<float>(config.footprint_half_length_m));
  const float half_width = std::max(0.0F, static_cast<float>(config.footprint_half_width_m));
  const std::array<std::pair<float, float>, 5> offsets = {{
      {0.0F, 0.0F},
      {half_length, half_width},
      {half_length, -half_width},
      {0.0F, half_width},
      {0.0F, -half_width},
  }};

  FootprintSamples samples{};
  for (std::size_t index = 0; index < offsets.size(); ++index) {
    const auto rotated = RotateOffset(offsets[index].first, offsets[index].second, local_yaw);
    samples[index] = {local_x + rotated.first, local_y + rotated.second};
  }
  return samples;
}

bool CollisionChecker::CostmapCollisionAtLocal(const data::GridMap2D& costmap, float local_x,
                                               float local_y) const {
  if (costmap.width == 0U || costmap.height == 0U || costmap.occupancy.empty() ||
      costmap.resolution_m <= 0.0F) {
    return false;
  }

  const int center_x = static_cast<int>(costmap.width / 2U);
  const int center_y = static_cast<int>(costmap.height / 2U);
  const int gx = center_x + static_cast<int>(std::round(local_x / costmap.resolution_m));
  const int gy = center_y + static_cast<int>(std::round(local_y / costmap.resolution_m));
  if (gx < 0 || gy < 0 || gx >= static_cast<int>(costmap.width) ||
      gy >= static_cast<int>(costmap.height)) {
    return true;
  }
  return costmap.occupancy[static_cast<std::size_t>(gy) * costmap.width +
                           static_cast<std::size_t>(gx)] >= 50U;
}

bool CollisionChecker::DynamicCollisionAhead(
    const data::Pose3f& pose, const data::ChassisCmd& cmd,
    const std::vector<data::DynamicObstacle>& obstacles,
    const config::SafetyConfig& config) const {
  float local_x = 0.0F;
  float local_y = 0.0F;
  float local_yaw = 0.0F;
  const float lookahead_s = static_cast<float>(config.collision_check_lookahead_s);
  const float dt_s = std::max(0.02F, static_cast<float>(config.collision_check_dt_s));
  for (float t = dt_s; t <= lookahead_s + 1.0e-5F; t += dt_s) {
    local_x += (cmd.vx_mps * std::cos(local_yaw) - cmd.vy_mps * std::sin(local_yaw)) * dt_s;
    local_y += (cmd.vx_mps * std::sin(local_yaw) + cmd.vy_mps * std::cos(local_yaw)) * dt_s;
    local_yaw = NormalizeAngle(local_yaw + cmd.wz_radps * dt_s);
    const auto footprint_samples = BuildFootprintSamples(local_x, local_y, local_yaw, config);
    for (const auto& obstacle : obstacles) {
      const auto& predicted_pose =
          t <= 0.5F ? obstacle.predicted_pose_05s : obstacle.predicted_pose_10s;
      for (const auto& sample : footprint_samples) {
        const float world_x =
            pose.position.x + std::cos(pose.rpy.z) * sample.first -
            std::sin(pose.rpy.z) * sample.second;
        const float world_y =
            pose.position.y + std::sin(pose.rpy.z) * sample.first +
            std::cos(pose.rpy.z) * sample.second;
        const float dx = predicted_pose.position.x - world_x;
        const float dy = predicted_pose.position.y - world_y;
        const float predicted_radius =
            std::max(obstacle.radius_m, obstacle.predicted_radius_m);
        const float clearance = std::sqrt(dx * dx + dy * dy) - predicted_radius;
        if (clearance <= static_cast<float>(config.emergency_stop_distance_m)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool CollisionChecker::ObstacleTooClose(const data::Pose3f& pose,
                                        const std::vector<data::DynamicObstacle>& obstacles,
                                        float threshold_m,
                                        const config::SafetyConfig& config) const {
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  const auto footprint_samples = BuildFootprintSamples(0.0F, 0.0F, 0.0F, config);
  for (const auto& obstacle : obstacles) {
    if (!obstacle.pose.is_valid) {
      continue;
    }
    for (const auto& sample : footprint_samples) {
      const float world_x = pose.position.x + cos_yaw * sample.first - sin_yaw * sample.second;
      const float world_y = pose.position.y + sin_yaw * sample.first + cos_yaw * sample.second;
      const float dx = obstacle.pose.position.x - world_x;
      const float dy = obstacle.pose.position.y - world_y;
      const float clearance = std::sqrt(dx * dx + dy * dy) - obstacle.radius_m;
      if (clearance <= threshold_m) {
        return true;
      }
    }
  }
  return false;
}

CollisionCheckResult CollisionChecker::Evaluate(
    const data::Pose3f& pose, const data::GridMap2D& costmap,
    const std::vector<data::DynamicObstacle>& obstacles, const data::ChassisCmd& cmd,
    const config::SafetyConfig& config) const {
  CollisionCheckResult result;
  result.obstacle_too_close =
      ObstacleTooClose(pose, obstacles, static_cast<float>(config.emergency_stop_distance_m),
                       config);

  float local_x = 0.0F;
  float local_y = 0.0F;
  float local_yaw = 0.0F;
  const float lookahead_s = static_cast<float>(config.collision_check_lookahead_s);
  const float dt_s = std::max(0.02F, static_cast<float>(config.collision_check_dt_s));
  for (float t = dt_s; t <= lookahead_s + 1.0e-5F; t += dt_s) {
    local_x += (cmd.vx_mps * std::cos(local_yaw) - cmd.vy_mps * std::sin(local_yaw)) * dt_s;
    local_y += (cmd.vx_mps * std::sin(local_yaw) + cmd.vy_mps * std::cos(local_yaw)) * dt_s;
    local_yaw = NormalizeAngle(local_yaw + cmd.wz_radps * dt_s);
    const auto footprint_samples = BuildFootprintSamples(local_x, local_y, local_yaw, config);
    for (const auto& sample : footprint_samples) {
      if (CostmapCollisionAtLocal(costmap, sample.first, sample.second)) {
        result.type = CollisionType::kStatic;
        return result;
      }
    }
  }

  if (DynamicCollisionAhead(pose, cmd, obstacles, config)) {
    result.type = CollisionType::kDynamic;
  }
  return result;
}

}  // namespace rm_nav::safety
