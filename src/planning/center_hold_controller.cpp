#include "rm_nav/planning/center_hold_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace rm_nav::planning {
namespace {

constexpr float kPi = 3.14159265358979323846F;

float NormalizeAngle(float angle) {
  while (angle > kPi) {
    angle -= 2.0F * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0F * kPi;
  }
  return angle;
}

float ClampAbs(float value, float limit) {
  if (limit <= 0.0F) {
    return 0.0F;
  }
  return std::max(-limit, std::min(limit, value));
}

bool CostmapOccupied(const data::GridMap2D& costmap, float local_x, float local_y) {
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
  const auto index = static_cast<std::size_t>(gy) * costmap.width + static_cast<std::size_t>(gx);
  return costmap.occupancy[index] >= 50U;
}

float Distance2d(const data::Pose3f& a, const data::Pose3f& b) {
  const float dx = a.position.x - b.position.x;
  const float dy = a.position.y - b.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

void CenterHoldController::Reset() {
  last_slot_index_ = 0;
  last_hold_pose_ = {};
}

std::vector<CenterHoldController::HoldCandidate> CenterHoldController::BuildCandidates(
    const data::Pose3f& current_pose, const data::Pose3f& target_pose,
    const data::GridMap2D& local_costmap,
    const std::vector<data::DynamicObstacle>& obstacles) const {
  std::vector<HoldCandidate> candidates;
  candidates.reserve(5);

  const float slot_radius = static_cast<float>(std::max(0.0, config_.center_hold_slot_radius_m));
  const float occupied_radius =
      static_cast<float>(std::max(0.0, config_.center_hold_occupied_radius_m));
  const std::array<std::pair<float, float>, 5> offsets = {{
      {0.0F, 0.0F},
      {slot_radius, 0.0F},
      {0.0F, slot_radius},
      {-slot_radius, 0.0F},
      {0.0F, -slot_radius},
  }};

  for (std::size_t index = 0; index < offsets.size(); ++index) {
    HoldCandidate candidate;
    candidate.slot_index = static_cast<int>(index);
    candidate.pose = target_pose;
    candidate.pose.position.x += offsets[index].first;
    candidate.pose.position.y += offsets[index].second;
    candidate.pose = ApplyDynamicBias(candidate.pose, obstacles);

    const float dx = candidate.pose.position.x - current_pose.position.x;
    const float dy = candidate.pose.position.y - current_pose.position.y;
    const float cos_yaw = std::cos(current_pose.rpy.z);
    const float sin_yaw = std::sin(current_pose.rpy.z);
    const float local_x = cos_yaw * dx + sin_yaw * dy;
    const float local_y = -sin_yaw * dx + cos_yaw * dy;

    bool occupied = CostmapOccupied(local_costmap, local_x, local_y);
    float nearest_obstacle = 10.0F;
    for (const auto& obstacle : obstacles) {
      if (!obstacle.pose.is_valid) {
        continue;
      }
      const float ox = obstacle.pose.position.x - candidate.pose.position.x;
      const float oy = obstacle.pose.position.y - candidate.pose.position.y;
      const float distance = std::sqrt(ox * ox + oy * oy) - obstacle.radius_m;
      nearest_obstacle = std::min(nearest_obstacle, distance);
      if (distance <= occupied_radius) {
        occupied = true;
      }
    }
    candidate.occupied = occupied;

    const float distance_to_robot = std::sqrt(dx * dx + dy * dy);
    const float switch_penalty =
        last_hold_pose_.is_valid && last_slot_index_ != candidate.slot_index ? 0.15F : 0.0F;
    const float center_preference = candidate.slot_index == 0 ? 0.18F : 0.0F;
    candidate.score = (occupied ? -100.0F : 0.0F) + center_preference +
                      0.25F * std::min(nearest_obstacle, 2.0F) - 0.7F * distance_to_robot -
                      switch_penalty;
    candidates.push_back(candidate);
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const HoldCandidate& lhs, const HoldCandidate& rhs) {
              return lhs.score > rhs.score;
            });
  return candidates;
}

data::Pose3f CenterHoldController::ApplyDynamicBias(
    const data::Pose3f& candidate, const std::vector<data::DynamicObstacle>& obstacles) const {
  data::Pose3f biased = candidate;
  const float bias_radius =
      static_cast<float>(std::max(0.0, config_.center_hold_dynamic_bias_radius_m));
  const float max_bias = static_cast<float>(std::max(0.0, config_.center_hold_max_bias_m));
  if (bias_radius <= 0.0F || max_bias <= 0.0F) {
    return biased;
  }

  float bias_x = 0.0F;
  float bias_y = 0.0F;
  for (const auto& obstacle : obstacles) {
    if (!obstacle.pose.is_valid) {
      continue;
    }
    const float dx = biased.position.x - obstacle.pose.position.x;
    const float dy = biased.position.y - obstacle.pose.position.y;
    const float distance = std::sqrt(dx * dx + dy * dy);
    if (distance <= 1.0e-4F || distance >= bias_radius) {
      continue;
    }
    const float weight =
        (bias_radius - distance) / bias_radius *
        std::clamp(obstacle.risk_score > 0.0F ? obstacle.risk_score : obstacle.confidence, 0.3F,
                   1.5F);
    bias_x += dx / distance * weight;
    bias_y += dy / distance * weight;
  }

  const float bias_norm = std::sqrt(bias_x * bias_x + bias_y * bias_y);
  if (bias_norm > 1.0e-4F) {
    const float applied = std::min(max_bias, bias_norm * 0.18F);
    biased.position.x += bias_x / bias_norm * applied;
    biased.position.y += bias_y / bias_norm * applied;
  }
  return biased;
}

common::Status CenterHoldController::BuildHoldCommand(const data::Pose3f& current_pose,
                                                      const data::Pose3f& target_pose,
                                                      const data::GridMap2D& local_costmap,
                                                      const std::vector<data::DynamicObstacle>& obstacles,
                                                      data::Path2D* path,
                                                      data::ChassisCmd* cmd) {
  if (path == nullptr || cmd == nullptr) {
    return common::Status::InvalidArgument("center hold output is null");
  }

  const auto candidates = BuildCandidates(current_pose, target_pose, local_costmap, obstacles);
  const auto selected = candidates.empty() ? HoldCandidate{} : candidates.front();
  const data::Pose3f hold_pose =
      selected.pose.is_valid ? selected.pose : (target_pose.is_valid ? target_pose : current_pose);

  path->stamp = current_pose.stamp;
  path->points.clear();
  data::PathPoint2f current_point;
  current_point.position.x = current_pose.position.x;
  current_point.position.y = current_pose.position.y;
  current_point.heading_rad = current_pose.rpy.z;
  current_point.target_speed_mps = 0.0F;
  path->points.push_back(current_point);

  data::PathPoint2f hold_point;
  hold_point.position.x = hold_pose.position.x;
  hold_point.position.y = hold_pose.position.y;
  hold_point.heading_rad = hold_pose.rpy.z;
  hold_point.target_speed_mps = 0.0F;
  path->points.push_back(hold_point);

  const float dx = hold_pose.position.x - current_pose.position.x;
  const float dy = hold_pose.position.y - current_pose.position.y;
  const float cos_yaw = std::cos(current_pose.rpy.z);
  const float sin_yaw = std::sin(current_pose.rpy.z);
  const float dx_body = cos_yaw * dx + sin_yaw * dy;
  const float dy_body = -sin_yaw * dx + cos_yaw * dy;
  const float yaw_error = NormalizeAngle(hold_pose.rpy.z - current_pose.rpy.z);

  const float position_deadband =
      static_cast<float>(std::max(0.0, config_.center_hold_position_deadband_m));
  const float yaw_deadband =
      static_cast<float>(std::max(0.0, config_.center_hold_yaw_deadband_rad));
  const float max_hold_v = static_cast<float>(
      std::min(config_.hold_max_v_mps, config_.center_hold_micro_max_v_mps));
  const float max_hold_w = static_cast<float>(
      std::min(config_.hold_max_wz_radps, config_.center_hold_micro_max_wz_radps));
  const float position_kp = static_cast<float>(std::max(0.0, config_.center_hold_position_kp));
  const float yaw_kp = static_cast<float>(std::max(0.0, config_.center_hold_yaw_kp));

  cmd->stamp = current_pose.stamp;
  cmd->vx_mps = std::fabs(dx_body) > position_deadband ? ClampAbs(dx_body * position_kp, max_hold_v)
                                                       : 0.0F;
  cmd->vy_mps = std::fabs(dy_body) > position_deadband ? ClampAbs(dy_body * position_kp, max_hold_v)
                                                       : 0.0F;
  cmd->wz_radps = std::fabs(yaw_error) > yaw_deadband ? ClampAbs(yaw_error * yaw_kp, max_hold_w)
                                                      : 0.0F;
  cmd->brake = std::fabs(cmd->vx_mps) < 1.0e-4F && std::fabs(cmd->vy_mps) < 1.0e-4F &&
               std::fabs(cmd->wz_radps) < 1.0e-4F;

  last_slot_index_ = selected.slot_index;
  last_hold_pose_ = hold_pose;
  return common::Status::Ok();
}

}  // namespace rm_nav::planning
