#include "rm_nav/planning/omni_dwa.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rm_nav::planning {
namespace {

struct DynamicSampleMetrics {
  float risk{0.0F};
  float clearance_min{10.0F};
  float high_risk_penalty{0.0F};
  float crossing_penalty{0.0F};
  int max_risk_level{0};
};

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

float DesiredHeading(const GoalState& goal, float x, float y) {
  if (goal.mode == GoalMode::kCenterHold) {
    return goal.target_pose.rpy.z;
  }
  return std::atan2(goal.target_pose.position.y - y, goal.target_pose.position.x - x);
}

float DistanceToPath(const data::Path2D& path, float x, float y) {
  float best = std::numeric_limits<float>::max();
  const std::size_t begin_index = path.points.size() > 1U ? 1U : 0U;
  for (std::size_t index = begin_index; index < path.points.size(); ++index) {
    const auto& point = path.points[index];
    const float dx = point.position.x - x;
    const float dy = point.position.y - y;
    best = std::min(best, std::sqrt(dx * dx + dy * dy));
  }
  return best == std::numeric_limits<float>::max() ? 0.0F : best;
}

bool InCollision(const data::GridMap2D& costmap, float local_x, float local_y) {
  if (costmap.width == 0U || costmap.height == 0U || costmap.occupancy.empty()) {
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

float DistanceToObstacleSet(const std::vector<data::DynamicObstacle>& obstacles, float world_x,
                            float world_y) {
  float best = std::numeric_limits<float>::max();
  for (const auto& obstacle : obstacles) {
    const float dx = obstacle.pose.position.x - world_x;
    const float dy = obstacle.pose.position.y - world_y;
    best = std::min(best, std::sqrt(dx * dx + dy * dy) -
                              std::max(obstacle.radius_m, obstacle.predicted_radius_m));
  }
  return best == std::numeric_limits<float>::max() ? 10.0F : best;
}

data::Pose3f PredictObstaclePose(const data::DynamicObstacle& obstacle, float t_s) {
  if (t_s <= 0.0F) {
    return obstacle.pose;
  }
  if (t_s <= 0.5F) {
    const float alpha = t_s / 0.5F;
    data::Pose3f pose = obstacle.pose;
    pose.position.x += alpha * (obstacle.predicted_pose_05s.position.x - obstacle.pose.position.x);
    pose.position.y += alpha * (obstacle.predicted_pose_05s.position.y - obstacle.pose.position.y);
    return pose;
  }
  if (t_s <= 1.0F) {
    const float alpha = (t_s - 0.5F) / 0.5F;
    data::Pose3f pose = obstacle.predicted_pose_05s;
    pose.position.x +=
        alpha * (obstacle.predicted_pose_10s.position.x - obstacle.predicted_pose_05s.position.x);
    pose.position.y +=
        alpha * (obstacle.predicted_pose_10s.position.y - obstacle.predicted_pose_05s.position.y);
    return pose;
  }

  data::Pose3f pose = obstacle.predicted_pose_10s;
  const float extra_t = t_s - 1.0F;
  pose.position.x += obstacle.velocity.x * extra_t;
  pose.position.y += obstacle.velocity.y * extra_t;
  return pose;
}

int RiskLevelValue(data::DynamicObstacleRiskLevel risk_level) {
  switch (risk_level) {
    case data::DynamicObstacleRiskLevel::kHigh:
      return 2;
    case data::DynamicObstacleRiskLevel::kMedium:
      return 1;
    default:
      return 0;
  }
}

float RiskLevelWeight(data::DynamicObstacleRiskLevel risk_level) {
  switch (risk_level) {
    case data::DynamicObstacleRiskLevel::kHigh:
      return 1.45F;
    case data::DynamicObstacleRiskLevel::kMedium:
      return 1.0F;
    default:
      return 0.7F;
  }
}

float PredictionRadiusAtTime(const data::DynamicObstacle& obstacle, float t_s) {
  const float predicted_radius =
      obstacle.predicted_radius_m > 0.0F ? obstacle.predicted_radius_m : obstacle.radius_m;
  const float alpha = std::clamp(t_s, 0.0F, 1.0F);
  return obstacle.radius_m + alpha * (predicted_radius - obstacle.radius_m);
}

DynamicSampleMetrics EvaluateDynamicSample(const std::vector<data::DynamicObstacle>& obstacles,
                                           float world_x, float world_y, float robot_world_vx,
                                           float robot_world_vy, float t_s,
                                           const config::PlannerConfig& config) {
  DynamicSampleMetrics metrics;
  if (obstacles.empty()) {
    return metrics;
  }

  const float influence_distance =
      std::max(static_cast<float>(config.dynamic_influence_distance_m), 0.2F);
  const float emergency_distance =
      std::clamp(static_cast<float>(config.dynamic_emergency_distance_m), 0.0F,
                 influence_distance - 0.05F);
  const float inflation_m = std::max(static_cast<float>(config.dynamic_inflation_m), 0.0F);

  float total_risk = 0.0F;
  for (const auto& obstacle : obstacles) {
    const auto predicted_pose = PredictObstaclePose(obstacle, t_s);
    const float prediction_radius = PredictionRadiusAtTime(obstacle, t_s);
    const float dx = predicted_pose.position.x - world_x;
    const float dy = predicted_pose.position.y - world_y;
    const float distance = std::sqrt(dx * dx + dy * dy);
    const float clearance = distance - (prediction_radius + inflation_m);
    metrics.clearance_min = std::min(metrics.clearance_min, clearance);
    if (clearance < influence_distance) {
      metrics.max_risk_level = std::max(metrics.max_risk_level, RiskLevelValue(obstacle.risk_level));
    }
    if (clearance <= 0.0F) {
      metrics.risk = 10.0F;
      metrics.high_risk_penalty = 10.0F;
      metrics.crossing_penalty = 10.0F;
      metrics.max_risk_level = std::max(metrics.max_risk_level, RiskLevelValue(obstacle.risk_level));
      return metrics;
    }
    if (clearance >= influence_distance) {
      continue;
    }

    const float risk_weight = RiskLevelWeight(obstacle.risk_level) *
                              std::clamp(obstacle.risk_score > 0.0F ? 0.5F + obstacle.risk_score
                                                                     : 0.5F + obstacle.confidence,
                                         0.35F, 1.6F);
    const float confidence =
        std::clamp(obstacle.is_confirmed ? std::max(obstacle.confidence, 0.6F)
                                         : std::max(obstacle.confidence, 0.25F),
                   0.0F, 1.0F);
    const float unit_x = distance > 1.0e-4F ? dx / distance : 0.0F;
    const float unit_y = distance > 1.0e-4F ? dy / distance : 0.0F;
    const float rel_vx = obstacle.velocity.x - robot_world_vx;
    const float rel_vy = obstacle.velocity.y - robot_world_vy;
    const float closing_speed = std::max(0.0F, -(rel_vx * unit_x + rel_vy * unit_y));

    float risk = 0.0F;
    if (clearance <= emergency_distance) {
      const float denom = std::max(emergency_distance, 0.05F);
      risk = 2.0F + (emergency_distance - clearance) / denom;
    } else {
      const float denom = std::max(influence_distance - emergency_distance, 0.05F);
      const float ratio = (influence_distance - clearance) / denom;
      risk = ratio * ratio;
    }
    risk *= confidence * risk_weight * (1.0F + 0.5F * closing_speed);
    total_risk += risk;

    if (obstacle.risk_level == data::DynamicObstacleRiskLevel::kHigh) {
      metrics.high_risk_penalty += risk_weight *
                                   ((influence_distance - clearance) /
                                    std::max(influence_distance, 0.05F));
    }

    const float robot_speed = std::sqrt(robot_world_vx * robot_world_vx + robot_world_vy * robot_world_vy);
    const float obstacle_speed =
        std::sqrt(obstacle.velocity.x * obstacle.velocity.x + obstacle.velocity.y * obstacle.velocity.y);
    if (robot_speed > 0.05F && obstacle_speed > 0.05F) {
      const float robot_dir_x = robot_world_vx / robot_speed;
      const float robot_dir_y = robot_world_vy / robot_speed;
      const float obstacle_dir_x = obstacle.velocity.x / obstacle_speed;
      const float obstacle_dir_y = obstacle.velocity.y / obstacle_speed;
      const float cross_mag = std::fabs(robot_dir_x * obstacle_dir_y - robot_dir_y * obstacle_dir_x);
      const float closing_ratio =
          std::clamp(closing_speed / std::max(robot_speed + obstacle_speed, 0.1F), 0.0F, 1.0F);
      const float crossing_window = std::max(emergency_distance * 1.5F, 0.3F);
      const float emergency_ratio =
          std::clamp((crossing_window - clearance) / crossing_window, 0.0F, 1.0F);
      metrics.crossing_penalty +=
          risk_weight * cross_mag * (0.5F + closing_ratio) * emergency_ratio;
    }
  }
  metrics.risk = std::min(total_risk, 10.0F);
  if (metrics.clearance_min == std::numeric_limits<float>::max()) {
    metrics.clearance_min = 10.0F;
  }
  return metrics;
}

float SampleValue(int sample_index, int total_samples, float max_abs) {
  if (total_samples <= 1) {
    return 0.0F;
  }
  const float alpha =
      static_cast<float>(sample_index) / static_cast<float>(total_samples - 1);
  return -max_abs + 2.0F * max_abs * alpha;
}

}  // namespace

common::Status OmniDwa::Plan(const data::Pose3f& current_pose, const GoalState& goal,
                             const data::Path2D& global_path,
                             const data::GridMap2D& local_costmap,
                             const std::vector<data::DynamicObstacle>& obstacles,
                             const data::ChassisCmd& previous_cmd, data::ChassisCmd* cmd,
                             DwaScore* score, float clearance_weight_scale) const {
  if (cmd == nullptr || score == nullptr) {
    return common::Status::InvalidArgument("dwa output is null");
  }
  const float effective_clearance_weight_scale =
      std::max(0.1F, clearance_weight_scale);

  const float max_vx = static_cast<float>(
      goal.mode == GoalMode::kCenterHold ? config_.hold_max_v_mps : config_.max_vx_mps);
  const float max_vy = static_cast<float>(
      goal.mode == GoalMode::kCenterHold ? config_.hold_max_v_mps : config_.max_vy_mps);
  const float max_wz = static_cast<float>(
      goal.mode == GoalMode::kCenterHold ? config_.hold_max_wz_radps : config_.max_wz_radps);

  data::ChassisCmd best_cmd;
  best_cmd.brake = true;
  DwaScore best_score;

  for (int ix = 0; ix < config_.dwa_linear_samples; ++ix) {
    const float vx = SampleValue(ix, config_.dwa_linear_samples, max_vx);
    for (int iy = 0; iy < config_.dwa_lateral_samples; ++iy) {
      const float vy = SampleValue(iy, config_.dwa_lateral_samples, max_vy);
      for (int iw = 0; iw < config_.dwa_yaw_samples; ++iw) {
        const float wz = SampleValue(iw, config_.dwa_yaw_samples, max_wz);

        float local_x = 0.0F;
        float local_y = 0.0F;
        float local_yaw = current_pose.rpy.z;
        bool collision = false;
        float integrated_dynamic_risk = 0.0F;
        float max_dynamic_risk = 0.0F;
        float integrated_high_risk_penalty = 0.0F;
        float integrated_crossing_penalty = 0.0F;
        float min_dynamic_clearance = std::numeric_limits<float>::max();
        int max_dynamic_risk_level = 0;
        for (double t = config_.dwa_dt_s; t <= config_.dwa_horizon_s + 1.0e-6;
             t += config_.dwa_dt_s) {
          local_x += (vx * std::cos(local_yaw) - vy * std::sin(local_yaw)) *
                     static_cast<float>(config_.dwa_dt_s);
          local_y += (vx * std::sin(local_yaw) + vy * std::cos(local_yaw)) *
                     static_cast<float>(config_.dwa_dt_s);
          local_yaw = NormalizeAngle(local_yaw + wz * static_cast<float>(config_.dwa_dt_s));
          if (InCollision(local_costmap, local_x, local_y)) {
            collision = true;
            break;
          }
          const float future_world_x = current_pose.position.x + local_x;
          const float future_world_y = current_pose.position.y + local_y;
          const float robot_world_vx = vx * std::cos(local_yaw) - vy * std::sin(local_yaw);
          const float robot_world_vy = vx * std::sin(local_yaw) + vy * std::cos(local_yaw);
          const auto sample_metrics = EvaluateDynamicSample(
              obstacles, future_world_x, future_world_y, robot_world_vx, robot_world_vy,
              static_cast<float>(t), config_);
          integrated_dynamic_risk += sample_metrics.risk * static_cast<float>(config_.dwa_dt_s);
          integrated_high_risk_penalty +=
              sample_metrics.high_risk_penalty * static_cast<float>(config_.dwa_dt_s);
          integrated_crossing_penalty +=
              sample_metrics.crossing_penalty * static_cast<float>(config_.dwa_dt_s);
          max_dynamic_risk = std::max(max_dynamic_risk, sample_metrics.risk);
          min_dynamic_clearance = std::min(min_dynamic_clearance, sample_metrics.clearance_min);
          max_dynamic_risk_level =
              std::max(max_dynamic_risk_level, sample_metrics.max_risk_level);
          if (sample_metrics.risk >= 10.0F || sample_metrics.clearance_min <= 0.0F) {
            collision = true;
            break;
          }
        }
        if (collision) {
          continue;
        }

        const float end_x = current_pose.position.x + local_x;
        const float end_y = current_pose.position.y + local_y;
        const float dx_goal = goal.target_pose.position.x - end_x;
        const float dy_goal = goal.target_pose.position.y - end_y;
        const float dist_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
        const float progress = std::max(0.0F, goal.distance_to_target_m - dist_to_goal);
        const float obstacle_clearance =
            DistanceToObstacleSet(obstacles, end_x, end_y);
        const float path_dist = DistanceToPath(global_path, end_x, end_y);
        const float smooth_delta = std::fabs(vx - previous_cmd.vx_mps) +
                                   std::fabs(vy - previous_cmd.vy_mps) +
                                   0.5F * std::fabs(wz - previous_cmd.wz_radps);
        const float linear_speed = std::sqrt(vx * vx + vy * vy);
        const float heading_error = std::fabs(
            NormalizeAngle(DesiredHeading(goal, end_x, end_y) - local_yaw));
        const auto sample_05 = EvaluateDynamicSample(
            obstacles, end_x, end_y, vx * std::cos(local_yaw) - vy * std::sin(local_yaw),
            vx * std::sin(local_yaw) + vy * std::cos(local_yaw), 0.5F, config_);
        const auto sample_10 = EvaluateDynamicSample(
            obstacles, end_x, end_y, vx * std::cos(local_yaw) - vy * std::sin(local_yaw),
            vx * std::sin(local_yaw) + vy * std::cos(local_yaw), 1.0F, config_);
        const float dynamic_penalty =
            0.35F * max_dynamic_risk + 0.20F * integrated_dynamic_risk +
            0.15F * sample_05.risk + 0.10F * sample_10.risk +
            static_cast<float>(config_.dynamic_high_risk_penalty_scale) *
                (0.15F * integrated_high_risk_penalty + 0.10F * sample_05.high_risk_penalty +
                 0.05F * sample_10.high_risk_penalty) +
            static_cast<float>(config_.dynamic_crossing_penalty_scale) *
                (0.20F * integrated_crossing_penalty + 0.15F * sample_05.crossing_penalty +
                 0.10F * sample_10.crossing_penalty);

        DwaScore candidate_score;
        candidate_score.goal_score = 10.0F * progress;
        candidate_score.path_score = 0.8F / (0.2F + path_dist);
        candidate_score.smooth_score = 0.4F / (0.2F + smooth_delta);
        candidate_score.heading_score = static_cast<float>(config_.weight_heading) *
                                        (0.8F / (0.2F + heading_error));
        candidate_score.clearance_score =
            static_cast<float>(config_.weight_clearance) * effective_clearance_weight_scale *
            std::min(1.0F, obstacle_clearance);
        candidate_score.velocity_score = static_cast<float>(config_.weight_velocity) *
                                         linear_speed;
        candidate_score.dynamic_risk_score =
            -static_cast<float>(config_.weight_dynamic) * dynamic_penalty;
        candidate_score.dynamic_max_risk = max_dynamic_risk;
        candidate_score.dynamic_integrated_risk = integrated_dynamic_risk;
        candidate_score.dynamic_clearance_min =
            min_dynamic_clearance == std::numeric_limits<float>::max() ? 10.0F
                                                                       : min_dynamic_clearance;
        candidate_score.dynamic_nearest_predicted_distance =
            candidate_score.dynamic_clearance_min;
        candidate_score.dynamic_risk_05 = sample_05.risk;
        candidate_score.dynamic_risk_10 = sample_10.risk;
        candidate_score.dynamic_high_risk_penalty =
            integrated_high_risk_penalty + sample_05.high_risk_penalty + sample_10.high_risk_penalty;
        candidate_score.dynamic_crossing_penalty =
            integrated_crossing_penalty + sample_05.crossing_penalty + sample_10.crossing_penalty;
        candidate_score.dynamic_max_risk_level = std::max(
            max_dynamic_risk_level, std::max(sample_05.max_risk_level, sample_10.max_risk_level));
        candidate_score.total_score = candidate_score.goal_score +
                                      candidate_score.path_score +
                                      candidate_score.smooth_score +
                                      candidate_score.heading_score +
                                      candidate_score.clearance_score +
                                      candidate_score.velocity_score +
                                      candidate_score.dynamic_risk_score +
                                      (goal.mode == GoalMode::kCenterHold ? 0.3F : 0.0F);
        if (!goal.reached && goal.distance_to_target_m > 0.5F && linear_speed < 0.05F) {
          candidate_score.total_score -= 6.0F;
        }

        if (candidate_score.total_score > best_score.total_score) {
          best_score = candidate_score;
          best_cmd.stamp = current_pose.stamp;
          best_cmd.vx_mps = vx;
          best_cmd.vy_mps = vy;
          best_cmd.wz_radps = wz;
          best_cmd.brake = false;
        }
      }
    }
  }

  if (goal.reached) {
    best_cmd = {};
    best_cmd.stamp = current_pose.stamp;
    best_cmd.brake = true;
    best_score.total_score = 0.0F;
  }

  *cmd = best_cmd;
  *score = best_score;
  return common::Status::Ok();
}

}  // namespace rm_nav::planning
