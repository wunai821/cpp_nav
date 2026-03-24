#include "rm_nav/app/match_mode_controller.hpp"

#include <algorithm>
#include <cmath>

namespace rm_nav::app {
namespace {

float Distance2D(const data::Pose3f& a, const data::Pose3f& b) {
  const float dx = a.position.x - b.position.x;
  const float dy = a.position.y - b.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

const char* ToString(MatchModePhase phase) {
  switch (phase) {
    case MatchModePhase::kReturnToSpawn:
      return "return_spawn";
    case MatchModePhase::kWaitingAtSpawn:
      return "wait_spawn";
    case MatchModePhase::kReturnToCenter:
      return "return_center";
    case MatchModePhase::kCenter:
    default:
      return "center";
  }
}

common::Status MatchModeController::Configure(const MatchModeConfig& config) {
  if (config.low_hp_threshold < 0 || config.spawn_wait_ms < 0 ||
      config.spawn_reach_tolerance_m <= 0.0F || !config.spawn_pose.is_valid ||
      config.center_reach_tolerance_m <= 0.0F || !config.center_pose.is_valid) {
    return common::Status::InvalidArgument("invalid match mode config");
  }
  config_ = config;
  Reset();
  configured_ = true;
  return common::Status::Ok();
}

void MatchModeController::Reset() {
  latest_decision_ = {};
  latest_decision_.count_as_center_goal = true;
  latest_decision_.goal_pose = config_.spawn_pose;
  wait_deadline_ = {};
}

MatchModeDecision MatchModeController::Update(common::TimePoint stamp,
                                              const data::RefereeState& referee,
                                              const data::Pose3f& current_pose,
                                              const planning::PlannerStatus& planner_status) {
  if (!configured_ || !config_.enabled) {
    Reset();
    return latest_decision_;
  }

  if (latest_decision_.rearm_required && ResourcesRecovered(referee)) {
    latest_decision_.rearm_required = false;
  }

  switch (latest_decision_.phase) {
    case MatchModePhase::kCenter:
      if (!latest_decision_.rearm_required && ResourcesLow(referee)) {
        latest_decision_.phase = MatchModePhase::kReturnToSpawn;
        latest_decision_.reason =
            referee.ammo == 0U ? "ammo_empty" : "hp_low";
      } else {
        latest_decision_.reason = "none";
      }
      break;
    case MatchModePhase::kReturnToSpawn:
      if (NearSpawn(current_pose)) {
        latest_decision_.phase = MatchModePhase::kWaitingAtSpawn;
        latest_decision_.reason = "spawn_wait";
        wait_deadline_ = stamp + std::chrono::milliseconds(config_.spawn_wait_ms);
      }
      break;
    case MatchModePhase::kWaitingAtSpawn:
      latest_decision_.reason = "spawn_wait";
      if (config_.spawn_wait_ms == 0 || (wait_deadline_ != common::TimePoint{} &&
                                         stamp >= wait_deadline_)) {
        latest_decision_.phase = MatchModePhase::kReturnToCenter;
        latest_decision_.reason = "return_center";
        latest_decision_.rearm_required = true;
      }
      break;
    case MatchModePhase::kReturnToCenter:
      latest_decision_.reason = "return_center";
      if (planner_status.reached && !planner_status.temporary_goal_active &&
          NearCenter(current_pose)) {
        latest_decision_.phase = MatchModePhase::kCenter;
        latest_decision_.reason = "none";
      }
      break;
  }

  latest_decision_.override_goal =
      latest_decision_.phase == MatchModePhase::kReturnToSpawn ||
      latest_decision_.phase == MatchModePhase::kWaitingAtSpawn;
  latest_decision_.count_as_center_goal =
      latest_decision_.phase == MatchModePhase::kCenter ||
      latest_decision_.phase == MatchModePhase::kReturnToCenter;
  latest_decision_.goal_pose = config_.spawn_pose;
  latest_decision_.wait_remaining_ns = 0;
  if (latest_decision_.phase == MatchModePhase::kWaitingAtSpawn &&
      wait_deadline_ != common::TimePoint{} && wait_deadline_ > stamp) {
    latest_decision_.wait_remaining_ns = common::ToNanoseconds(wait_deadline_ - stamp);
  }
  return latest_decision_;
}

bool MatchModeController::ResourcesLow(const data::RefereeState& referee) const {
  if (!referee.is_online || referee.game_stage == 0U || referee.remaining_time_s == 0U) {
    return false;
  }
  return referee.ammo == 0U ||
         referee.robot_hp < static_cast<std::uint16_t>(std::max(0, config_.low_hp_threshold));
}

bool MatchModeController::ResourcesRecovered(const data::RefereeState& referee) const {
  if (!referee.is_online) {
    return false;
  }
  return referee.ammo > 0U &&
         referee.robot_hp >= static_cast<std::uint16_t>(std::max(0, config_.low_hp_threshold));
}

bool MatchModeController::NearSpawn(const data::Pose3f& current_pose) const {
  return current_pose.is_valid && Distance2D(current_pose, config_.spawn_pose) <=
                                      config_.spawn_reach_tolerance_m;
}

bool MatchModeController::NearCenter(const data::Pose3f& current_pose) const {
  return current_pose.is_valid && Distance2D(current_pose, config_.center_pose) <=
                                      config_.center_reach_tolerance_m;
}

}  // namespace rm_nav::app
