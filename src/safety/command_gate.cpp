#include "rm_nav/safety/command_gate.hpp"

#include <algorithm>
#include <cmath>

namespace rm_nav::safety {
namespace {

float ClampAbs(float value, float limit) {
  if (limit <= 0.0F) {
    return 0.0F;
  }
  return std::max(-limit, std::min(limit, value));
}

float ClampDelta(float target, float previous, float delta_limit) {
  if (delta_limit <= 0.0F) {
    return previous;
  }
  const float delta = target - previous;
  if (delta > delta_limit) {
    return previous + delta_limit;
  }
  if (delta < -delta_limit) {
    return previous - delta_limit;
  }
  return target;
}

}  // namespace

common::Status CommandGate::Configure(const config::SafetyConfig& config) {
  config_ = config;
  Reset();
  configured_ = true;
  return common::Status::Ok();
}

void CommandGate::Reset() {
  last_command_ = {};
  last_command_.brake = true;
}

CommandGateResult CommandGate::Gate(const data::ChassisCmd& proposed_cmd, common::TimePoint stamp,
                                    bool motion_allowed,
                                    CollisionType collision_type) {
  CommandGateResult result;
  result.command = proposed_cmd;
  result.command.stamp = stamp;

  if (!configured_) {
    result.command = {};
    result.command.stamp = stamp;
    result.command.brake = true;
    result.blocked = true;
    result.reason = CommandGateReason::kStateBlocked;
    return result;
  }

  if (!motion_allowed) {
    result.command = {};
    result.command.stamp = stamp;
    result.command.brake = true;
    result.blocked = true;
    result.reason = CommandGateReason::kStateBlocked;
    last_command_ = result.command;
    return result;
  }

  if (proposed_cmd.brake) {
    result.command = proposed_cmd;
    result.command.stamp = stamp;
    result.command.brake = true;
    result.blocked = true;
    result.reason = CommandGateReason::kBrakeRequested;
    last_command_ = result.command;
    return result;
  }

  if (collision_type == CollisionType::kStatic || collision_type == CollisionType::kDynamic) {
    result.command = {};
    result.command.stamp = stamp;
    result.command.brake = true;
    result.blocked = true;
    result.reason = collision_type == CollisionType::kStatic
                        ? CommandGateReason::kStaticCollision
                        : CommandGateReason::kDynamicCollision;
    last_command_ = result.command;
    return result;
  }

  const float max_vx = static_cast<float>(std::max(0.0, config_.max_vx_mps));
  const float max_vy = static_cast<float>(std::max(0.0, config_.max_vy_mps));
  const float max_wz = static_cast<float>(std::max(0.0, config_.max_wz_radps));
  const float delta_v = static_cast<float>(std::max(0.0, config_.max_delta_v_per_tick));
  const float delta_w = static_cast<float>(std::max(0.0, config_.max_delta_w_per_tick));

  const float clamped_vx = ClampAbs(proposed_cmd.vx_mps, max_vx);
  const float clamped_vy = ClampAbs(proposed_cmd.vy_mps, max_vy);
  const float clamped_wz = ClampAbs(proposed_cmd.wz_radps, max_wz);

  result.command.vx_mps = ClampAbs(ClampDelta(clamped_vx, last_command_.vx_mps, delta_v), max_vx);
  result.command.vy_mps = ClampAbs(ClampDelta(clamped_vy, last_command_.vy_mps, delta_v), max_vy);
  result.command.wz_radps =
      ClampAbs(ClampDelta(clamped_wz, last_command_.wz_radps, delta_w), max_wz);
  result.command.brake = false;
  last_command_ = result.command;
  return result;
}

}  // namespace rm_nav::safety
