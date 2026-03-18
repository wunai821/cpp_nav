#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/safety/command_gate.hpp"

namespace {

bool Near(float value, float expected, float tolerance = 1.0e-5F) {
  return std::fabs(value - expected) <= tolerance;
}

rm_nav::data::ChassisCmd MakeCmd(float vx, float vy, float wz) {
  rm_nav::data::ChassisCmd cmd;
  cmd.vx_mps = vx;
  cmd.vy_mps = vy;
  cmd.wz_radps = wz;
  cmd.brake = false;
  return cmd;
}

}  // namespace

int main() {
  rm_nav::safety::CommandGate gate;
  rm_nav::config::SafetyConfig config;
  config.max_vx_mps = 0.5;
  config.max_vy_mps = 0.4;
  config.max_wz_radps = 0.6;
  config.max_delta_v_per_tick = 0.1;
  config.max_delta_w_per_tick = 0.2;
  assert(gate.Configure(config).ok());

  const auto base_time = rm_nav::common::FromNanoseconds(1000000000LL);

  auto result = gate.Gate(MakeCmd(1.2F, -0.9F, 1.5F), base_time, true,
                          rm_nav::safety::CollisionType::kNone);
  assert(!result.blocked);
  assert(!result.command.brake);
  assert(Near(result.command.vx_mps, 0.1F));
  assert(Near(result.command.vy_mps, -0.1F));
  assert(Near(result.command.wz_radps, 0.2F));

  result = gate.Gate(MakeCmd(1.2F, -0.9F, 1.5F), base_time + std::chrono::milliseconds(20), true,
                     rm_nav::safety::CollisionType::kNone);
  assert(Near(result.command.vx_mps, 0.2F));
  assert(Near(result.command.vy_mps, -0.2F));
  assert(Near(result.command.wz_radps, 0.4F));

  for (int step = 0; step < 6; ++step) {
    result = gate.Gate(MakeCmd(1.2F, -0.9F, 1.5F),
                       base_time + std::chrono::milliseconds(40 + 20 * step), true,
                       rm_nav::safety::CollisionType::kNone);
  }
  assert(Near(result.command.vx_mps, 0.5F));
  assert(Near(result.command.vy_mps, -0.4F));
  assert(Near(result.command.wz_radps, 0.6F));

  result = gate.Gate(MakeCmd(0.4F, 0.2F, 0.1F), base_time + std::chrono::milliseconds(200), false,
                     rm_nav::safety::CollisionType::kNone);
  assert(result.blocked);
  assert(result.command.brake);
  assert(result.reason == rm_nav::safety::CommandGateReason::kStateBlocked);

  result = gate.Gate(MakeCmd(-0.4F, 0.3F, -0.5F), base_time + std::chrono::milliseconds(220), true,
                     rm_nav::safety::CollisionType::kNone);
  assert(!result.command.brake);
  assert(Near(result.command.vx_mps, -0.1F));
  assert(Near(result.command.vy_mps, 0.1F));
  assert(Near(result.command.wz_radps, -0.2F));

  rm_nav::data::ChassisCmd brake_cmd;
  brake_cmd.brake = true;
  result = gate.Gate(brake_cmd, base_time + std::chrono::milliseconds(240), true,
                     rm_nav::safety::CollisionType::kNone);
  assert(result.blocked);
  assert(result.command.brake);
  assert(result.reason == rm_nav::safety::CommandGateReason::kBrakeRequested);

  result = gate.Gate(MakeCmd(0.5F, 0.0F, 0.0F), base_time + std::chrono::milliseconds(260), true,
                     rm_nav::safety::CollisionType::kStatic);
  assert(result.blocked);
  assert(result.command.brake);
  assert(result.reason == rm_nav::safety::CommandGateReason::kStaticCollision);

  std::cout << "test_command_gate passed\n";
  return 0;
}
