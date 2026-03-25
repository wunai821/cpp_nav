#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/app/command_frame_transform.hpp"

namespace {

bool Near(float value, float expected, float tolerance = 1.0e-5F) {
  return std::fabs(value - expected) <= tolerance;
}

}  // namespace

int main() {
  rm_nav::data::ChassisCmd cmd;
  cmd.vx_mps = 1.0F;
  cmd.vy_mps = 0.0F;
  cmd.wz_radps = 0.3F;

  auto rotated = rm_nav::app::RotateCommandIntoChildFrame(cmd, 0.0F);
  assert(Near(rotated.vx_mps, 1.0F));
  assert(Near(rotated.vy_mps, 0.0F));
  assert(Near(rotated.wz_radps, 0.3F));

  rotated = rm_nav::app::RotateCommandIntoChildFrame(cmd, 3.14159265358979323846F / 2.0F);
  assert(Near(rotated.vx_mps, 0.0F, 1.0e-4F));
  assert(Near(rotated.vy_mps, -1.0F, 1.0e-4F));
  assert(Near(rotated.wz_radps, 0.3F));

  cmd.vx_mps = 0.5F;
  cmd.vy_mps = 0.5F;
  rotated = rm_nav::app::RotateCommandIntoChildFrame(cmd, 3.14159265358979323846F / 4.0F);
  assert(Near(rotated.vx_mps, 0.70710677F, 1.0e-4F));
  assert(Near(rotated.vy_mps, 0.0F, 1.0e-4F));

  std::cout << "test_command_frame_transform passed\n";
  return 0;
}
