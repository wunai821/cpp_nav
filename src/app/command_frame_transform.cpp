#include "rm_nav/app/command_frame_transform.hpp"

#include <cmath>

namespace rm_nav::app {

data::ChassisCmd RotateCommandIntoChildFrame(const data::ChassisCmd& parent_frame_cmd,
                                             float child_yaw_in_parent_rad) {
  data::ChassisCmd child_frame_cmd = parent_frame_cmd;
  const float cos_yaw = std::cos(child_yaw_in_parent_rad);
  const float sin_yaw = std::sin(child_yaw_in_parent_rad);
  child_frame_cmd.vx_mps =
      cos_yaw * parent_frame_cmd.vx_mps + sin_yaw * parent_frame_cmd.vy_mps;
  child_frame_cmd.vy_mps =
      -sin_yaw * parent_frame_cmd.vx_mps + cos_yaw * parent_frame_cmd.vy_mps;
  return child_frame_cmd;
}

}  // namespace rm_nav::app
