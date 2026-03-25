#pragma once

#include "rm_nav/data/chassis_cmd.hpp"

namespace rm_nav::app {

data::ChassisCmd RotateCommandIntoChildFrame(const data::ChassisCmd& parent_frame_cmd,
                                             float child_yaw_in_parent_rad);

}  // namespace rm_nav::app
