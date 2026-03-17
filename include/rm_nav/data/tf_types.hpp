#pragma once

#include <string_view>

#include "rm_nav/data/pose.hpp"

namespace rm_nav::tf {

data::Pose3f MakeTransform(std::string_view parent, std::string_view child, float x_m,
                           float y_m, float z_m, float roll_rad, float pitch_rad,
                           float yaw_rad,
                           common::TimePoint stamp = common::TimePoint{});
data::Pose3f MakeTransform(std::string_view parent, std::string_view child, float x_m,
                           float y_m, float yaw_rad,
                           common::TimePoint stamp = common::TimePoint{});
data::Pose3f Inverse(const data::Pose3f& transform);
data::Pose3f Compose(const data::Pose3f& parent_to_mid,
                     const data::Pose3f& mid_to_child);

}  // namespace rm_nav::tf
