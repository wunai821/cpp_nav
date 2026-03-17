#pragma once

#include <optional>
#include <string_view>

#include "rm_nav/common/status.hpp"
#include "rm_nav/tf/static_tf_registry.hpp"
#include "rm_nav/tf/timed_pose_buffer.hpp"

namespace rm_nav::tf {

class TfTreeLite {
 public:
  common::Status RegisterStaticTransform(const data::Pose3f& transform);
  common::Status PushMapToOdom(const data::Pose3f& transform);
  common::Status PushOdomToBase(const data::Pose3f& transform);

  std::optional<data::Pose3f> Lookup(std::string_view parent, std::string_view child,
                                     common::TimePoint stamp) const;

 private:
  StaticTfRegistry static_registry_{};
  TimedPoseBuffer map_to_odom_buffer_{};
  TimedPoseBuffer odom_to_base_buffer_{};
};

}  // namespace rm_nav::tf
