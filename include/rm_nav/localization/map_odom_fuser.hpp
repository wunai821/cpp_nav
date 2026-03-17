#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::localization {

class MapOdomFuser {
 public:
  common::Status Fuse(const data::Pose3f& odom_to_base,
                      const data::Pose3f& map_to_base,
                      data::Pose3f* map_to_odom) const;
};

}  // namespace rm_nav::localization
