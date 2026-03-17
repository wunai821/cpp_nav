#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/preint_block.hpp"

namespace rm_nav::sync {

class Deskew {
 public:
  common::Status Apply(const data::LidarFrame& input,
                       const data::PreintegratedImuBlock& preint,
                       data::LidarFrame* output) const;
};

}  // namespace rm_nav::sync
