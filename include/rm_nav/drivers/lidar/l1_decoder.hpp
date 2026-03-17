#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/drivers/lidar/l1_packet_parser.hpp"

namespace rm_nav::drivers::lidar {

class L1Decoder {
 public:
  common::Status Decode(const L1Packet& packet, data::LidarFrame* frame) const;
};

}  // namespace rm_nav::drivers::lidar
