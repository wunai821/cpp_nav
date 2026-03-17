#include "rm_nav/drivers/lidar/l1_decoder.hpp"

namespace rm_nav::drivers::lidar {

common::Status L1Decoder::Decode(const L1Packet& packet, data::LidarFrame* frame) const {
  if (frame == nullptr) {
    return common::Status::InvalidArgument("frame output is null");
  }
  if (packet.bytes.empty()) {
    return common::Status::NotReady("empty lidar packet");
  }
  return common::Status::Unimplemented("hardware L1 decode is not implemented yet");
}

}  // namespace rm_nav::drivers::lidar
