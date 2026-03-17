#include "rm_nav/drivers/lidar/l1_packet_parser.hpp"

namespace rm_nav::drivers::lidar {

common::Status L1PacketParser::Parse(const std::uint8_t* data, std::size_t size,
                                     L1Packet* packet) const {
  if (data == nullptr || packet == nullptr) {
    return common::Status::InvalidArgument("parser input is null");
  }
  packet->bytes.assign(data, data + size);
  return common::Status::Ok();
}

}  // namespace rm_nav::drivers::lidar
