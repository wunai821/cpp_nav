#include "rm_nav/drivers/imu/imu_decoder.hpp"

namespace rm_nav::drivers::imu {

common::Status ImuDecoder::Decode(const std::uint8_t* data, std::size_t size,
                                  data::ImuPacket* packet) const {
  if (data == nullptr || packet == nullptr || size == 0U) {
    return common::Status::InvalidArgument("imu decode input is invalid");
  }
  return common::Status::Unimplemented("hardware IMU decode is not implemented yet");
}

}  // namespace rm_nav::drivers::imu
