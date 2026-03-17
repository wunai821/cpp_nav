#pragma once

#include <deque>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/common/time.hpp"
#include "rm_nav/data/imu_packet.hpp"
#include "rm_nav/data/lidar_frame.hpp"

namespace unitree_lidar_sdk {
class UnitreeLidarReader;
}

namespace rm_nav::drivers::lidar {

struct L1DriverConfig {
  std::string source{"synthetic"};
  std::string device_path{"/dev/ttyACM0"};
  int baud_rate{2000000};
  int cloud_scan_num{18};
  std::string synthetic_map_pcd_path{};
  double frame_rate_hz{10.0};
  std::size_t points_per_frame{360};
  float radius_m{5.0F};
};

class L1Driver {
 public:
  common::Status Configure(const L1DriverConfig& config);
  std::optional<data::LidarFrame> PollFrame();
  std::optional<data::ImuPacket> PollImuPacket();

 private:
  std::optional<data::LidarFrame> PollSyntheticFrame();
  std::optional<data::LidarFrame> PollUnitreeSdkFrame();
  std::optional<data::ImuPacket> PollUnitreeSdkImuPacket();
  bool PumpUnitreeSdkOnce();

  std::vector<data::PointXYZI> synthetic_map_points_{};
  unitree_lidar_sdk::UnitreeLidarReader* unitree_reader_{nullptr};
  std::optional<data::LidarFrame> pending_sdk_frame_{};
  std::deque<data::ImuPacket> pending_sdk_imu_packets_{};
  L1DriverConfig config_{};
  common::TimePoint next_frame_time_{};
  common::TimeConverter driver_time_converter_{};
  common::TimePoint last_aux_log_time_{};
  std::uint32_t imu_sample_index_{0};
  std::uint32_t frame_index_{0};
  bool configured_{false};
  bool sdk_version_logged_{false};
  bool sdk_warmup_done_{false};
};

}  // namespace rm_nav::drivers::lidar
