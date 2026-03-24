#include "rm_nav/drivers/lidar/l1_driver.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <sstream>

#include <unistd.h>

#include "rm_nav/common/time.hpp"
#include "rm_nav/tf/frame_ids.hpp"
#include "rm_nav/utils/logger.hpp"

#ifdef RM_NAV_HAS_UNITREE_LIDAR_SDK
#include "unitree_lidar_sdk.h"
#endif

namespace rm_nav::drivers::lidar {
namespace {

constexpr float kPi = 3.14159265358979323846F;

struct SyntheticPose2d {
  float x{0.0F};
  float y{0.0F};
  float yaw{0.0F};
};

SyntheticPose2d SyntheticTruePose(std::uint32_t frame_index) {
  const float index = static_cast<float>(frame_index);
  SyntheticPose2d pose;
  pose.x = 1.0F + 0.08F * index;
  pose.y = 2.0F + 0.45F * std::sin(index * 0.06F);
  pose.yaw = 0.10F * std::sin(index * 0.05F);
  return pose;
}

bool LoadSyntheticMap(const std::string& path, std::vector<data::PointXYZI>* points) {
  if (points == nullptr || path.empty()) {
    return false;
  }
  std::ifstream input(path);
  if (!input.is_open()) {
    return false;
  }
  points->clear();
  std::string line;
  bool data_section = false;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    if (!data_section) {
      if (line.rfind("DATA", 0) == 0) {
        data_section = true;
      }
      continue;
    }
    std::istringstream stream(line);
    data::PointXYZI point;
    if (!(stream >> point.x >> point.y >> point.z)) {
      continue;
    }
    if (!(stream >> point.intensity)) {
      point.intensity = 1.0F;
    }
    points->push_back(point);
  }
  return !points->empty();
}

data::PointXYZI ToLocalPoint(const data::PointXYZI& world_point, const SyntheticPose2d& pose) {
  const float dx = world_point.x - pose.x;
  const float dy = world_point.y - pose.y;
  const float cos_yaw = std::cos(pose.yaw);
  const float sin_yaw = std::sin(pose.yaw);
  data::PointXYZI local = world_point;
  local.x = cos_yaw * dx + sin_yaw * dy;
  local.y = -sin_yaw * dx + cos_yaw * dy;
  local.z = world_point.z;
  if (std::fabs(local.z) < 0.05F) {
    local.z = 0.6F;
  }
  return local;
}

#ifdef RM_NAV_HAS_UNITREE_LIDAR_SDK
common::TimePoint ToSystemTimePoint(double driver_stamp_s, common::TimeConverter* converter) {
  if (converter == nullptr) {
    return common::Now();
  }
  const auto driver_time_ns =
      static_cast<common::TimeNs>(std::llround(driver_stamp_s * 1.0e9));
  if (!converter->is_calibrated()) {
    converter->Calibrate(driver_time_ns, common::Now());
  }
  return converter->DriverToSystem(driver_time_ns);
}

data::LidarFrame BuildFrameFromUnitreeCloud(
    const unitree_lidar_sdk::PointCloudUnitree& cloud, std::uint32_t frame_index,
    common::TimeConverter* converter) {
  data::LidarFrame frame;
  frame.frame_id = tf::kLaserFrame;
  frame.frame_index = frame_index;
  frame.points.reserve(cloud.points.size());

  float max_relative_time_s = 0.0F;
  for (const auto& unitree_point : cloud.points) {
    data::PointXYZI point;
    point.x = unitree_point.x;
    point.y = unitree_point.y;
    point.z = unitree_point.z;
    point.intensity = unitree_point.intensity;
    point.relative_time_s = unitree_point.time;
    max_relative_time_s = std::max(max_relative_time_s, point.relative_time_s);
    frame.points.push_back(point);
  }

  const auto end_stamp = ToSystemTimePoint(cloud.stamp, converter);
  frame.stamp = end_stamp;
  frame.scan_end_stamp = end_stamp;
  frame.scan_begin_stamp =
      end_stamp - std::chrono::duration_cast<common::Duration>(
                      std::chrono::duration<double>(max_relative_time_s));
  return frame;
}

data::ImuPacket BuildPacketFromUnitreeImu(const unitree_lidar_sdk::IMUUnitree& imu,
                                          std::uint32_t sample_index,
                                          common::TimeConverter* converter) {
  data::ImuPacket packet;
  packet.stamp = ToSystemTimePoint(imu.stamp, converter);
  packet.sample_index = sample_index;
  packet.frame_id = tf::kImuFrame;
  packet.angular_velocity.x = imu.angular_velocity[0];
  packet.angular_velocity.y = imu.angular_velocity[1];
  packet.angular_velocity.z = imu.angular_velocity[2];
  packet.linear_acceleration.x = imu.linear_acceleration[0];
  packet.linear_acceleration.y = imu.linear_acceleration[1];
  packet.linear_acceleration.z = imu.linear_acceleration[2];
  packet.is_valid = true;
  return packet;
}
#endif

}  // namespace

common::Status L1Driver::Configure(const L1DriverConfig& config) {
  config_ = config;
  frame_index_ = 0;
  configured_ = false;
  sdk_version_logged_ = false;
  sdk_warmup_done_ = false;
  synthetic_map_points_.clear();
  pending_sdk_frame_.reset();
  pending_sdk_imu_packets_.clear();
  driver_time_converter_.Reset();
  last_aux_log_time_ = common::TimePoint{};
  imu_sample_index_ = 0;

  if (config.source == "synthetic") {
    if (config.frame_rate_hz <= 0.0 || config.points_per_frame == 0U) {
      return common::Status::InvalidArgument("invalid synthetic L1 config");
    }
    const auto period = std::chrono::duration_cast<common::Duration>(
        std::chrono::duration<double>(1.0 / config.frame_rate_hz));
    next_frame_time_ = common::Now() + period;
    if (!config.synthetic_map_pcd_path.empty()) {
      LoadSyntheticMap(config.synthetic_map_pcd_path, &synthetic_map_points_);
    }
    configured_ = true;
    return common::Status::Ok();
  }

  if (config.source == "unitree_sdk") {
#ifndef RM_NAV_HAS_UNITREE_LIDAR_SDK
    return common::Status::Unimplemented("unitree_lidar_sdk is not linked into this build");
#else
    if (config.device_path.empty()) {
      return common::Status::InvalidArgument("unitree_sdk source requires a lidar device path");
    }
    if (config.cloud_scan_num <= 0) {
      return common::Status::InvalidArgument("unitree_sdk cloud_scan_num must be positive");
    }
    if (unitree_reader_ == nullptr) {
      unitree_reader_ = unitree_lidar_sdk::createUnitreeLidarReader();
    }
    if (unitree_reader_ == nullptr) {
      return common::Status::Unavailable("failed to create Unitree lidar reader");
    }
    if (unitree_reader_->initialize(static_cast<std::uint16_t>(config.cloud_scan_num),
                                    config.device_path,
                                    static_cast<std::uint32_t>(config.baud_rate)) != 0) {
      return common::Status::Unavailable("Unitree lidar initialization failed");
    }
    const auto warmup_deadline = common::Now() + std::chrono::seconds(5);
    int auxiliary_count = 0;
    while (common::Now() < warmup_deadline) {
      const auto warmup_before = pending_sdk_frame_.has_value();
      if (!PumpUnitreeSdkOnce()) {
        ::usleep(500);
      }
      if (pending_sdk_frame_.has_value() && !warmup_before) {
        break;
      }
      if (!pending_sdk_imu_packets_.empty()) {
        ++auxiliary_count;
        sdk_warmup_done_ = true;
      }
    }
    configured_ = true;
    return common::Status::Ok();
#endif
  }

  return common::Status::InvalidArgument("unsupported lidar source: " + config.source);
}

bool L1Driver::WaitForData(std::chrono::milliseconds timeout) {
  if (!configured_) {
    return false;
  }
  if (config_.source == "synthetic") {
    const auto deadline = common::Now() + timeout;
    if (next_frame_time_ <= common::Now()) {
      return true;
    }
    common::SleepUntil(std::min(next_frame_time_, deadline));
    return next_frame_time_ <= common::Now();
  }
  if (config_.source != "unitree_sdk") {
    return false;
  }
  if (pending_sdk_frame_.has_value() || !pending_sdk_imu_packets_.empty()) {
    return true;
  }

  const auto deadline = common::Now() + timeout;
  while (common::Now() < deadline) {
    if (PumpUnitreeSdkOnce()) {
      if (pending_sdk_frame_.has_value() || !pending_sdk_imu_packets_.empty()) {
        return true;
      }
      continue;
    }
    ::usleep(500);
  }
  return pending_sdk_frame_.has_value() || !pending_sdk_imu_packets_.empty();
}

std::optional<data::LidarFrame> L1Driver::PollFrame() {
  if (!configured_) {
    return std::nullopt;
  }
  if (config_.source == "synthetic") {
    return PollSyntheticFrame();
  }
  if (config_.source == "unitree_sdk") {
    return PollUnitreeSdkFrame();
  }
  return std::nullopt;
}

std::optional<data::ImuPacket> L1Driver::PollImuPacket() {
  if (!configured_ || config_.source != "unitree_sdk") {
    return std::nullopt;
  }
  return PollUnitreeSdkImuPacket();
}

std::optional<data::LidarFrame> L1Driver::PollSyntheticFrame() {
  const auto period = std::chrono::duration_cast<common::Duration>(
      std::chrono::duration<double>(1.0 / config_.frame_rate_hz));
  const auto now = common::Now();
  if (next_frame_time_ > now) {
    common::SleepUntil(next_frame_time_);
  }

  data::LidarFrame frame;
  frame.scan_end_stamp = common::Now();
  frame.scan_begin_stamp = frame.scan_end_stamp - period;
  frame.stamp = frame.scan_end_stamp;
  frame.frame_index = frame_index_++;
  frame.points.reserve(config_.points_per_frame);

  const float duration_s = static_cast<float>(common::ToNanoseconds(period)) / 1.0e9F;
  if (!synthetic_map_points_.empty()) {
    const SyntheticPose2d pose = SyntheticTruePose(frame.frame_index);
    std::vector<data::PointXYZI> visible;
    visible.reserve(synthetic_map_points_.size());
    for (const auto& map_point : synthetic_map_points_) {
      const auto local = ToLocalPoint(map_point, pose);
      const float range_sq = local.x * local.x + local.y * local.y;
      if (range_sq > config_.radius_m * config_.radius_m || local.x < -1.0F) {
        continue;
      }
      visible.push_back(local);
    }

    if (!visible.empty()) {
      for (std::size_t index = 0; index < config_.points_per_frame; ++index) {
        data::PointXYZI point = visible[(index * visible.size()) / config_.points_per_frame];
        point.intensity = static_cast<float>((index % 32U) + 1U);
        point.relative_time_s =
            (static_cast<float>(index) / static_cast<float>(config_.points_per_frame)) *
            duration_s;
        frame.points.push_back(point);
      }
    }
  }

  if (frame.points.empty()) {
    for (std::size_t index = 0; index < config_.points_per_frame; ++index) {
      const float angle = static_cast<float>(2.0 * kPi * index /
                                             static_cast<double>(config_.points_per_frame));
      const float radius =
          config_.radius_m + 0.25F * std::sin(static_cast<float>(frame.frame_index) * 0.1F);
      data::PointXYZI point;
      point.x = radius * std::cos(angle);
      point.y = radius * std::sin(angle);
      point.z = 0.05F * std::sin(angle * 4.0F);
      point.intensity = static_cast<float>(index % 255U);
      point.relative_time_s =
          (static_cast<float>(index) / static_cast<float>(config_.points_per_frame)) *
          duration_s;
      frame.points.push_back(point);
    }
  }

  next_frame_time_ += period;
  if (next_frame_time_ < frame.scan_end_stamp) {
    next_frame_time_ = frame.scan_end_stamp + period;
  }
  return frame;
}

std::optional<data::LidarFrame> L1Driver::PollUnitreeSdkFrame() {
#ifndef RM_NAV_HAS_UNITREE_LIDAR_SDK
  return std::nullopt;
#else
  if (unitree_reader_ == nullptr) {
    return std::nullopt;
  }
  if (pending_sdk_frame_.has_value()) {
    auto frame = std::move(*pending_sdk_frame_);
    pending_sdk_frame_.reset();
    return frame;
  }

  const auto deadline = common::Now() + std::chrono::milliseconds(120);
  while (common::Now() < deadline) {
    if (PumpUnitreeSdkOnce()) {
      if (pending_sdk_frame_.has_value()) {
        auto frame = std::move(*pending_sdk_frame_);
        pending_sdk_frame_.reset();
        return frame;
      }
    } else {
      ::usleep(500);
    }
  }
  return std::nullopt;
#endif
}

std::optional<data::ImuPacket> L1Driver::PollUnitreeSdkImuPacket() {
#ifndef RM_NAV_HAS_UNITREE_LIDAR_SDK
  return std::nullopt;
#else
  if (unitree_reader_ == nullptr) {
    return std::nullopt;
  }
  if (!pending_sdk_imu_packets_.empty()) {
    auto packet = pending_sdk_imu_packets_.front();
    pending_sdk_imu_packets_.pop_front();
    return packet;
  }

  const auto deadline = common::Now() + std::chrono::milliseconds(5);
  while (common::Now() < deadline) {
    if (PumpUnitreeSdkOnce()) {
      if (!pending_sdk_imu_packets_.empty()) {
        auto packet = pending_sdk_imu_packets_.front();
        pending_sdk_imu_packets_.pop_front();
        return packet;
      }
    } else {
      ::usleep(500);
    }
  }
  return std::nullopt;
#endif
}

bool L1Driver::PumpUnitreeSdkOnce() {
#ifndef RM_NAV_HAS_UNITREE_LIDAR_SDK
  return false;
#else
  if (unitree_reader_ == nullptr) {
    return false;
  }
  const auto result = unitree_reader_->runParse();
  if (result == unitree_lidar_sdk::VERSION && !sdk_version_logged_) {
    sdk_version_logged_ = true;
    utils::LogInfo("l1_driver",
                   "Unitree firmware=" + unitree_reader_->getVersionOfFirmware() +
                       " sdk=" + unitree_reader_->getVersionOfSDK());
    return true;
  }
  if (result == unitree_lidar_sdk::AUXILIARY) {
    const auto now = common::Now();
    if (last_aux_log_time_ == common::TimePoint{} ||
        now - last_aux_log_time_ >= std::chrono::milliseconds(500)) {
      last_aux_log_time_ = now;
      utils::LogInfo("l1_driver",
                     "Unitree dirty_percentage=" +
                         std::to_string(unitree_reader_->getDirtyPercentage()));
    }
    sdk_warmup_done_ = true;
    return true;
  }
  if (result == unitree_lidar_sdk::POINTCLOUD) {
    pending_sdk_frame_ =
        BuildFrameFromUnitreeCloud(unitree_reader_->getCloud(), frame_index_++,
                                   &driver_time_converter_);
    sdk_warmup_done_ = true;
    return true;
  }
  if (result == unitree_lidar_sdk::IMU) {
    if (pending_sdk_imu_packets_.size() >= 256U) {
      pending_sdk_imu_packets_.pop_front();
    }
    pending_sdk_imu_packets_.push_back(
        BuildPacketFromUnitreeImu(unitree_reader_->getIMU(), imu_sample_index_++,
                                  &driver_time_converter_));
    sdk_warmup_done_ = true;
    return true;
  }
  return false;
#endif
}

}  // namespace rm_nav::drivers::lidar
