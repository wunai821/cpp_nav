#include "rm_nav/drivers/lidar/l1_driver.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

#include "rm_nav/common/time.hpp"

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

bool LoadSyntheticMap(const std::string& path,
                      std::vector<data::PointXYZI>* points) {
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

data::PointXYZI ToLocalPoint(const data::PointXYZI& world_point,
                             const SyntheticPose2d& pose) {
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

}  // namespace

common::Status L1Driver::Configure(const L1DriverConfig& config) {
  if (config.source != "synthetic") {
    return common::Status::Unimplemented("only synthetic L1 source is available");
  }
  if (config.frame_rate_hz <= 0.0 || config.points_per_frame == 0U) {
    return common::Status::InvalidArgument("invalid synthetic L1 config");
  }
  const auto period = std::chrono::duration_cast<common::Duration>(
      std::chrono::duration<double>(1.0 / config.frame_rate_hz));
  config_ = config;
  next_frame_time_ = common::Now() + period;
  frame_index_ = 0;
  synthetic_map_points_.clear();
  if (!config.synthetic_map_pcd_path.empty()) {
    LoadSyntheticMap(config.synthetic_map_pcd_path, &synthetic_map_points_);
  }
  configured_ = true;
  return common::Status::Ok();
}

std::optional<data::LidarFrame> L1Driver::PollFrame() {
  if (!configured_) {
    return std::nullopt;
  }

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
        data::PointXYZI point =
            visible[(index * visible.size()) / config_.points_per_frame];
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

}  // namespace rm_nav::drivers::lidar
