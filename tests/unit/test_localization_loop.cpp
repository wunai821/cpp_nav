#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/config/spawn_config.hpp"
#include "rm_nav/data/odom_state.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/localization/localization_engine.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/tf/tf_tree_lite.hpp"

namespace {

rm_nav::data::PointXYZI ToLocal(const rm_nav::data::PointXYZI& world_point, float x, float y,
                                float yaw, float relative_time_s) {
  const float dx = world_point.x - x;
  const float dy = world_point.y - y;
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  rm_nav::data::PointXYZI local = world_point;
  local.x = cos_yaw * dx + sin_yaw * dy;
  local.y = -sin_yaw * dx + cos_yaw * dy;
  local.relative_time_s = relative_time_s;
  return local;
}

}  // namespace

int main() {
  rm_nav::config::LocalizationConfig localization_config;
  rm_nav::config::SpawnConfig spawn_config;
  spawn_config.x_m = 2.5;
  spawn_config.y_m = 2.2;
  spawn_config.theta_rad = 0.1;
  rm_nav::localization::MapLoader loader;
  rm_nav::localization::StaticMap map;
  assert(loader.Load("config", localization_config, &map).ok());

  rm_nav::tf::TfTreeLite tf_tree;
  rm_nav::localization::LocalizationEngine localization;
  assert(localization.Initialize("config", localization_config, spawn_config, &tf_tree).ok());

  constexpr float kTrueX = 2.8F;
  constexpr float kTrueY = 2.4F;
  constexpr float kTrueYaw = 0.15F;

  rm_nav::data::SyncedFrame frame;
  frame.stamp = rm_nav::common::Now();
  frame.lidar.stamp = frame.stamp;
  frame.lidar.scan_begin_stamp = frame.stamp - std::chrono::milliseconds(100);
  frame.lidar.scan_end_stamp = frame.stamp;
  frame.lidar.frame_index = 42;

  for (std::size_t index = 0; index < map.global_points.size(); ++index) {
    const auto local = ToLocal(map.global_points[index], kTrueX, kTrueY, kTrueYaw,
                               static_cast<float>(index) * 0.001F);
    const float range_sq = local.x * local.x + local.y * local.y;
    if (range_sq > 25.0F || local.x < -1.0F) {
      continue;
    }
    frame.lidar.points.push_back(local);
  }
  assert(!frame.lidar.points.empty());

  rm_nav::data::OdomState odom;
  odom.stamp = frame.stamp;
  odom.x_m = 2.55F;
  odom.y_m = 2.55F;
  odom.yaw_rad = 0.05F;
  localization.SetLatestOdom(odom);

  rm_nav::localization::LocalizationResult result;
  assert(localization.Process(frame, &result).ok());
  assert(result.map_to_base.is_valid);
  assert(result.map_to_odom.is_valid);
  assert(result.status.iterations > 0);
  assert(result.status.map_loaded);
  assert(result.status.consecutive_failures <= 1U);

  std::cout << "test_localization_loop passed\n";
  return 0;
}
