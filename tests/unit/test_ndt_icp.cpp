#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/common/status.hpp"
#include "rm_nav/common/time.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/localization/icp_matcher.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/localization/ndt_matcher.hpp"
#include "rm_nav/tf/frame_ids.hpp"

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
  rm_nav::localization::MapLoader loader;
  rm_nav::localization::StaticMap map;
  assert(loader.Load("config", localization_config, &map).ok());

  constexpr float kTrueX = 2.8F;
  constexpr float kTrueY = 2.4F;
  constexpr float kTrueYaw = 0.15F;

  rm_nav::data::LidarFrame scan;
  scan.frame_id = rm_nav::tf::kLaserFrame;
  scan.stamp = rm_nav::common::Now();
  for (std::size_t index = 0; index < map.global_points.size(); ++index) {
    const auto local = ToLocal(map.global_points[index], kTrueX, kTrueY, kTrueYaw,
                               static_cast<float>(index) * 0.001F);
    const float range_sq = local.x * local.x + local.y * local.y;
    if (range_sq > 25.0F || local.x < -1.0F) {
      continue;
    }
    scan.points.push_back(local);
  }
  assert(!scan.points.empty());

  rm_nav::data::Pose3f initial_guess;
  initial_guess.reference_frame = rm_nav::tf::kMapFrame;
  initial_guess.child_frame = rm_nav::tf::kBaseLinkFrame;
  initial_guess.position.x = 2.55F;
  initial_guess.position.y = 2.55F;
  initial_guess.rpy.z = 0.05F;
  initial_guess.is_valid = true;

  rm_nav::localization::IcpMatcher icp;
  rm_nav::localization::ScanMatchConfig config;
  config.max_iterations = 12;
  config.correspondence_distance_m = 0.8F;
  config.min_match_score = 0.2F;
  assert(icp.Configure(config).ok());

  rm_nav::localization::ScanMatchResult icp_result;
  assert(icp.Match(map, scan, initial_guess, &icp_result).ok());
  assert(icp_result.matched_pose.is_valid);
  assert(icp_result.iterations > 0);
  assert(icp_result.score >= 0.0F);

  rm_nav::localization::NdtMatcher ndt;
  assert(ndt.Configure(config).ok());
  rm_nav::localization::ScanMatchResult ndt_result;
  const auto ndt_status = ndt.Match(map, scan, initial_guess, &ndt_result);
  assert(ndt_status.code == rm_nav::common::StatusCode::kUnimplemented);

  std::cout << "test_ndt_icp passed\n";
  return 0;
}
