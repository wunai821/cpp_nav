#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/config/config_loader.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/mapping/mapping_engine.hpp"
#include "rm_nav/mapping/waypoint_manager.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

struct Options {
  std::string config_dir{"config"};
  std::string output_dir{};
  int duration_s{20};
  int loop_hz{-1};
};

rm_nav::common::Status ParseArgs(int argc, char** argv, Options* options) {
  if (options == nullptr) {
    return rm_nav::common::Status::InvalidArgument("options output is null");
  }
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--config" && index + 1 < argc) {
      options->config_dir = argv[++index];
      continue;
    }
    if (arg == "--output-dir" && index + 1 < argc) {
      options->output_dir = argv[++index];
      continue;
    }
    if (arg == "--duration-s" && index + 1 < argc) {
      options->duration_s = std::atoi(argv[++index]);
      continue;
    }
    if (arg == "--loop-hz" && index + 1 < argc) {
      options->loop_hz = std::atoi(argv[++index]);
      continue;
    }
    return rm_nav::common::Status::InvalidArgument(
        "usage: warmup_mapping_tool [--config dir] [--output-dir dir] "
        "[--duration-s N] [--loop-hz N]");
  }
  if (options->duration_s <= 0) {
    return rm_nav::common::Status::InvalidArgument("duration must be positive");
  }
  return rm_nav::common::Status::Ok();
}

std::filesystem::path ResolvePath(const std::string& config_dir, const std::string& raw_path) {
  const std::filesystem::path path(raw_path);
  if (path.is_absolute()) {
    return path;
  }
  return (std::filesystem::path(config_dir) / path).lexically_normal();
}

rm_nav::data::Pose3f MakePose(rm_nav::common::TimePoint stamp, float x, float y, float yaw) {
  rm_nav::data::Pose3f pose;
  pose.stamp = stamp;
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.position.x = x;
  pose.position.y = y;
  pose.rpy.z = yaw;
  pose.is_valid = true;
  return pose;
}

rm_nav::data::PointXYZI ToLocalPoint(const rm_nav::data::PointXYZI& world_point,
                                     const rm_nav::data::Pose3f& pose) {
  const float dx = world_point.x - pose.position.x;
  const float dy = world_point.y - pose.position.y;
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  rm_nav::data::PointXYZI local = world_point;
  local.x = cos_yaw * dx + sin_yaw * dy;
  local.y = -sin_yaw * dx + cos_yaw * dy;
  local.z = world_point.z;
  return local;
}

rm_nav::data::LidarFrame BuildSyntheticScan(const std::vector<rm_nav::data::PointXYZI>& map_points,
                                            const rm_nav::data::Pose3f& pose,
                                            float radius_m, std::size_t max_points,
                                            std::uint32_t frame_index) {
  rm_nav::data::LidarFrame frame;
  frame.stamp = pose.stamp;
  frame.scan_begin_stamp = pose.stamp;
  frame.scan_end_stamp = pose.stamp;
  frame.frame_index = frame_index;
  frame.is_deskewed = true;
  frame.points.reserve(max_points);

  std::vector<rm_nav::data::PointXYZI> visible;
  visible.reserve(map_points.size());
  for (const auto& world_point : map_points) {
    auto local = ToLocalPoint(world_point, pose);
    if (std::fabs(local.z) < 0.05F) {
      local.z = 0.6F;
    }
    const float range_sq = local.x * local.x + local.y * local.y;
    if (range_sq > radius_m * radius_m) {
      continue;
    }
    visible.push_back(local);
  }

  if (visible.empty()) {
    return frame;
  }

  for (std::size_t index = 0; index < max_points; ++index) {
    auto point = visible[(index * visible.size()) / max_points];
    point.relative_time_s = 0.0F;
    frame.points.push_back(point);
  }
  return frame;
}

rm_nav::data::Pose3f StepPoseToward(const rm_nav::data::Pose3f& current_pose,
                                    const rm_nav::data::Pose3f& goal_pose, float step_m) {
  const float dx = goal_pose.position.x - current_pose.position.x;
  const float dy = goal_pose.position.y - current_pose.position.y;
  const float distance = std::sqrt(dx * dx + dy * dy);
  if (distance <= 1.0e-4F || distance <= step_m) {
    return MakePose(current_pose.stamp, goal_pose.position.x, goal_pose.position.y,
                    goal_pose.rpy.z);
  }
  const float alpha = step_m / distance;
  const float yaw = std::atan2(dy, dx);
  return MakePose(current_pose.stamp, current_pose.position.x + dx * alpha,
                  current_pose.position.y + dy * alpha, yaw);
}

void WriteSummary(const std::filesystem::path& path, const Options& options,
                  const rm_nav::config::MappingConfig& mapping_config,
                  std::size_t frames, std::size_t points) {
  std::ofstream output(path);
  output << "config_dir=" << options.config_dir << "\n";
  output << "duration_s=" << options.duration_s << "\n";
  output << "loop_hz=" << mapping_config.loop_hz << "\n";
  output << "frames=" << frames << "\n";
  output << "global_points=" << points << "\n";
  output << "active_dir=" << mapping_config.active_dir << "\n";
  output << "staging_dir=" << mapping_config.staging_dir << "\n";
  output << "failed_dir=" << mapping_config.failed_dir << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  auto status = ParseArgs(argc, argv, &options);
  if (!status.ok()) {
    std::cerr << status.message << "\n";
    return 1;
  }

  rm_nav::config::ConfigLoader loader;
  rm_nav::config::LoadedConfig loaded_config;
  status = loader.LoadFromDirectory(options.config_dir, &loaded_config);
  if (!status.ok()) {
    std::cerr << status.message << "\n";
    return 1;
  }

  rm_nav::config::MappingConfig mapping_config = loaded_config.mapping;
  if (options.loop_hz > 0) {
    mapping_config.loop_hz = options.loop_hz;
  }
  if (!options.output_dir.empty()) {
    const std::filesystem::path override_path(options.output_dir);
    mapping_config.active_dir =
        (override_path.is_absolute() ? override_path : std::filesystem::current_path() / override_path)
            .lexically_normal()
            .string();
  } else {
    mapping_config.active_dir =
        ResolvePath(options.config_dir, mapping_config.active_dir).lexically_normal().string();
  }
  if (!mapping_config.staging_dir.empty()) {
    mapping_config.staging_dir =
        ResolvePath(options.config_dir, mapping_config.staging_dir).lexically_normal().string();
  }
  if (!mapping_config.failed_dir.empty()) {
    mapping_config.failed_dir =
        ResolvePath(options.config_dir, mapping_config.failed_dir).lexically_normal().string();
  }

  rm_nav::mapping::WaypointManager waypoint_manager;
  status = waypoint_manager.Load(
      ResolvePath(options.config_dir, mapping_config.waypoint_path).string());
  if (!status.ok()) {
    std::cerr << status.message << "\n";
    return 1;
  }

  rm_nav::localization::MapLoader map_loader;
  rm_nav::localization::StaticMap environment_map;
  status = map_loader.Load(options.config_dir, loaded_config.localization, &environment_map);
  if (!status.ok()) {
    std::cerr << status.message << "\n";
    return 1;
  }

  rm_nav::mapping::MappingEngine engine;
  status = engine.Initialize(mapping_config);
  if (!status.ok()) {
    std::cerr << status.message << "\n";
    return 1;
  }

  const float dt_s = 1.0F / static_cast<float>(std::max(1, mapping_config.loop_hz));
  auto current_pose = waypoint_manager.CurrentGoal();
  waypoint_manager.AdvanceIfReached(current_pose,
                                    static_cast<float>(mapping_config.route_reach_tolerance_m));

  std::size_t frame_count = 0U;
  const std::size_t total_steps =
      static_cast<std::size_t>(options.duration_s * std::max(1, mapping_config.loop_hz));
  for (std::size_t step = 0; step < total_steps; ++step) {
    const auto stamp = rm_nav::common::Now();
    current_pose.stamp = stamp;
    const auto goal_pose = waypoint_manager.CurrentGoal();
    current_pose = StepPoseToward(
        current_pose, goal_pose,
        static_cast<float>(mapping_config.route_speed_mps) * dt_s);
    current_pose.stamp = stamp;
    current_pose.reference_frame = rm_nav::tf::kMapFrame;
    current_pose.child_frame = rm_nav::tf::kBaseLinkFrame;
    current_pose.is_valid = true;

    rm_nav::data::SyncedFrame frame;
    frame.stamp = stamp;
    frame.lidar = BuildSyntheticScan(
        environment_map.global_points, current_pose,
        static_cast<float>(mapping_config.synthetic_scan_radius_m),
        static_cast<std::size_t>(std::max(32, mapping_config.synthetic_points_per_frame)),
        static_cast<std::uint32_t>(step));

    status = engine.Update(frame, current_pose);
    if (!status.ok()) {
      std::cerr << status.message << "\n";
      return 1;
    }
    ++frame_count;
    waypoint_manager.AdvanceIfReached(
        current_pose, static_cast<float>(mapping_config.route_reach_tolerance_m));
  }

  rm_nav::localization::StaticMap exported_map;
  status = engine.SaveMap(mapping_config.active_dir, &exported_map);
  if (!status.ok()) {
    std::cerr << status.message << "\n";
    return 1;
  }

  rm_nav::config::LocalizationConfig saved_config;
  saved_config.global_map_pcd_path = "global_map.pcd";
  saved_config.occupancy_path = "occupancy.bin";
  saved_config.map_meta_path = "map_meta.json";
  rm_nav::localization::StaticMap reload_map;
  status = map_loader.Load(mapping_config.active_dir, saved_config, &reload_map);
  if (!status.ok()) {
    std::cerr << "reload failed: " << status.message << "\n";
    return 1;
  }

  WriteSummary(std::filesystem::path(mapping_config.active_dir) / "summary.txt", options,
               mapping_config, frame_count, reload_map.global_points.size());
  std::cout << "warmup_mapping_tool finished, active_dir=" << mapping_config.active_dir << "\n";
  return 0;
}
