#include "rm_nav/config/config_loader.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace rm_nav::config {
namespace {

using FlatConfig = std::unordered_map<std::string, std::string>;

std::string Trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::string StripQuotes(const std::string& value) {
  if (value.size() >= 2U &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    return value.substr(1, value.size() - 2U);
  }
  return value;
}

std::string JoinKeys(const std::vector<std::string>& stack, const std::string& key,
                     std::size_t level) {
  std::string joined;
  for (std::size_t index = 0; index < level; ++index) {
    if (stack[index].empty()) {
      continue;
    }
    if (!joined.empty()) {
      joined += '.';
    }
    joined += stack[index];
  }
  if (!joined.empty()) {
    joined += '.';
  }
  joined += key;
  return joined;
}

common::Status ParseYamlFile(const std::filesystem::path& path, FlatConfig* values) {
  std::ifstream input(path);
  if (!input.is_open()) {
    return common::Status::Unavailable("failed to open config file");
  }

  std::vector<std::string> stack(16);
  std::string line;
  while (std::getline(input, line)) {
    const auto hash_pos = line.find('#');
    if (hash_pos != std::string::npos) {
      line = line.substr(0, hash_pos);
    }

    if (Trim(line).empty()) {
      continue;
    }

    const auto indent = line.find_first_not_of(' ');
    if (indent == std::string::npos) {
      continue;
    }
    if (indent % 2U != 0U) {
      return common::Status::InvalidArgument("yaml indentation must use 2 spaces");
    }

    const std::size_t level = indent / 2U;
    if (level >= stack.size()) {
      stack.resize(level + 1U);
    }
    for (std::size_t index = level; index < stack.size(); ++index) {
      stack[index].clear();
    }

    const std::string content = Trim(line.substr(indent));
    const auto colon_pos = content.find(':');
    if (colon_pos == std::string::npos) {
      return common::Status::InvalidArgument("yaml line missing ':'");
    }

    const std::string key = Trim(content.substr(0, colon_pos));
    const std::string raw_value = Trim(content.substr(colon_pos + 1U));
    if (raw_value.empty()) {
      stack[level] = key;
      continue;
    }

    (*values)[JoinKeys(stack, key, level)] = StripQuotes(raw_value);
  }

  return common::Status::Ok();
}

std::string GetString(const FlatConfig& values, const std::string& key,
                      const std::string& fallback) {
  const auto it = values.find(key);
  return it == values.end() ? fallback : it->second;
}

bool GetBool(const FlatConfig& values, const std::string& key, bool fallback) {
  const auto value = GetString(values, key, fallback ? "true" : "false");
  return value == "true" || value == "True" || value == "1";
}

int GetInt(const FlatConfig& values, const std::string& key, int fallback) {
  const auto it = values.find(key);
  if (it == values.end()) {
    return fallback;
  }
  return std::stoi(it->second);
}

double GetDouble(const FlatConfig& values, const std::string& key, double fallback) {
  const auto it = values.find(key);
  if (it == values.end()) {
    return fallback;
  }
  return std::stod(it->second);
}

std::filesystem::path ResolveConfigFile(const std::filesystem::path& config_dir,
                                        const std::string& base_name) {
  const auto yaml_path = config_dir / (base_name + ".yaml");
  if (std::filesystem::exists(yaml_path)) {
    return yaml_path;
  }
  return config_dir / (base_name + ".yam");
}

bool FileLooksPresent(const std::filesystem::path& path) {
  return std::filesystem::exists(path) && std::filesystem::is_regular_file(path);
}

}  // namespace

common::Status ConfigLoader::LoadFromDirectory(
    const std::string& config_dir, LoadedConfig* loaded_config) const {
  if (loaded_config == nullptr) {
    return common::Status::InvalidArgument("loaded_config must not be null");
  }

  const std::filesystem::path config_path(config_dir);
  if (!std::filesystem::exists(config_path) ||
      !std::filesystem::is_directory(config_path)) {
    return common::Status::Unavailable("config directory not found");
  }

  LoadedConfig result;
  result.config_dir = config_path.lexically_normal().string();

  const std::vector<std::string> required_files = {
      "system", "frames", "sensors", "comm", "debug", "planner", "safety"};
  const std::vector<std::string> optional_files = {"localization", "mapping", "spawn"};
  FlatConfig merged_values;

  try {
    for (const auto& file : required_files) {
      const auto file_path = ResolveConfigFile(config_path, file);
      if (!std::filesystem::exists(file_path)) {
        return common::Status::Unavailable("required config file is missing");
      }

      FlatConfig file_values;
      auto status = ParseYamlFile(file_path, &file_values);
      if (!status.ok()) {
        return status;
      }

      merged_values.insert(file_values.begin(), file_values.end());
      result.loaded_files.push_back(file_path.filename().string());
    }

    for (const auto& file : optional_files) {
      const auto file_path = ResolveConfigFile(config_path, file);
      if (!FileLooksPresent(file_path)) {
        continue;
      }

      FlatConfig file_values;
      auto status = ParseYamlFile(file_path, &file_values);
      if (!status.ok()) {
        return status;
      }

      merged_values.insert(file_values.begin(), file_values.end());
      result.loaded_files.push_back(file_path.filename().string());
    }

    result.system.app_name = GetString(merged_values, "app_name", result.system.app_name);
    result.system.version = GetString(merged_values, "version", result.system.version);
    result.system.log_level =
        GetString(merged_values, "log_level", result.system.log_level);
    result.system.bringup_mode =
        GetString(merged_values, "bringup_mode", result.system.bringup_mode);
    result.system.auto_shutdown_ms =
        GetInt(merged_values, "auto_shutdown_ms", result.system.auto_shutdown_ms);
    result.system.manual_mode_selector =
        GetInt(merged_values, "manual_mode_selector",
               result.system.manual_mode_selector);
    result.system.thread_affinity.driver_cpu =
        GetInt(merged_values, "threads.driver_cpu",
               result.system.thread_affinity.driver_cpu);
    result.system.thread_affinity.sync_cpu =
        GetInt(merged_values, "threads.sync_cpu",
               result.system.thread_affinity.sync_cpu);
    result.system.thread_affinity.pose_core_cpu =
        GetInt(merged_values, "threads.pose_core_cpu",
               result.system.thread_affinity.pose_core_cpu);
    result.system.thread_affinity.perception_cpu =
        GetInt(merged_values, "threads.perception_cpu",
               result.system.thread_affinity.perception_cpu);
    result.system.thread_affinity.planner_cpu =
        GetInt(merged_values, "threads.planner_cpu",
               result.system.thread_affinity.planner_cpu);
    result.system.thread_affinity.safety_fsm_cpu =
        GetInt(merged_values, "threads.safety_fsm_cpu",
               result.system.thread_affinity.safety_fsm_cpu);
    result.system.thread_affinity.debug_cpu =
        GetInt(merged_values, "threads.debug_cpu",
               result.system.thread_affinity.debug_cpu);

    result.frames.map = GetString(merged_values, "map", result.frames.map);
    result.frames.odom = GetString(merged_values, "odom", result.frames.odom);
    result.frames.base_link =
        GetString(merged_values, "base_link", result.frames.base_link);
    result.frames.laser_link =
        GetString(merged_values, "laser_link", result.frames.laser_link);
    result.frames.imu_link = GetString(merged_values, "imu_link", result.frames.imu_link);

    result.sensors.lidar_enabled =
        GetBool(merged_values, "lidar.enabled", result.sensors.lidar_enabled);
    result.sensors.lidar_source =
        GetString(merged_values, "lidar.source", result.sensors.lidar_source);
    result.sensors.lidar_model =
        GetString(merged_values, "lidar.model", result.sensors.lidar_model);
    result.sensors.lidar_port =
        GetString(merged_values, "lidar.port", result.sensors.lidar_port);
    result.sensors.lidar_baud_rate =
        GetInt(merged_values, "lidar.baud_rate", result.sensors.lidar_baud_rate);
    result.sensors.lidar_cloud_scan_num =
        GetInt(merged_values, "lidar.cloud_scan_num", result.sensors.lidar_cloud_scan_num);
    result.sensors.lidar_mount.x_m =
        static_cast<float>(GetDouble(merged_values, "lidar.mount.x_m",
                                     result.sensors.lidar_mount.x_m));
    result.sensors.lidar_mount.y_m =
        static_cast<float>(GetDouble(merged_values, "lidar.mount.y_m",
                                     result.sensors.lidar_mount.y_m));
    result.sensors.lidar_mount.z_m =
        static_cast<float>(GetDouble(merged_values, "lidar.mount.z_m",
                                     result.sensors.lidar_mount.z_m));
    result.sensors.lidar_mount.roll_rad =
        static_cast<float>(GetDouble(merged_values, "lidar.mount.roll_rad",
                                     result.sensors.lidar_mount.roll_rad));
    result.sensors.lidar_mount.pitch_rad =
        static_cast<float>(GetDouble(merged_values, "lidar.mount.pitch_rad",
                                     result.sensors.lidar_mount.pitch_rad));
    result.sensors.lidar_mount.yaw_rad =
        static_cast<float>(GetDouble(merged_values, "lidar.mount.yaw_rad",
                                     result.sensors.lidar_mount.yaw_rad));
    result.sensors.lidar_self_mask.enabled =
        GetBool(merged_values, "lidar.self_mask.enabled",
                result.sensors.lidar_self_mask.enabled);
    result.sensors.lidar_self_mask.x_min_m =
        static_cast<float>(GetDouble(merged_values, "lidar.self_mask.x_min_m",
                                     result.sensors.lidar_self_mask.x_min_m));
    result.sensors.lidar_self_mask.x_max_m =
        static_cast<float>(GetDouble(merged_values, "lidar.self_mask.x_max_m",
                                     result.sensors.lidar_self_mask.x_max_m));
    result.sensors.lidar_self_mask.y_min_m =
        static_cast<float>(GetDouble(merged_values, "lidar.self_mask.y_min_m",
                                     result.sensors.lidar_self_mask.y_min_m));
    result.sensors.lidar_self_mask.y_max_m =
        static_cast<float>(GetDouble(merged_values, "lidar.self_mask.y_max_m",
                                     result.sensors.lidar_self_mask.y_max_m));
    result.sensors.imu_enabled =
        GetBool(merged_values, "imu.enabled", result.sensors.imu_enabled);
    result.sensors.imu_source =
        GetString(merged_values, "imu.source", result.sensors.imu_source);
    result.sensors.imu_model =
        GetString(merged_values, "imu.model", result.sensors.imu_model);
    result.sensors.imu_mount.x_m =
        static_cast<float>(GetDouble(merged_values, "imu.mount.x_m",
                                     result.sensors.imu_mount.x_m));
    result.sensors.imu_mount.y_m =
        static_cast<float>(GetDouble(merged_values, "imu.mount.y_m",
                                     result.sensors.imu_mount.y_m));
    result.sensors.imu_mount.z_m =
        static_cast<float>(GetDouble(merged_values, "imu.mount.z_m",
                                     result.sensors.imu_mount.z_m));
    result.sensors.imu_mount.roll_rad =
        static_cast<float>(GetDouble(merged_values, "imu.mount.roll_rad",
                                     result.sensors.imu_mount.roll_rad));
    result.sensors.imu_mount.pitch_rad =
        static_cast<float>(GetDouble(merged_values, "imu.mount.pitch_rad",
                                     result.sensors.imu_mount.pitch_rad));
    result.sensors.imu_mount.yaw_rad =
        static_cast<float>(GetDouble(merged_values, "imu.mount.yaw_rad",
                                     result.sensors.imu_mount.yaw_rad));

    result.comm.stm32_enabled =
        GetBool(merged_values, "stm32.enabled", result.comm.stm32_enabled);
    result.comm.stm32_port =
        GetString(merged_values, "stm32.port", result.comm.stm32_port);
    result.comm.stm32_baud_rate =
        GetInt(merged_values, "stm32.baud_rate", result.comm.stm32_baud_rate);

    result.debug.websocket_enabled =
        GetBool(merged_values, "websocket.enabled", result.debug.websocket_enabled);
    result.debug.websocket_host =
        GetString(merged_values, "websocket.host", result.debug.websocket_host);
    result.debug.websocket_port =
        GetInt(merged_values, "websocket.port", result.debug.websocket_port);
    result.debug.publish_hz =
        GetInt(merged_values, "websocket.publish_hz", result.debug.publish_hz);
    result.debug.pointcloud_publish_hz =
        GetInt(merged_values, "websocket.pointcloud_publish_hz",
               result.debug.pointcloud_publish_hz);
    result.debug.scalar_publish_hz =
        GetInt(merged_values, "websocket.scalar_publish_hz",
               result.debug.scalar_publish_hz);
    result.debug.high_load_debug_threshold_ms =
        GetInt(merged_values, "websocket.high_load_debug_threshold_ms",
               result.debug.high_load_debug_threshold_ms);
    result.debug.drop_pointcloud_on_high_load =
        GetBool(merged_values, "websocket.drop_pointcloud_on_high_load",
                result.debug.drop_pointcloud_on_high_load);

    result.localization.enabled =
        GetBool(merged_values, "localization.enabled", result.localization.enabled);
    result.localization.occupancy_path = GetString(
        merged_values, "localization.occupancy_path", result.localization.occupancy_path);
    result.localization.global_map_pcd_path =
        GetString(merged_values, "localization.global_map_pcd_path",
                  result.localization.global_map_pcd_path);
    result.localization.map_meta_path = GetString(
        merged_values, "localization.map_meta_path", result.localization.map_meta_path);
    result.localization.matcher =
        GetString(merged_values, "localization.matcher", result.localization.matcher);
    result.localization.max_iterations = GetInt(
        merged_values, "localization.max_iterations", result.localization.max_iterations);
    result.localization.correspondence_distance_m =
        GetDouble(merged_values, "localization.correspondence_distance_m",
                  result.localization.correspondence_distance_m);
    result.localization.min_match_score =
        GetDouble(merged_values, "localization.min_match_score",
                  result.localization.min_match_score);
    result.localization.max_position_jump_m =
        GetDouble(merged_values, "localization.max_position_jump_m",
                  result.localization.max_position_jump_m);
    result.localization.max_yaw_jump_rad =
        GetDouble(merged_values, "localization.max_yaw_jump_rad",
                  result.localization.max_yaw_jump_rad);
    result.localization.map_downsample_step =
        GetInt(merged_values, "localization.map_downsample_step",
               result.localization.map_downsample_step);
    result.localization.slow_match_threshold_ms =
        GetInt(merged_values, "localization.slow_match_threshold_ms",
               result.localization.slow_match_threshold_ms);
    result.localization.reduced_max_iterations =
        GetInt(merged_values, "localization.reduced_max_iterations",
               result.localization.reduced_max_iterations);
    result.localization.reduced_scan_stride =
        GetInt(merged_values, "localization.reduced_scan_stride",
               result.localization.reduced_scan_stride);
    result.localization.relocalization_failure_threshold =
        GetInt(merged_values, "localization.relocalization_failure_threshold",
               result.localization.relocalization_failure_threshold);
    result.localization.relocalization_retry_interval_ms =
        GetInt(merged_values, "localization.relocalization_retry_interval_ms",
               result.localization.relocalization_retry_interval_ms);
    result.localization.relocalization_submap_radius_m =
        GetDouble(merged_values, "localization.relocalization_submap_radius_m",
                  result.localization.relocalization_submap_radius_m);
    result.localization.relocalization_submap_max_points =
        GetInt(merged_values, "localization.relocalization_submap_max_points",
               result.localization.relocalization_submap_max_points);
    result.localization.relocalization_max_iterations =
        GetInt(merged_values, "localization.relocalization_max_iterations",
               result.localization.relocalization_max_iterations);
    result.localization.relocalization_min_match_score =
        GetDouble(merged_values, "localization.relocalization_min_match_score",
                  result.localization.relocalization_min_match_score);
    result.localization.relocalization_linear_search_step_m =
        GetDouble(merged_values, "localization.relocalization_linear_search_step_m",
                  result.localization.relocalization_linear_search_step_m);
    result.localization.relocalization_yaw_search_step_rad =
        GetDouble(merged_values, "localization.relocalization_yaw_search_step_rad",
                  result.localization.relocalization_yaw_search_step_rad);
    result.localization.relocalization_stabilization_frames =
        GetInt(merged_values, "localization.relocalization_stabilization_frames",
               result.localization.relocalization_stabilization_frames);
    result.localization.relocalization_stabilization_time_ms =
        GetInt(merged_values, "localization.relocalization_stabilization_time_ms",
               result.localization.relocalization_stabilization_time_ms);
    result.localization.relocalization_map_to_odom_apply_translation_step_m =
        GetDouble(merged_values,
                  "localization.relocalization_map_to_odom_apply_translation_step_m",
                  result.localization.relocalization_map_to_odom_apply_translation_step_m);
    result.localization.relocalization_map_to_odom_apply_yaw_step_rad =
        GetDouble(merged_values,
                  "localization.relocalization_map_to_odom_apply_yaw_step_rad",
                  result.localization.relocalization_map_to_odom_apply_yaw_step_rad);
    result.localization.relocalization_recovery_spin_wz_radps =
        GetDouble(merged_values, "localization.relocalization_recovery_spin_wz_radps",
                  result.localization.relocalization_recovery_spin_wz_radps);
    result.localization.relocalization_recovery_backoff_vx_mps =
        GetDouble(merged_values, "localization.relocalization_recovery_backoff_vx_mps",
                  result.localization.relocalization_recovery_backoff_vx_mps);
    result.localization.relocalization_recovery_backoff_ticks =
        GetInt(merged_values, "localization.relocalization_recovery_backoff_ticks",
               result.localization.relocalization_recovery_backoff_ticks);
    result.localization.relocalization_ambiguity_score_gap =
        GetDouble(merged_values, "localization.relocalization_ambiguity_score_gap",
                  result.localization.relocalization_ambiguity_score_gap);
    result.localization.relocalization_ambiguity_translation_m =
        GetDouble(merged_values, "localization.relocalization_ambiguity_translation_m",
                  result.localization.relocalization_ambiguity_translation_m);
    result.localization.relocalization_ambiguity_yaw_rad =
        GetDouble(merged_values, "localization.relocalization_ambiguity_yaw_rad",
                  result.localization.relocalization_ambiguity_yaw_rad);
    result.localization.map_to_odom_guard_translation_m =
        GetDouble(merged_values, "localization.map_to_odom_guard_translation_m",
                  result.localization.map_to_odom_guard_translation_m);
    result.localization.map_to_odom_guard_yaw_rad =
        GetDouble(merged_values, "localization.map_to_odom_guard_yaw_rad",
                  result.localization.map_to_odom_guard_yaw_rad);

    result.mapping.enabled =
        GetBool(merged_values, "mapping.enabled", result.mapping.enabled);
    result.mapping.loop_hz =
        GetInt(merged_values, "mapping.loop_hz", result.mapping.loop_hz);
    result.mapping.output_dir =
        GetString(merged_values, "mapping.output_dir", result.mapping.output_dir);
    result.mapping.staging_dir =
        GetString(merged_values, "mapping.staging_dir", result.mapping.staging_dir);
    result.mapping.last_good_dir =
        GetString(merged_values, "mapping.last_good_dir", result.mapping.last_good_dir);
    result.mapping.failed_dir =
        GetString(merged_values, "mapping.failed_dir", result.mapping.failed_dir);
    result.mapping.waypoint_path =
        GetString(merged_values, "mapping.waypoint_path", result.mapping.waypoint_path);
    result.mapping.route_speed_mps =
        GetDouble(merged_values, "mapping.route_speed_mps",
                  result.mapping.route_speed_mps);
    result.mapping.route_reach_tolerance_m =
        GetDouble(merged_values, "mapping.route_reach_tolerance_m",
                  result.mapping.route_reach_tolerance_m);
    result.mapping.voxel_size_m =
        GetDouble(merged_values, "mapping.voxel_size_m", result.mapping.voxel_size_m);
    result.mapping.compression_interval_frames =
        GetInt(merged_values, "mapping.compression_interval_frames",
               result.mapping.compression_interval_frames);
    result.mapping.z_min_m =
        GetDouble(merged_values, "mapping.z_min_m", result.mapping.z_min_m);
    result.mapping.z_max_m =
        GetDouble(merged_values, "mapping.z_max_m", result.mapping.z_max_m);
    result.mapping.occupancy_resolution_m =
        GetDouble(merged_values, "mapping.occupancy_resolution_m",
                  result.mapping.occupancy_resolution_m);
    result.mapping.occupancy_padding_m =
        GetDouble(merged_values, "mapping.occupancy_padding_m",
                  result.mapping.occupancy_padding_m);
    result.mapping.occupancy_hit_log_odds =
        GetDouble(merged_values, "mapping.occupancy_hit_log_odds",
                  result.mapping.occupancy_hit_log_odds);
    result.mapping.occupancy_miss_log_odds =
        GetDouble(merged_values, "mapping.occupancy_miss_log_odds",
                  result.mapping.occupancy_miss_log_odds);
    result.mapping.occupancy_min_log_odds =
        GetDouble(merged_values, "mapping.occupancy_min_log_odds",
                  result.mapping.occupancy_min_log_odds);
    result.mapping.occupancy_max_log_odds =
        GetDouble(merged_values, "mapping.occupancy_max_log_odds",
                  result.mapping.occupancy_max_log_odds);
    result.mapping.occupancy_free_threshold_log_odds =
        GetDouble(merged_values, "mapping.occupancy_free_threshold_log_odds",
                  result.mapping.occupancy_free_threshold_log_odds);
    result.mapping.occupancy_occupied_threshold_log_odds =
        GetDouble(merged_values, "mapping.occupancy_occupied_threshold_log_odds",
                  result.mapping.occupancy_occupied_threshold_log_odds);
    result.mapping.occupancy_inflation_radius_m =
        GetDouble(merged_values, "mapping.occupancy_inflation_radius_m",
                  result.mapping.occupancy_inflation_radius_m);
    result.mapping.synthetic_scan_radius_m =
        GetDouble(merged_values, "mapping.synthetic_scan_radius_m",
                  result.mapping.synthetic_scan_radius_m);
    result.mapping.synthetic_points_per_frame =
        GetInt(merged_values, "mapping.synthetic_points_per_frame",
               result.mapping.synthetic_points_per_frame);
    result.mapping.dynamic_suppression_enabled =
        GetBool(merged_values, "mapping.dynamic_suppression_enabled",
                result.mapping.dynamic_suppression_enabled);
    result.mapping.dynamic_near_field_radius_m =
        GetDouble(merged_values, "mapping.dynamic_near_field_radius_m",
                  result.mapping.dynamic_near_field_radius_m);
    result.mapping.dynamic_consistency_frames =
        GetInt(merged_values, "mapping.dynamic_consistency_frames",
               result.mapping.dynamic_consistency_frames);
    result.mapping.dynamic_pending_ttl_frames =
        GetInt(merged_values, "mapping.dynamic_pending_ttl_frames",
               result.mapping.dynamic_pending_ttl_frames);
    result.mapping.dynamic_known_obstacle_mask_enabled =
        GetBool(merged_values, "mapping.dynamic_known_obstacle_mask_enabled",
                result.mapping.dynamic_known_obstacle_mask_enabled);
    result.mapping.dynamic_known_obstacle_margin_m =
        GetDouble(merged_values, "mapping.dynamic_known_obstacle_margin_m",
                  result.mapping.dynamic_known_obstacle_margin_m);
    result.mapping.dynamic_known_obstacle_min_confidence =
        GetDouble(merged_values, "mapping.dynamic_known_obstacle_min_confidence",
                  result.mapping.dynamic_known_obstacle_min_confidence);
    result.mapping.pose_source =
        GetString(merged_values, "mapping.pose_source", result.mapping.pose_source);
    result.mapping.frontend_match_max_iterations =
        GetInt(merged_values, "mapping.frontend_match_max_iterations",
               result.mapping.frontend_match_max_iterations);
    result.mapping.frontend_correspondence_distance_m =
        GetDouble(merged_values, "mapping.frontend_correspondence_distance_m",
                  result.mapping.frontend_correspondence_distance_m);
    result.mapping.frontend_min_match_score =
        GetDouble(merged_values, "mapping.frontend_min_match_score",
                  result.mapping.frontend_min_match_score);
    result.mapping.frontend_match_max_points =
        GetInt(merged_values, "mapping.frontend_match_max_points",
               result.mapping.frontend_match_max_points);
    result.mapping.frontend_min_map_points =
        GetInt(merged_values, "mapping.frontend_min_map_points",
               result.mapping.frontend_min_map_points);
    result.mapping.frontend_submap_radius_m =
        GetDouble(merged_values, "mapping.frontend_submap_radius_m",
                  result.mapping.frontend_submap_radius_m);
    result.mapping.frontend_submap_max_points =
        GetInt(merged_values, "mapping.frontend_submap_max_points",
               result.mapping.frontend_submap_max_points);
    result.mapping.keyframe_translation_threshold_m =
        GetDouble(merged_values, "mapping.keyframe_translation_threshold_m",
                  result.mapping.keyframe_translation_threshold_m);
    result.mapping.keyframe_yaw_threshold_rad =
        GetDouble(merged_values, "mapping.keyframe_yaw_threshold_rad",
                  result.mapping.keyframe_yaw_threshold_rad);
    result.mapping.max_keyframes =
        GetInt(merged_values, "mapping.max_keyframes", result.mapping.max_keyframes);
    result.mapping.keyframe_max_points =
        GetInt(merged_values, "mapping.keyframe_max_points",
               result.mapping.keyframe_max_points);
    result.mapping.loop_candidate_distance_threshold_m =
        GetDouble(merged_values, "mapping.loop_candidate_distance_threshold_m",
                  result.mapping.loop_candidate_distance_threshold_m);
    result.mapping.loop_candidate_min_time_separation_s =
        GetDouble(merged_values, "mapping.loop_candidate_min_time_separation_s",
                  result.mapping.loop_candidate_min_time_separation_s);
    result.mapping.loop_candidate_max_yaw_delta_rad =
        GetDouble(merged_values, "mapping.loop_candidate_max_yaw_delta_rad",
                  result.mapping.loop_candidate_max_yaw_delta_rad);
    result.mapping.loop_matcher =
        GetString(merged_values, "mapping.loop_matcher", result.mapping.loop_matcher);
    result.mapping.loop_match_max_iterations =
        GetInt(merged_values, "mapping.loop_match_max_iterations",
               result.mapping.loop_match_max_iterations);
    result.mapping.loop_match_correspondence_distance_m =
        GetDouble(merged_values, "mapping.loop_match_correspondence_distance_m",
                  result.mapping.loop_match_correspondence_distance_m);
    result.mapping.loop_match_min_score =
        GetDouble(merged_values, "mapping.loop_match_min_score",
                  result.mapping.loop_match_min_score);
    result.mapping.loop_match_max_points =
        GetInt(merged_values, "mapping.loop_match_max_points",
               result.mapping.loop_match_max_points);
    result.mapping.loop_correction_min_score =
        GetDouble(merged_values, "mapping.loop_correction_min_score",
                  result.mapping.loop_correction_min_score);
    result.mapping.loop_correction_max_translation_m =
        GetDouble(merged_values, "mapping.loop_correction_max_translation_m",
                  result.mapping.loop_correction_max_translation_m);
    result.mapping.loop_correction_max_yaw_rad =
        GetDouble(merged_values, "mapping.loop_correction_max_yaw_rad",
                  result.mapping.loop_correction_max_yaw_rad);
    result.mapping.loop_correction_max_consecutive_failures =
        GetInt(merged_values, "mapping.loop_correction_max_consecutive_failures",
               result.mapping.loop_correction_max_consecutive_failures);
    result.mapping.loop_correction_apply_translation_step_m =
        GetDouble(merged_values, "mapping.loop_correction_apply_translation_step_m",
                  result.mapping.loop_correction_apply_translation_step_m);
    result.mapping.loop_correction_apply_yaw_step_rad =
        GetDouble(merged_values, "mapping.loop_correction_apply_yaw_step_rad",
                  result.mapping.loop_correction_apply_yaw_step_rad);
    result.mapping.validation_min_global_points =
        GetInt(merged_values, "mapping.validation_min_global_points",
               result.mapping.validation_min_global_points);
    result.mapping.validation_min_occupied_cells =
        GetInt(merged_values, "mapping.validation_min_occupied_cells",
               result.mapping.validation_min_occupied_cells);
    result.mapping.validation_min_width =
        GetInt(merged_values, "mapping.validation_min_width",
               result.mapping.validation_min_width);
    result.mapping.validation_min_height =
        GetInt(merged_values, "mapping.validation_min_height",
               result.mapping.validation_min_height);
    result.mapping.validation_min_occupied_ratio =
        GetDouble(merged_values, "mapping.validation_min_occupied_ratio",
                  result.mapping.validation_min_occupied_ratio);
    result.mapping.validation_max_loop_translation_regression_m =
        GetDouble(merged_values, "mapping.validation_max_loop_translation_regression_m",
                  result.mapping.validation_max_loop_translation_regression_m);
    result.mapping.validation_max_loop_yaw_regression_rad =
        GetDouble(merged_values, "mapping.validation_max_loop_yaw_regression_rad",
                  result.mapping.validation_max_loop_yaw_regression_rad);

    result.planner.loop_hz =
        GetInt(merged_values, "planner.loop_hz", result.planner.loop_hz);
    result.planner.supply_start_x_m =
        GetDouble(merged_values, "planner.supply_start_x_m",
                  result.planner.supply_start_x_m);
    result.planner.supply_start_y_m =
        GetDouble(merged_values, "planner.supply_start_y_m",
                  result.planner.supply_start_y_m);
    result.planner.center_goal_x_m =
        GetDouble(merged_values, "planner.center_goal_x_m",
                  result.planner.center_goal_x_m);
    result.planner.center_goal_y_m =
        GetDouble(merged_values, "planner.center_goal_y_m",
                  result.planner.center_goal_y_m);
    result.planner.center_radius_m =
        GetDouble(merged_values, "planner.center_radius_m",
                  result.planner.center_radius_m);
    result.planner.yaw_align_tolerance_rad =
        GetDouble(merged_values, "planner.yaw_align_tolerance_rad",
                  result.planner.yaw_align_tolerance_rad);
    result.planner.center_hold_settle_frames =
        GetInt(merged_values, "planner.center_hold_settle_frames",
               result.planner.center_hold_settle_frames);
    result.planner.center_hold_settle_time_ms =
        GetInt(merged_values, "planner.center_hold_settle_time_ms",
               result.planner.center_hold_settle_time_ms);
    result.planner.recenter_threshold_m =
        GetDouble(merged_values, "planner.recenter_threshold_m",
                  result.planner.recenter_threshold_m);
    result.planner.max_vx_mps =
        GetDouble(merged_values, "planner.max_vx_mps", result.planner.max_vx_mps);
    result.planner.max_vy_mps =
        GetDouble(merged_values, "planner.max_vy_mps", result.planner.max_vy_mps);
    result.planner.max_wz_radps =
        GetDouble(merged_values, "planner.max_wz_radps", result.planner.max_wz_radps);
    result.planner.hold_max_v_mps =
        GetDouble(merged_values, "planner.hold_max_v_mps",
                  result.planner.hold_max_v_mps);
    result.planner.hold_max_wz_radps =
        GetDouble(merged_values, "planner.hold_max_wz_radps",
                  result.planner.hold_max_wz_radps);
    result.planner.dwa_linear_samples =
        GetInt(merged_values, "planner.dwa_linear_samples",
               result.planner.dwa_linear_samples);
    result.planner.dwa_lateral_samples =
        GetInt(merged_values, "planner.dwa_lateral_samples",
               result.planner.dwa_lateral_samples);
    result.planner.dwa_yaw_samples =
        GetInt(merged_values, "planner.dwa_yaw_samples",
               result.planner.dwa_yaw_samples);
    result.planner.dwa_horizon_s =
        GetDouble(merged_values, "planner.dwa_horizon_s", result.planner.dwa_horizon_s);
    result.planner.dwa_dt_s =
        GetDouble(merged_values, "planner.dwa_dt_s", result.planner.dwa_dt_s);
    result.planner.weight_heading =
        GetDouble(merged_values, "planner.weight_heading",
                  result.planner.weight_heading);
    result.planner.weight_clearance =
        GetDouble(merged_values, "planner.weight_clearance",
                  result.planner.weight_clearance);
    result.planner.weight_velocity =
        GetDouble(merged_values, "planner.weight_velocity",
                  result.planner.weight_velocity);
    result.planner.weight_dynamic =
        GetDouble(merged_values, "planner.weight_dynamic",
                  result.planner.weight_dynamic);
    result.planner.dynamic_inflation_m =
        GetDouble(merged_values, "planner.dynamic_inflation_m",
                  result.planner.dynamic_inflation_m);
    result.planner.dynamic_influence_distance_m =
        GetDouble(merged_values, "planner.dynamic_influence_distance_m",
                  result.planner.dynamic_influence_distance_m);
    result.planner.dynamic_emergency_distance_m =
        GetDouble(merged_values, "planner.dynamic_emergency_distance_m",
                  result.planner.dynamic_emergency_distance_m);

    result.safety.loop_hz =
        GetInt(merged_values, "safety.loop_hz", result.safety.loop_hz);
    result.safety.emergency_stop_distance_m =
        GetDouble(merged_values, "safety.emergency_stop_distance_m",
                  result.safety.emergency_stop_distance_m);
    result.safety.heartbeat_timeout_ms =
        GetInt(merged_values, "safety.heartbeat_timeout_ms",
               result.safety.heartbeat_timeout_ms);
    result.safety.deadman_timeout_ms =
        GetInt(merged_values, "safety.deadman_timeout_ms",
               result.safety.deadman_timeout_ms);
    result.safety.collision_check_lookahead_s =
        GetDouble(merged_values, "safety.collision_check_lookahead_s",
                  result.safety.collision_check_lookahead_s);
    result.safety.collision_check_dt_s =
        GetDouble(merged_values, "safety.collision_check_dt_s",
                  result.safety.collision_check_dt_s);
    result.safety.hold_timeout_ms =
        GetInt(merged_values, "safety.hold_timeout_ms",
               result.safety.hold_timeout_ms);
    result.safety.planner_fail_timeout_ms =
        GetInt(merged_values, "safety.planner_fail_timeout_ms",
               result.safety.planner_fail_timeout_ms);
    result.safety.localization_fail_timeout_ms =
        GetInt(merged_values, "safety.localization_fail_timeout_ms",
               result.safety.localization_fail_timeout_ms);
    result.safety.mission_timeout_ms =
        GetInt(merged_values, "safety.mission_timeout_ms",
               result.safety.mission_timeout_ms);
    result.safety.footprint_half_length_m =
        GetDouble(merged_values, "safety.footprint_half_length_m",
                  result.safety.footprint_half_length_m);
    result.safety.footprint_half_width_m =
        GetDouble(merged_values, "safety.footprint_half_width_m",
                  result.safety.footprint_half_width_m);
    result.safety.max_vx_mps =
        GetDouble(merged_values, "safety.max_vx_mps", result.safety.max_vx_mps);
    result.safety.max_vy_mps =
        GetDouble(merged_values, "safety.max_vy_mps", result.safety.max_vy_mps);
    result.safety.max_wz_radps =
        GetDouble(merged_values, "safety.max_wz_radps", result.safety.max_wz_radps);
    result.safety.max_delta_v_per_tick =
        GetDouble(merged_values, "safety.max_delta_v_per_tick",
                  result.safety.max_delta_v_per_tick);
    result.safety.max_delta_w_per_tick =
        GetDouble(merged_values, "safety.max_delta_w_per_tick",
                  result.safety.max_delta_w_per_tick);
    result.safety.recovery_speed_scale =
        GetDouble(merged_values, "safety.recovery_speed_scale",
                  result.safety.recovery_speed_scale);
    result.safety.recovery_yaw_scale =
        GetDouble(merged_values, "safety.recovery_yaw_scale",
                  result.safety.recovery_yaw_scale);

    result.spawn.x_m = GetDouble(merged_values, "spawn.x_m", result.spawn.x_m);
    result.spawn.y_m = GetDouble(merged_values, "spawn.y_m", result.spawn.y_m);
    result.spawn.theta_rad =
        GetDouble(merged_values, "spawn.theta_rad", result.spawn.theta_rad);

    if (result.system.bringup_mode == "lidar_view") {
      result.system.manual_mode_selector = -1;
      result.localization.enabled = false;
      result.mapping.enabled = false;
      result.debug.websocket_enabled = true;
      if (result.system.auto_shutdown_ms <= 1000) {
        result.system.auto_shutdown_ms = 15000;
      }
    }
  } catch (const std::exception&) {
    return common::Status::InvalidArgument("invalid scalar value in config");
  }

  *loaded_config = std::move(result);
  return common::Status::Ok();
}

std::string ConfigLoader::BuildSummary(const LoadedConfig& loaded_config) const {
  std::ostringstream summary;
  summary << "config_dir=" << loaded_config.config_dir
          << ", files=" << loaded_config.loaded_files.size()
          << ", log_level=" << loaded_config.system.log_level
          << ", bringup_mode=" << loaded_config.system.bringup_mode
          << ", auto_shutdown_ms=" << loaded_config.system.auto_shutdown_ms
          << ", manual_mode_selector=" << loaded_config.system.manual_mode_selector
          << ", frames=[" << loaded_config.frames.map << ", " << loaded_config.frames.odom
          << ", " << loaded_config.frames.base_link << ", "
          << loaded_config.frames.laser_link << ", " << loaded_config.frames.imu_link
          << "]"
          << ", sensors=[lidar:" << loaded_config.sensors.lidar_model
          << "/" << loaded_config.sensors.lidar_source
          << ":" << loaded_config.sensors.lidar_port
          << "@" << loaded_config.sensors.lidar_mount.x_m << ','
          << loaded_config.sensors.lidar_mount.y_m << ','
          << loaded_config.sensors.lidar_mount.z_m << ','
          << loaded_config.sensors.lidar_mount.roll_rad << ','
          << loaded_config.sensors.lidar_mount.pitch_rad << ','
          << loaded_config.sensors.lidar_mount.yaw_rad << ", imu:"
          << loaded_config.sensors.imu_model << "/" << loaded_config.sensors.imu_source << "@"
          << loaded_config.sensors.imu_mount.x_m << ','
          << loaded_config.sensors.imu_mount.y_m << ','
          << loaded_config.sensors.imu_mount.z_m << ','
          << loaded_config.sensors.imu_mount.roll_rad << ','
          << loaded_config.sensors.imu_mount.pitch_rad << ','
          << loaded_config.sensors.imu_mount.yaw_rad << "]"
          << ", comm=[stm32:" << loaded_config.comm.stm32_port << ':'
          << loaded_config.comm.stm32_baud_rate << "]"
          << ", debug=[ws:" << (loaded_config.debug.websocket_enabled ? "on" : "off")
          << ':' << loaded_config.debug.websocket_host << ':'
          << loaded_config.debug.websocket_port
          << ", scene_hz=" << loaded_config.debug.pointcloud_publish_hz
          << ", scalar_hz=" << loaded_config.debug.scalar_publish_hz << "]"
          << ", localization=[enabled="
          << (loaded_config.localization.enabled ? "true" : "false")
          << ", matcher=" << loaded_config.localization.matcher
          << ", slow_match_ms=" << loaded_config.localization.slow_match_threshold_ms << "]"
          << ", mapping=[enabled=" << (loaded_config.mapping.enabled ? "true" : "false")
          << ", loop_hz=" << loaded_config.mapping.loop_hz
          << ", output_dir=" << loaded_config.mapping.output_dir << "]"
          << ", spawn=[" << loaded_config.spawn.x_m << ", " << loaded_config.spawn.y_m
          << ", " << loaded_config.spawn.theta_rad << "]"
          << ", planner_hz=" << loaded_config.planner.loop_hz
          << ", center_goal=[" << loaded_config.planner.center_goal_x_m << ", "
          << loaded_config.planner.center_goal_y_m << "]"
          << ", dwa_weights=[heading:" << loaded_config.planner.weight_heading
          << ", clearance:" << loaded_config.planner.weight_clearance
          << ", velocity:" << loaded_config.planner.weight_velocity
          << ", dynamic:" << loaded_config.planner.weight_dynamic << "]"
          << ", safety_hz=" << loaded_config.safety.loop_hz
          << ", heartbeat_timeout_ms=" << loaded_config.safety.heartbeat_timeout_ms;
  return summary.str();
}

}  // namespace rm_nav::config
