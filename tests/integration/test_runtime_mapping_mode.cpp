#include <atomic>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>

#include "rm_nav/app/runtime.hpp"

namespace {

void WriteFile(const std::filesystem::path& path, const std::string& text) {
  std::ofstream output(path);
  output << text;
}

}  // namespace

int main() {
  const auto root = std::filesystem::current_path();
  const auto test_root = root / "logs" / "test_runtime_mapping_mode";
  const auto config_dir = test_root / "config";
  const auto output_dir = test_root / "output";
  const auto debug_dir = root / "logs" / "debug" / "foxglove";
  std::filesystem::remove_all(debug_dir);
  std::filesystem::create_directories(config_dir);
  std::filesystem::create_directories(output_dir);

  const auto map_pcd = (root / "maps" / "combat" / "active" / "global_map.pcd").string();
  const auto waypoint_path = (config_dir / "waypoints.txt").string();
  WriteFile(config_dir / "waypoints.txt", "1.0, 2.0, 0.0\n2.5, 2.2, 0.0\n4.0, 2.8, 0.0\n");

  WriteFile(config_dir / "system.yaml",
            "app_name: rm_nav_main\n"
            "version: 0.1.0\n"
            "log_level: INFO\n"
            "auto_shutdown_ms: 1500\n"
            "manual_mode_selector: 0\n"
            "threads:\n"
            "  driver_cpu: -1\n"
            "  sync_cpu: -1\n"
            "  pose_core_cpu: -1\n"
            "  perception_cpu: -1\n"
            "  planner_cpu: -1\n"
            "  safety_fsm_cpu: -1\n"
            "  debug_cpu: -1\n");
  WriteFile(config_dir / "frames.yaml",
            "map: map\n"
            "odom: odom\n"
            "base_link: base_link\n"
            "laser_link: laser_link\n"
            "imu_link: imu_link\n");
  WriteFile(config_dir / "sensors.yaml",
            "lidar:\n"
            "  enabled: true\n"
            "  model: synthetic\n"
            "  mount:\n"
            "    x_m: 0.18\n"
            "    y_m: 0.02\n"
            "    z_m: 0.12\n"
            "    roll_rad: 0.01\n"
            "    pitch_rad: -0.02\n"
            "    yaw_rad: 0.05\n"
            "imu:\n"
            "  enabled: false\n"
            "  model: disabled\n"
            "  mount:\n"
            "    x_m: -0.08\n"
            "    y_m: -0.01\n"
            "    z_m: 0.03\n"
            "    roll_rad: 0.00\n"
            "    pitch_rad: 0.00\n"
            "    yaw_rad: 0.00\n");
  WriteFile(config_dir / "comm.yaml",
            "stm32:\n"
            "  enabled: false\n"
            "  port: /dev/null\n"
            "  baud_rate: 115200\n");
  WriteFile(config_dir / "debug.yaml",
            "websocket:\n"
            "  enabled: true\n"
            "  port: 0\n"
            "  publish_hz: 5\n");
  WriteFile(config_dir / "planner.yaml",
            "planner:\n"
            "  loop_hz: 20\n"
            "  supply_start_x_m: 1.0\n"
            "  supply_start_y_m: 2.0\n"
            "  center_goal_x_m: 5.0\n"
            "  center_goal_y_m: 3.0\n"
            "  center_radius_m: 0.6\n"
            "  yaw_align_tolerance_rad: 0.12\n"
            "  recenter_threshold_m: 0.9\n"
            "  max_vx_mps: 1.0\n"
            "  max_vy_mps: 0.8\n"
            "  max_wz_radps: 1.0\n"
            "  hold_max_v_mps: 0.25\n"
            "  hold_max_wz_radps: 0.35\n"
            "  dwa_linear_samples: 5\n"
            "  dwa_lateral_samples: 5\n"
            "  dwa_yaw_samples: 5\n"
            "  dwa_horizon_s: 1.0\n"
            "  dwa_dt_s: 0.2\n"
            "  weight_heading: 2.0\n"
            "  weight_clearance: 5.0\n"
            "  weight_velocity: 0.2\n"
            "  weight_dynamic: 8.0\n"
            "  dynamic_inflation_m: 0.2\n"
            "  dynamic_influence_distance_m: 1.6\n"
            "  dynamic_emergency_distance_m: 0.2\n");
  WriteFile(config_dir / "safety.yaml",
            "safety:\n"
            "  loop_hz: 20\n"
            "  emergency_stop_distance_m: 0.4\n");
  WriteFile(config_dir / "spawn.yaml",
            "spawn:\n"
            "  x_m: 1.0\n"
            "  y_m: 2.0\n"
            "  theta_rad: 0.0\n");
  WriteFile(config_dir / "localization.yaml",
            std::string("localization:\n") +
                "  enabled: false\n"
                "  occupancy_path: occupancy.bin\n"
                "  global_map_pcd_path: " + map_pcd + "\n"
                "  map_meta_path: map_meta.json\n"
                "  matcher: icp\n"
                "  max_iterations: 12\n"
                "  correspondence_distance_m: 0.8\n"
                "  min_match_score: 0.25\n"
                "  max_position_jump_m: 1.0\n"
                "  max_yaw_jump_rad: 0.6\n"
                "  map_downsample_step: 1\n");
  WriteFile(config_dir / "mapping.yaml",
            std::string("mapping:\n") +
                "  enabled: false\n"
                "  loop_hz: 8\n"
                "  active_dir: " + output_dir.string() + "\n"
                "  waypoint_path: " + waypoint_path + "\n"
                "  route_speed_mps: 0.7\n"
                "  route_reach_tolerance_m: 0.35\n"
                "  voxel_size_m: 0.1\n"
                "  compression_interval_frames: 5\n"
                "  z_min_m: 0.2\n"
                "  z_max_m: 1.2\n"
                "  occupancy_resolution_m: 0.1\n"
                "  occupancy_padding_m: 1.0\n"
                "  synthetic_scan_radius_m: 8.0\n"
                "  synthetic_points_per_frame: 480\n"
                "  validation_min_global_points: 40\n"
                "  validation_min_occupied_cells: 10\n"
                "  validation_min_width: 8\n"
                "  validation_min_height: 8\n"
                "  validation_min_occupied_ratio: 0.0005\n");

  rm_nav::app::Runtime runtime;
  assert(runtime.Initialize(config_dir.string()).ok());
  std::atomic_bool stop{false};
  assert(runtime.Run(stop) == 0);

  assert(std::filesystem::exists(output_dir / "global_map.pcd"));
  assert(std::filesystem::exists(output_dir / "occupancy.bin"));
  assert(std::filesystem::exists(output_dir / "occupancy.png"));
  assert(std::filesystem::exists(output_dir / "map_meta.json"));
  assert(std::filesystem::exists(output_dir / "map_validation_report.json"));
  assert(std::filesystem::exists(debug_dir / "fsm_status.json"));
  assert(std::filesystem::exists(debug_dir / "fsm_event.json"));

  std::ifstream fsm_status_input(debug_dir / "fsm_status.json");
  std::string fsm_status_text((std::istreambuf_iterator<char>(fsm_status_input)),
                              std::istreambuf_iterator<char>());
  assert(!fsm_status_text.empty());
  assert(fsm_status_text.find("MODE_SAVE") != std::string::npos ||
         fsm_status_text.find("MODE_COMBAT") != std::string::npos ||
         fsm_status_text.find("GOTO_CENTER") != std::string::npos);

  std::cout << "test_runtime_mapping_mode passed\n";
  return 0;
}
