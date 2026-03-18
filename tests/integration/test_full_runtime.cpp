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
  const auto test_root = root / "logs" / "test_full_runtime";
  const auto config_dir = test_root / "config";
  const auto watchdog_file = root / "logs" / "watchdog" / "process_heartbeat.json";
  const auto runtime_state_file = root / "logs" / "crash" / "last_runtime_state.json";
  std::filesystem::create_directories(config_dir);

  const auto map_pcd = (root / "maps" / "combat" / "active" / "global_map.pcd").string();
  const auto occupancy_bin = (root / "maps" / "combat" / "active" / "occupancy.bin").string();
  const auto map_meta = (root / "maps" / "combat" / "active" / "map_meta.json").string();

  WriteFile(config_dir / "system.yaml",
            "app_name: rm_nav_main\n"
            "version: 0.1.0\n"
            "log_level: INFO\n"
            "auto_shutdown_ms: 1200\n"
            "manual_mode_selector: 1\n"
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
            std::string("lidar:\n") +
                "  enabled: true\n"
                "  source: synthetic\n"
                "  model: L1\n"
                "  port: /dev/null\n"
                "  mount:\n"
                "    x_m: 0.20\n"
                "    y_m: 0.00\n"
                "    z_m: 0.00\n"
                "    roll_rad: 0.00\n"
                "    pitch_rad: 0.00\n"
                "    yaw_rad: 0.00\n"
                "imu:\n"
                "  enabled: true\n"
                "  source: synthetic\n"
                "  model: GenericIMU\n"
                "  mount:\n"
                "    x_m: -0.10\n"
                "    y_m: 0.00\n"
                "    z_m: 0.00\n"
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
            "  enabled: false\n"
            "  host: 127.0.0.1\n"
            "  port: 8765\n"
            "  publish_hz: 5\n"
            "  scalar_publish_hz: 10\n"
            "  pointcloud_publish_hz: 2\n");
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
            "  dwa_dt_s: 0.2\n");
  WriteFile(config_dir / "safety.yaml",
            "safety:\n"
            "  loop_hz: 20\n"
            "  heartbeat_timeout_ms: 100\n"
            "  deadman_timeout_ms: 250\n"
            "  emergency_stop_distance_m: 0.4\n");
  WriteFile(config_dir / "spawn.yaml",
            "spawn:\n"
            "  x_m: 1.0\n"
            "  y_m: 2.0\n"
            "  theta_rad: 0.0\n");
  WriteFile(config_dir / "localization.yaml",
            std::string("localization:\n") +
                "  enabled: true\n"
                "  occupancy_path: " + occupancy_bin + "\n"
                "  global_map_pcd_path: " + map_pcd + "\n"
                "  map_meta_path: " + map_meta + "\n"
                "  matcher: icp\n"
                "  max_iterations: 12\n"
                "  correspondence_distance_m: 0.8\n"
                "  min_match_score: 0.25\n"
                "  max_position_jump_m: 1.0\n"
                "  max_yaw_jump_rad: 0.6\n"
                "  map_downsample_step: 1\n");
  WriteFile(config_dir / "mapping.yaml",
            "mapping:\n"
            "  enabled: false\n"
            "  loop_hz: 8\n"
            "  output_dir: logs/test_full_runtime/output\n"
            "  waypoint_path: warmup_waypoints.txt\n");

  rm_nav::app::Runtime runtime;
  assert(runtime.Initialize(config_dir.string()).ok());
  std::atomic_bool stop{false};
  assert(runtime.Run(stop) == 0);

  assert(std::filesystem::exists(watchdog_file));
  assert(std::filesystem::exists(runtime_state_file));

  std::ifstream runtime_state_input(runtime_state_file);
  std::string runtime_state_text((std::istreambuf_iterator<char>(runtime_state_input)),
                                 std::istreambuf_iterator<char>());
  assert(!runtime_state_text.empty());
  assert(runtime_state_text.find("driver_cpu_usage_milli") != std::string::npos);
  assert(runtime_state_text.find("sync_process_latency_ns") != std::string::npos);
  assert(runtime_state_text.find("planner_latency_ns") != std::string::npos);

  std::cout << "test_full_runtime passed\n";
  return 0;
}
