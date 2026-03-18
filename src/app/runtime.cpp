#include "rm_nav/app/runtime.hpp"

#include <algorithm>
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "rm_nav/common/time.hpp"
#include "rm_nav/debug/foxglove_server.hpp"
#include "rm_nav/data/tf_types.hpp"
#include "rm_nav/tf/frame_ids.hpp"
#include "rm_nav/tf/transform_query.hpp"
#include "rm_nav/utils/logger.hpp"

namespace rm_nav::app {
namespace {

std::string JoinFiles(const std::vector<std::string>& values) {
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      stream << ", ";
    }
    stream << values[index];
  }
  return stream.str();
}

std::string JoinModules(const RuntimeManifest& manifest) {
  std::ostringstream stream;
  bool first = true;
  const auto append = [&](const auto& stages) {
    for (const auto& stage : stages) {
      if (!first) {
        stream << ", ";
      }
      first = false;
      stream << stage.name;
    }
  };
  append(manifest.main_chain);
  append(manifest.debug_chain);
  return stream.str();
}

void SetCurrentThreadName(ThreadDomain domain) {
  const char* name = "rm_nav";
  switch (domain) {
    case ThreadDomain::kDriver:
      name = "rm_driver";
      break;
    case ThreadDomain::kSync:
      name = "rm_sync";
      break;
    case ThreadDomain::kPoseCore:
      name = "rm_pose";
      break;
    case ThreadDomain::kPerception:
      name = "rm_percept";
      break;
    case ThreadDomain::kPlanner:
      name = "rm_planner";
      break;
    case ThreadDomain::kSafetyFsm:
      name = "rm_safety";
      break;
    case ThreadDomain::kDebug:
      name = "rm_debug";
      break;
  }
#if defined(__linux__)
  pthread_setname_np(pthread_self(), name);
#endif
}

std::string BindCurrentThread(int cpu_id) {
  if (cpu_id < 0) {
    return "cpu=unbound";
  }
#if defined(__linux__)
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  CPU_SET(cpu_id, &cpu_set);
  const int result =
      pthread_setaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set);
  if (result != 0) {
    return "cpu=" + std::to_string(cpu_id) + " bind_failed";
  }
  return "cpu=" + std::to_string(cpu_id) + " bound";
#else
  return "cpu=" + std::to_string(cpu_id) + " unsupported";
#endif
}

std::chrono::milliseconds LoopPeriodFromHz(int hz, int fallback_ms) {
  if (hz <= 0) {
    return std::chrono::milliseconds(fallback_ms);
  }
  return std::chrono::milliseconds(1000 / hz);
}

std::filesystem::path ResolveConfigAsset(const std::string& config_dir, const std::string& path) {
  const std::filesystem::path asset_path(path);
  if (asset_path.is_absolute()) {
    return asset_path;
  }
  return (std::filesystem::path(config_dir) / asset_path).lexically_normal();
}

data::OdomState MakeSyntheticOdom(std::uint32_t frame_index, common::TimePoint stamp) {
  const float index = static_cast<float>(frame_index);
  data::OdomState odom;
  odom.stamp = stamp;
  odom.sequence = frame_index;
  odom.x_m = 1.0F + 0.08F * index + 0.15F * std::sin(index * 0.11F);
  odom.y_m = 2.0F + 0.50F * std::sin(index * 0.06F) + 0.08F * std::cos(index * 0.17F);
  odom.yaw_rad = 0.12F * std::sin(index * 0.05F) + 0.04F * std::cos(index * 0.13F);
  odom.vx_mps = 0.8F;
  odom.wz_radps = 0.02F * std::cos(index * 0.05F);
  return odom;
}

data::Pose3f MakeMapPose(common::TimePoint stamp, float x, float y, float yaw) {
  data::Pose3f pose;
  pose.stamp = stamp;
  pose.reference_frame = tf::kMapFrame;
  pose.child_frame = tf::kBaseLinkFrame;
  pose.position.x = x;
  pose.position.y = y;
  pose.rpy.z = yaw;
  pose.is_valid = true;
  return pose;
}

common::Vec2f TransformPoint2D(const data::Pose3f& transform, const common::Vec2f& point) {
  const auto transformed_pose =
      tf::Compose(transform, tf::MakeTransform(transform.child_frame, transform.child_frame,
                                              point.x, point.y, 0.0F, 0.0F, 0.0F, 0.0F,
                                              transform.stamp));
  common::Vec2f transformed;
  transformed.x = transformed_pose.position.x;
  transformed.y = transformed_pose.position.y;
  return transformed;
}

struct RgbaColor {
  float r{1.0F};
  float g{1.0F};
  float b{1.0F};
  float a{1.0F};
};

constexpr std::uint32_t kRuntimeMappingStatusChannelId = 201U;
constexpr std::uint32_t kRuntimeCurrentWaypointChannelId = 202U;
constexpr std::uint32_t kRuntimePartialMapSceneChannelId = 203U;
constexpr std::uint32_t kRuntimeFsmStatusChannelId = 204U;
constexpr std::uint32_t kRuntimeFsmEventChannelId = 205U;
constexpr std::uint32_t kRuntimePerfStatusChannelId = 206U;
constexpr std::uint32_t kRuntimeSafetyEventChannelId = 207U;
constexpr std::uint32_t kRuntimePoseChannelId = 208U;
constexpr std::uint32_t kRuntimeCmdChannelId = 209U;
constexpr std::uint32_t kRuntimeLocalCostmapSceneChannelId = 210U;
constexpr std::uint32_t kRuntimeFootprintSceneChannelId = 211U;
constexpr std::uint32_t kRuntimeLaserLinkSceneChannelId = 212U;
constexpr std::uint32_t kRuntimeBaseLinkSceneChannelId = 213U;
constexpr std::uint32_t kRuntimeCurrentScanSceneChannelId = 214U;

constexpr char kRuntimeMappingStatusSchema[] =
    R"({"type":"object","properties":{"processed_frames":{"type":"integer"},"accumulated_points":{"type":"integer"},"processing_latency_ns":{"type":"integer"},"current_waypoint_index":{"type":"integer"},"waypoint_count":{"type":"integer"}},"required":["processed_frames","accumulated_points","processing_latency_ns","current_waypoint_index","waypoint_count"]})";
constexpr char kRuntimeWaypointSchema[] =
    R"({"type":"object","properties":{"x":{"type":"number"},"y":{"type":"number"},"yaw":{"type":"number"},"index":{"type":"integer"},"total":{"type":"integer"}},"required":["x","y","yaw","index","total"]})";
constexpr char kRuntimeFsmStatusSchema[] =
    R"({"type":"object","properties":{"state":{"type":"string"},"previous_state":{"type":"string"},"last_event_code":{"type":"string"},"last_event_summary":{"type":"string"},"mapping_active":{"type":"boolean"},"waypoint_active":{"type":"boolean"},"localization_active":{"type":"boolean"},"center_hold_active":{"type":"boolean"},"recovery_active":{"type":"boolean"},"failsafe_active":{"type":"boolean"}},"required":["state","previous_state","last_event_code","last_event_summary","mapping_active","waypoint_active","localization_active","center_hold_active","recovery_active","failsafe_active"]})";
constexpr char kRuntimeFsmEventSchema[] =
    R"({"type":"object","properties":{"event_code":{"type":"string"},"summary":{"type":"string"},"stamp_ns":{"type":"integer"}},"required":["event_code","summary","stamp_ns"]})";
constexpr char kRuntimePerfStatusSchema[] =
    R"({"type":"object","properties":{"driver_cpu_usage_milli":{"type":"integer"},"sync_process_latency_ns":{"type":"integer"},"sync_preintegration_latency_ns":{"type":"integer"},"sync_deskew_latency_ns":{"type":"integer"},"localization_latency_ns":{"type":"integer"},"matcher_latency_ns":{"type":"integer"},"localization_light_mode":{"type":"boolean"},"mot_clustering_latency_ns":{"type":"integer"},"mot_total_latency_ns":{"type":"integer"},"mot_reduced_roi_active":{"type":"boolean"},"planner_latency_ns":{"type":"integer"},"safety_tick_latency_ns":{"type":"integer"},"foxglove_publish_latency_ns":{"type":"integer"},"debug_encode_latency_ns":{"type":"integer"},"debug_scene_suppressed":{"type":"boolean"}},"required":["driver_cpu_usage_milli","sync_process_latency_ns","sync_preintegration_latency_ns","sync_deskew_latency_ns","localization_latency_ns","matcher_latency_ns","localization_light_mode","mot_clustering_latency_ns","mot_total_latency_ns","mot_reduced_roi_active","planner_latency_ns","safety_tick_latency_ns","foxglove_publish_latency_ns","debug_encode_latency_ns","debug_scene_suppressed"]})";
constexpr char kRuntimeSafetyEventSchema[] =
    R"({"type":"object","properties":{"code":{"type":"string"},"severity":{"type":"string"},"message":{"type":"string"},"stamp_ns":{"type":"integer"}},"required":["code","severity","message","stamp_ns"]})";
constexpr char kRuntimePoseSchema[] =
    R"({"type":"object","properties":{"x":{"type":"number"},"y":{"type":"number"},"yaw":{"type":"number"},"valid":{"type":"boolean"}},"required":["x","y","yaw","valid"]})";
constexpr char kRuntimeCmdSchema[] =
    R"({"type":"object","properties":{"vx":{"type":"number"},"vy":{"type":"number"},"wz":{"type":"number"},"brake":{"type":"boolean"}},"required":["vx","vy","wz","brake"]})";

std::vector<debug::FoxgloveChannel> BuildRuntimeMappingFoxgloveChannels() {
  return {
      {kRuntimeMappingStatusChannelId, "/rm_nav/runtime/mapping_status", "json",
       "rm_nav.MappingStatus", "jsonschema", kRuntimeMappingStatusSchema},
      {kRuntimeCurrentWaypointChannelId, "/rm_nav/runtime/current_waypoint", "json",
       "rm_nav.MappingWaypoint", "jsonschema", kRuntimeWaypointSchema},
      {kRuntimePartialMapSceneChannelId, "/rm_nav/runtime/partial_map_scene", "json",
       "foxglove.SceneUpdate", "", ""},
      {kRuntimeFsmStatusChannelId, "/rm_nav/runtime/fsm_status", "json",
       "rm_nav.NavFsmStatus", "jsonschema", kRuntimeFsmStatusSchema},
      {kRuntimeFsmEventChannelId, "/rm_nav/runtime/fsm_event", "json",
       "rm_nav.NavFsmEvent", "jsonschema", kRuntimeFsmEventSchema},
      {kRuntimePerfStatusChannelId, "/rm_nav/runtime/perf_status", "json",
       "rm_nav.RuntimePerfStatus", "jsonschema", kRuntimePerfStatusSchema},
      {kRuntimeSafetyEventChannelId, "/rm_nav/runtime/safety_event", "json",
       "rm_nav.RuntimeSafetyEvent", "jsonschema", kRuntimeSafetyEventSchema},
      {kRuntimePoseChannelId, "/rm_nav/runtime/pose", "json", "rm_nav.RuntimePose",
       "jsonschema", kRuntimePoseSchema},
      {kRuntimeCmdChannelId, "/rm_nav/runtime/cmd", "json", "rm_nav.RuntimeCmd",
       "jsonschema", kRuntimeCmdSchema},
      {kRuntimeLocalCostmapSceneChannelId, "/rm_nav/runtime/local_costmap_scene", "json",
       "foxglove.SceneUpdate", "", ""},
      {kRuntimeFootprintSceneChannelId, "/rm_nav/runtime/footprint_scene", "json",
       "foxglove.SceneUpdate", "", ""},
      {kRuntimeLaserLinkSceneChannelId, "/rm_nav/runtime/laser_link_scene", "json",
       "foxglove.SceneUpdate", "", ""},
      {kRuntimeBaseLinkSceneChannelId, "/rm_nav/runtime/base_link_scene", "json",
       "foxglove.SceneUpdate", "", ""},
      {kRuntimeCurrentScanSceneChannelId, "/rm_nav/runtime/current_scan_scene", "json",
       "foxglove.SceneUpdate", "", ""},
  };
}

float ClampAbs(float value, float limit) {
  return std::max(-limit, std::min(limit, value));
}

common::TimeNs ThreadCpuTimeNs() {
#if defined(__linux__)
  timespec ts{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0) {
    return static_cast<common::TimeNs>(ts.tv_sec) * common::kNanosecondsPerSecond + ts.tv_nsec;
  }
#endif
  return 0;
}

float NormalizeAngle(float angle) {
  constexpr float kPi = 3.14159265358979323846F;
  while (angle > kPi) {
    angle -= 2.0F * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0F * kPi;
  }
  return angle;
}

bool CostmapCollisionAtLocal(const data::GridMap2D& costmap, float local_x, float local_y) {
  if (costmap.width == 0U || costmap.height == 0U || costmap.occupancy.empty() ||
      costmap.resolution_m <= 0.0F) {
    return false;
  }
  const int center_x = static_cast<int>(costmap.width / 2U);
  const int center_y = static_cast<int>(costmap.height / 2U);
  const int gx = center_x + static_cast<int>(std::round(local_x / costmap.resolution_m));
  const int gy = center_y + static_cast<int>(std::round(local_y / costmap.resolution_m));
  if (gx < 0 || gy < 0 || gx >= static_cast<int>(costmap.width) ||
      gy >= static_cast<int>(costmap.height)) {
    return true;
  }
  return costmap.occupancy[static_cast<std::size_t>(gy) * costmap.width +
                           static_cast<std::size_t>(gx)] >= 50U;
}

bool DynamicCollisionAhead(const data::Pose3f& pose, const data::ChassisCmd& cmd,
                           const std::vector<data::DynamicObstacle>& obstacles,
                           const config::SafetyConfig& config) {
  float local_x = 0.0F;
  float local_y = 0.0F;
  float local_yaw = 0.0F;
  const float lookahead_s = static_cast<float>(config.collision_check_lookahead_s);
  const float dt_s = std::max(0.02F, static_cast<float>(config.collision_check_dt_s));
  for (float t = dt_s; t <= lookahead_s + 1.0e-5F; t += dt_s) {
    local_x += (cmd.vx_mps * std::cos(local_yaw) - cmd.vy_mps * std::sin(local_yaw)) * dt_s;
    local_y += (cmd.vx_mps * std::sin(local_yaw) + cmd.vy_mps * std::cos(local_yaw)) * dt_s;
    local_yaw = NormalizeAngle(local_yaw + cmd.wz_radps * dt_s);
    const float world_x =
        pose.position.x + std::cos(pose.rpy.z) * local_x - std::sin(pose.rpy.z) * local_y;
    const float world_y =
        pose.position.y + std::sin(pose.rpy.z) * local_x + std::cos(pose.rpy.z) * local_y;
    for (const auto& obstacle : obstacles) {
      const auto& predicted_pose =
          t <= 0.5F ? obstacle.predicted_pose_05s : obstacle.predicted_pose_10s;
      const float dx = predicted_pose.position.x - world_x;
      const float dy = predicted_pose.position.y - world_y;
      const float clearance = std::sqrt(dx * dx + dy * dy) - obstacle.radius_m;
      if (clearance <= static_cast<float>(config.emergency_stop_distance_m)) {
        return true;
      }
    }
  }
  return false;
}

bool PlannerCmdTimedOut(const data::ChassisCmd& cmd, common::TimePoint now,
                        const config::SafetyConfig& config) {
  if (cmd.stamp == common::TimePoint{}) {
    return true;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - cmd.stamp).count() >
         config.deadman_timeout_ms;
}

enum class CommandCollisionType {
  kNone = 0,
  kStatic,
  kDynamic,
};

CommandCollisionType DetectCommandCollision(const data::Pose3f& pose,
                                           const data::GridMap2D& costmap,
                                           const std::vector<data::DynamicObstacle>& obstacles,
                                           const data::ChassisCmd& cmd,
                                           const config::SafetyConfig& config) {
  float local_x = 0.0F;
  float local_y = 0.0F;
  float local_yaw = 0.0F;
  const float lookahead_s = static_cast<float>(config.collision_check_lookahead_s);
  const float dt_s = std::max(0.02F, static_cast<float>(config.collision_check_dt_s));
  for (float t = dt_s; t <= lookahead_s + 1.0e-5F; t += dt_s) {
    local_x += (cmd.vx_mps * std::cos(local_yaw) - cmd.vy_mps * std::sin(local_yaw)) * dt_s;
    local_y += (cmd.vx_mps * std::sin(local_yaw) + cmd.vy_mps * std::cos(local_yaw)) * dt_s;
    local_yaw = NormalizeAngle(local_yaw + cmd.wz_radps * dt_s);
    if (CostmapCollisionAtLocal(costmap, local_x, local_y)) {
      return CommandCollisionType::kStatic;
    }
  }
  if (DynamicCollisionAhead(pose, cmd, obstacles, config)) {
    return CommandCollisionType::kDynamic;
  }
  return CommandCollisionType::kNone;
}

data::ChassisCmd ClampCommandForMode(const data::ChassisCmd& input,
                                     const fsm::NavFsmSnapshot& snapshot,
                                     const config::LoadedConfig& config,
                                     common::TimePoint stamp) {
  data::ChassisCmd cmd = input;
  cmd.stamp = stamp;
  const bool hold_mode = snapshot.center_hold_active;
  const float max_vx = static_cast<float>(hold_mode ? config.planner.hold_max_v_mps
                                                    : config.planner.max_vx_mps);
  const float max_vy = static_cast<float>(hold_mode ? config.planner.hold_max_v_mps
                                                    : config.planner.max_vy_mps);
  const float max_wz = static_cast<float>(hold_mode ? config.planner.hold_max_wz_radps
                                                    : config.planner.max_wz_radps);
  cmd.vx_mps = ClampAbs(cmd.vx_mps, max_vx);
  cmd.vy_mps = ClampAbs(cmd.vy_mps, max_vy);
  cmd.wz_radps = ClampAbs(cmd.wz_radps, max_wz);
  return cmd;
}

data::ChassisCmd BuildWaypointCmd(const data::Pose3f& pose, const data::Pose3f& goal,
                                  const config::LoadedConfig& config) {
  data::ChassisCmd cmd;
  cmd.stamp = pose.stamp;
  if (!pose.is_valid || !goal.is_valid) {
    cmd.brake = true;
    return cmd;
  }

  const float dx_world = goal.position.x - pose.position.x;
  const float dy_world = goal.position.y - pose.position.y;
  const float distance = std::sqrt(dx_world * dx_world + dy_world * dy_world);
  if (distance < 0.05F) {
    cmd.brake = true;
    return cmd;
  }

  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  const float dx_body = cos_yaw * dx_world + sin_yaw * dy_world;
  const float dy_body = -sin_yaw * dx_world + cos_yaw * dy_world;
  const float speed_limit = static_cast<float>(std::max(
      0.1, std::min(config.mapping.route_speed_mps,
                    std::max(config.planner.max_vx_mps, config.planner.max_vy_mps))));
  cmd.vx_mps = ClampAbs(dx_body * 0.8F, speed_limit);
  cmd.vy_mps = ClampAbs(dy_body * 0.8F, speed_limit);
  cmd.wz_radps = ClampAbs(NormalizeAngle(goal.rpy.z - pose.rpy.z) * 0.8F,
                          static_cast<float>(std::max(0.3, config.planner.max_wz_radps)));
  cmd.brake = false;
  return cmd;
}

data::Pose3f IntegratePose(const data::Pose3f& pose, const data::ChassisCmd& cmd, double dt_s) {
  if (!pose.is_valid) {
    return pose;
  }
  data::Pose3f next = pose;
  const float dt = static_cast<float>(dt_s);
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  next.position.x += (cmd.vx_mps * cos_yaw - cmd.vy_mps * sin_yaw) * dt;
  next.position.y += (cmd.vx_mps * sin_yaw + cmd.vy_mps * cos_yaw) * dt;
  next.rpy.z = NormalizeAngle(pose.rpy.z + cmd.wz_radps * dt);
  next.stamp = common::Now();
  return next;
}

std::string BuildScalarPoseJson(const data::Pose3f& pose, std::size_t index, std::size_t total) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"x\": " << pose.position.x << ",\n";
  output << "  \"y\": " << pose.position.y << ",\n";
  output << "  \"yaw\": " << pose.rpy.z << ",\n";
  output << "  \"index\": " << index << ",\n";
  output << "  \"total\": " << total << "\n";
  output << "}\n";
  return output.str();
}

std::string BuildRuntimeMappingStatusJson(const mapping::MappingEngine& mapping_engine,
                                          const mapping::WaypointManager& waypoint_manager) {
  const auto result = mapping_engine.LatestResult();
  std::ostringstream output;
  output << "{\n";
  output << "  \"processed_frames\": " << result.processed_frames << ",\n";
  output << "  \"accumulated_points\": " << result.accumulated_points << ",\n";
  output << "  \"processing_latency_ns\": " << result.processing_latency_ns << ",\n";
  output << "  \"current_waypoint_index\": " << waypoint_manager.current_index() << ",\n";
  output << "  \"waypoint_count\": " << waypoint_manager.waypoint_count() << "\n";
  output << "}\n";
  return output.str();
}

std::string BuildFsmStatusJson(const fsm::NavFsmSnapshot& snapshot) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"state\": \"" << fsm::ToString(snapshot.state) << "\",\n";
  output << "  \"previous_state\": \"" << fsm::ToString(snapshot.previous_state) << "\",\n";
  output << "  \"last_event_code\": \"" << fsm::ToString(snapshot.last_event.code) << "\",\n";
  output << "  \"last_event_summary\": \"" << snapshot.last_event.summary << "\",\n";
  output << "  \"mapping_active\": " << (snapshot.mapping_active ? "true" : "false") << ",\n";
  output << "  \"waypoint_active\": " << (snapshot.waypoint_active ? "true" : "false")
         << ",\n";
  output << "  \"localization_active\": "
         << (snapshot.localization_active ? "true" : "false") << ",\n";
  output << "  \"center_hold_active\": "
         << (snapshot.center_hold_active ? "true" : "false") << ",\n";
  output << "  \"recovery_active\": " << (snapshot.recovery_active ? "true" : "false")
         << ",\n";
  output << "  \"failsafe_active\": " << (snapshot.failsafe_active ? "true" : "false")
         << "\n";
  output << "}\n";
  return output.str();
}

std::string BuildFsmEventJson(const fsm::NavEvent& event) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"event_code\": \"" << fsm::ToString(event.code) << "\",\n";
  output << "  \"summary\": \"" << event.summary << "\",\n";
  output << "  \"stamp_ns\": " << common::ToNanoseconds(event.stamp) << "\n";
  output << "}\n";
  return output.str();
}

std::string SafetyEventCodeString(data::SafetyEventCode code) {
  switch (code) {
    case data::SafetyEventCode::kNone:
      return "NONE";
    case data::SafetyEventCode::kSensorTimeout:
      return "SENSOR_TIMEOUT";
    case data::SafetyEventCode::kDeadmanTimeout:
      return "DEADMAN_TIMEOUT";
    case data::SafetyEventCode::kPoseLost:
      return "POSE_LOST";
    case data::SafetyEventCode::kPlannerStall:
      return "PLANNER_STALL";
    case data::SafetyEventCode::kObstacleTooClose:
      return "OBSTACLE_TOO_CLOSE";
    case data::SafetyEventCode::kStaticCollision:
      return "STATIC_COLLISION";
    case data::SafetyEventCode::kDynamicCollision:
      return "DYNAMIC_COLLISION";
    case data::SafetyEventCode::kEmergencyStop:
      return "EMERGENCY_STOP";
    case data::SafetyEventCode::kFailsafeOverride:
      return "FAILSAFE_OVERRIDE";
  }
  return "UNKNOWN";
}

std::string SafetySeverityString(data::SafetySeverity severity) {
  switch (severity) {
    case data::SafetySeverity::kInfo:
      return "INFO";
    case data::SafetySeverity::kWarning:
      return "WARNING";
    case data::SafetySeverity::kCritical:
      return "CRITICAL";
  }
  return "UNKNOWN";
}

std::string BuildSafetyEventJson(const data::SafetyEvent& event) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"code\": \"" << SafetyEventCodeString(event.code) << "\",\n";
  output << "  \"severity\": \"" << SafetySeverityString(event.severity) << "\",\n";
  output << "  \"message\": \"" << event.message << "\",\n";
  output << "  \"stamp_ns\": " << common::ToNanoseconds(event.stamp) << "\n";
  output << "}\n";
  return output.str();
}

std::string BuildPerfStatusJson(common::TimeNs driver_cpu_usage_milli,
                                common::TimeNs sync_process_latency_ns,
                                common::TimeNs sync_preintegration_latency_ns,
                                common::TimeNs sync_deskew_latency_ns,
                                common::TimeNs localization_latency_ns,
                                common::TimeNs matcher_latency_ns,
                                bool localization_light_mode,
                                common::TimeNs mot_clustering_latency_ns,
                                common::TimeNs mot_total_latency_ns,
                                bool mot_reduced_roi_active,
                                common::TimeNs planner_latency_ns,
                                common::TimeNs safety_tick_latency_ns,
                                common::TimeNs foxglove_publish_latency_ns,
                                common::TimeNs debug_encode_latency_ns,
                                bool debug_scene_suppressed) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"driver_cpu_usage_milli\": " << driver_cpu_usage_milli << ",\n";
  output << "  \"sync_process_latency_ns\": " << sync_process_latency_ns << ",\n";
  output << "  \"sync_preintegration_latency_ns\": " << sync_preintegration_latency_ns << ",\n";
  output << "  \"sync_deskew_latency_ns\": " << sync_deskew_latency_ns << ",\n";
  output << "  \"localization_latency_ns\": " << localization_latency_ns << ",\n";
  output << "  \"matcher_latency_ns\": " << matcher_latency_ns << ",\n";
  output << "  \"localization_light_mode\": "
         << (localization_light_mode ? "true" : "false") << ",\n";
  output << "  \"mot_clustering_latency_ns\": " << mot_clustering_latency_ns << ",\n";
  output << "  \"mot_total_latency_ns\": " << mot_total_latency_ns << ",\n";
  output << "  \"mot_reduced_roi_active\": "
         << (mot_reduced_roi_active ? "true" : "false") << ",\n";
  output << "  \"planner_latency_ns\": " << planner_latency_ns << ",\n";
  output << "  \"safety_tick_latency_ns\": " << safety_tick_latency_ns << ",\n";
  output << "  \"foxglove_publish_latency_ns\": " << foxglove_publish_latency_ns << ",\n";
  output << "  \"debug_encode_latency_ns\": " << debug_encode_latency_ns << ",\n";
  output << "  \"debug_scene_suppressed\": "
         << (debug_scene_suppressed ? "true" : "false") << "\n";
  output << "}\n";
  return output.str();
}

std::string BuildPoseJson(const data::Pose3f& pose) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"x\": " << pose.position.x << ",\n";
  output << "  \"y\": " << pose.position.y << ",\n";
  output << "  \"yaw\": " << pose.rpy.z << ",\n";
  output << "  \"valid\": " << (pose.is_valid ? "true" : "false") << "\n";
  output << "}\n";
  return output.str();
}

std::string BuildCmdJson(const data::ChassisCmd& cmd) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"vx\": " << cmd.vx_mps << ",\n";
  output << "  \"vy\": " << cmd.vy_mps << ",\n";
  output << "  \"wz\": " << cmd.wz_radps << ",\n";
  output << "  \"brake\": " << (cmd.brake ? "true" : "false") << "\n";
  output << "}\n";
  return output.str();
}

std::vector<data::PointXYZI> DownsampleScenePoints(const std::vector<data::PointXYZI>& points,
                                                   std::size_t max_points) {
  if (points.size() <= max_points) {
    return points;
  }
  std::vector<data::PointXYZI> sampled;
  sampled.reserve(max_points);
  for (std::size_t index = 0; index < max_points; ++index) {
    sampled.push_back(points[(index * points.size()) / max_points]);
  }
  return sampled;
}

std::string BuildSceneUpdateJson(const std::string& entity_id, std::string_view frame_id,
                                 const std::vector<data::PointXYZI>& points,
                                 float cube_size_xy_m, float cube_size_z_m,
                                 RgbaColor color) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"deletions\": [],\n";
  output << "  \"entities\": [\n";
  output << "    {\n";
  output << "      \"id\": \"" << entity_id << "\",\n";
  output << "      \"frame_id\": \"" << frame_id << "\",\n";
  output << "      \"frameId\": \"" << frame_id << "\",\n";
  output << "      \"frame_locked\": false,\n";
  output << "      \"frameLocked\": false,\n";
  output << "      \"cubes\": [\n";
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto& point = points[index];
    output << "        {\"pose\": {\"position\": {\"x\": " << point.x << ", \"y\": " << point.y
           << ", \"z\": " << point.z
           << "}, \"orientation\": {\"x\": 0, \"y\": 0, \"z\": 0, \"w\": 1}}, "
           << "\"size\": {\"x\": " << cube_size_xy_m << ", \"y\": " << cube_size_xy_m
           << ", \"z\": " << cube_size_z_m << "}, "
           << "\"color\": {\"r\": " << color.r << ", \"g\": " << color.g << ", \"b\": "
           << color.b << ", \"a\": " << color.a << "}}";
    if (index + 1U != points.size()) {
      output << ",";
    }
    output << "\n";
  }
  output << "      ]\n";
  output << "    }\n";
  output << "  ]\n";
  output << "}\n";
  return output.str();
}

std::vector<data::PointXYZI> CostmapOccupiedPoints(const data::GridMap2D& costmap,
                                                   std::size_t max_points) {
  std::vector<data::PointXYZI> points;
  if (costmap.width == 0U || costmap.height == 0U || costmap.occupancy.empty()) {
    return points;
  }

  const int center_x = static_cast<int>(costmap.width / 2U);
  const int center_y = static_cast<int>(costmap.height / 2U);
  const float cos_yaw = std::cos(costmap.origin.rpy.z);
  const float sin_yaw = std::sin(costmap.origin.rpy.z);
  points.reserve(std::min<std::size_t>(max_points, costmap.occupancy.size()));

  for (std::uint32_t gy = 0; gy < costmap.height; ++gy) {
    for (std::uint32_t gx = 0; gx < costmap.width; ++gx) {
      const auto index = static_cast<std::size_t>(gy) * costmap.width + gx;
      if (index >= costmap.occupancy.size() || costmap.occupancy[index] == 0U) {
        continue;
      }
      const float local_x =
          (static_cast<float>(static_cast<int>(gx) - center_x) + 0.5F) * costmap.resolution_m;
      const float local_y =
          (static_cast<float>(static_cast<int>(gy) - center_y) + 0.5F) * costmap.resolution_m;
      data::PointXYZI point;
      point.x = costmap.origin.position.x + cos_yaw * local_x - sin_yaw * local_y;
      point.y = costmap.origin.position.y + sin_yaw * local_x + cos_yaw * local_y;
      point.z = costmap.origin.position.z;
      point.intensity = static_cast<float>(costmap.occupancy[index]);
      points.push_back(point);
      if (points.size() >= max_points) {
        return points;
      }
    }
  }
  return points;
}

std::vector<data::PointXYZI> FootprintOutlinePoints(const config::RectMaskConfig& mask,
                                                    const data::Pose3f& pose,
                                                    std::size_t samples_per_edge) {
  std::vector<data::PointXYZI> points;
  if (!mask.enabled || !pose.is_valid || samples_per_edge == 0U) {
    return points;
  }

  const std::array<common::Vec2f, 4> corners = {
      common::Vec2f{mask.x_min_m, mask.y_min_m},
      common::Vec2f{mask.x_max_m, mask.y_min_m},
      common::Vec2f{mask.x_max_m, mask.y_max_m},
      common::Vec2f{mask.x_min_m, mask.y_max_m},
  };
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  points.reserve(samples_per_edge * 4U);

  for (std::size_t edge = 0; edge < corners.size(); ++edge) {
    const auto& start = corners[edge];
    const auto& end = corners[(edge + 1U) % corners.size()];
    for (std::size_t index = 0; index < samples_per_edge; ++index) {
      const float alpha = samples_per_edge == 1U
                              ? 0.0F
                              : static_cast<float>(index) /
                                    static_cast<float>(samples_per_edge - 1U);
      const float local_x = start.x + (end.x - start.x) * alpha;
      const float local_y = start.y + (end.y - start.y) * alpha;
      data::PointXYZI point;
      point.x = pose.position.x + cos_yaw * local_x - sin_yaw * local_y;
      point.y = pose.position.y + sin_yaw * local_x + cos_yaw * local_y;
      point.z = pose.position.z + 0.05F;
      point.intensity = 1.0F;
      points.push_back(point);
    }
  }
  return points;
}

std::vector<data::PointXYZI> LaserLinkMarkerPoints(const config::ExtrinsicConfig& mount,
                                                   const data::Pose3f& base_pose) {
  std::vector<data::PointXYZI> points;
  if (!base_pose.is_valid) {
    return points;
  }

  const auto base_to_laser =
      tf::MakeTransform(tf::kBaseLinkFrame, tf::kLaserFrame, mount.x_m, mount.y_m, mount.z_m,
                        mount.roll_rad, mount.pitch_rad, mount.yaw_rad, base_pose.stamp);
  const auto map_to_laser = tf::Compose(base_pose, base_to_laser);

  data::PointXYZI center;
  center.x = map_to_laser.position.x;
  center.y = map_to_laser.position.y;
  center.z = map_to_laser.position.z + 0.08F;
  center.intensity = 1.0F;
  points.push_back(center);

  const float axis_len = 0.12F;
  const float cos_yaw = std::cos(map_to_laser.rpy.z);
  const float sin_yaw = std::sin(map_to_laser.rpy.z);
  data::PointXYZI forward = center;
  forward.x += axis_len * cos_yaw;
  forward.y += axis_len * sin_yaw;
  points.push_back(forward);

  data::PointXYZI left = center;
  left.x += axis_len * -sin_yaw;
  left.y += axis_len * cos_yaw;
  points.push_back(left);
  return points;
}

std::vector<data::PointXYZI> BaseLinkMarkerPoints(const data::Pose3f& base_pose) {
  std::vector<data::PointXYZI> points;
  if (!base_pose.is_valid) {
    return points;
  }

  data::PointXYZI center;
  center.x = base_pose.position.x;
  center.y = base_pose.position.y;
  center.z = base_pose.position.z + 0.06F;
  center.intensity = 1.0F;
  points.push_back(center);

  const float axis_len = 0.16F;
  const float cos_yaw = std::cos(base_pose.rpy.z);
  const float sin_yaw = std::sin(base_pose.rpy.z);
  data::PointXYZI forward = center;
  forward.x += axis_len * cos_yaw;
  forward.y += axis_len * sin_yaw;
  points.push_back(forward);

  data::PointXYZI left = center;
  left.x += axis_len * -sin_yaw;
  left.y += axis_len * cos_yaw;
  points.push_back(left);
  return points;
}

data::PointXYZI TransformPointToMap(const data::PointXYZI& point,
                                    const data::Pose3f& map_to_sensor) {
  const float cr = std::cos(map_to_sensor.rpy.x);
  const float sr = std::sin(map_to_sensor.rpy.x);
  const float cp = std::cos(map_to_sensor.rpy.y);
  const float sp = std::sin(map_to_sensor.rpy.y);
  const float cy = std::cos(map_to_sensor.rpy.z);
  const float sy = std::sin(map_to_sensor.rpy.z);

  data::PointXYZI transformed = point;
  transformed.x = map_to_sensor.position.x +
                  (cy * cp) * point.x + (cy * sp * sr - sy * cr) * point.y +
                  (cy * sp * cr + sy * sr) * point.z;
  transformed.y = map_to_sensor.position.y +
                  (sy * cp) * point.x + (sy * sp * sr + cy * cr) * point.y +
                  (sy * sp * cr - cy * sr) * point.z;
  transformed.z = map_to_sensor.position.z + (-sp) * point.x + (cp * sr) * point.y +
                  (cp * cr) * point.z;
  return transformed;
}

std::vector<data::PointXYZI> CurrentScanScenePoints(const data::LidarFrame& frame,
                                                    const data::Pose3f& base_pose,
                                                    const config::ExtrinsicConfig& mount,
                                                    std::size_t max_points) {
  std::vector<data::PointXYZI> points;
  if (!base_pose.is_valid || frame.points.empty()) {
    return points;
  }

  const auto base_to_laser =
      tf::MakeTransform(tf::kBaseLinkFrame, tf::kLaserFrame, mount.x_m, mount.y_m, mount.z_m,
                        mount.roll_rad, mount.pitch_rad, mount.yaw_rad, base_pose.stamp);
  const auto map_to_laser = tf::Compose(base_pose, base_to_laser);
  const auto sampled = DownsampleScenePoints(frame.points, max_points);
  points.reserve(sampled.size());
  for (const auto& point : sampled) {
    points.push_back(TransformPointToMap(point, map_to_laser));
  }
  return points;
}

void PublishFoxgloveJson(debug::FoxgloveServer* server, std::uint32_t channel_id,
                         const std::string& payload, common::TimePoint stamp) {
  if (server == nullptr || !server->is_running()) {
    return;
  }
  server->PublishJson(channel_id, payload, stamp);
}

void WriteDebugSnapshot(const localization::LocalizationEngine& localization,
                        const planning::PlannerCoordinator& planner,
                        const std::filesystem::path& output_dir) {
  std::filesystem::create_directories(output_dir);

  const auto result = localization.LatestResult();
  const auto scan = localization.LatestAlignedScan();
  const auto& map = localization.static_map();
  const auto path = planner.LatestPath();
  const auto planner_status = planner.LatestStatus();
  const auto cmd = planner.LatestCmd();

  {
    std::ofstream output(output_dir / "pose.json");
    output << "{\n";
    output << "  \"x\": " << result.map_to_base.position.x << ",\n";
    output << "  \"y\": " << result.map_to_base.position.y << ",\n";
    output << "  \"yaw\": " << result.map_to_base.rpy.z << ",\n";
    output << "  \"trusted\": " << (result.status.pose_trusted ? "true" : "false") << "\n";
    output << "}\n";
  }
  {
    std::ofstream output(output_dir / "map_to_odom_status.json");
    output << "{\n";
    output << "  \"score\": " << result.status.match_score << ",\n";
    output << "  \"iterations\": " << result.status.iterations << ",\n";
    output << "  \"converged\": " << (result.status.converged ? "true" : "false") << ",\n";
    output << "  \"failures\": " << result.status.consecutive_failures << ",\n";
    output << "  \"jump_m\": " << result.status.pose_jump_m << ",\n";
    output << "  \"jump_yaw_rad\": " << result.status.yaw_jump_rad << "\n";
    output << "}\n";
  }
  {
    std::ofstream output(output_dir / "planner_status.json");
    output << "{\n";
    output << "  \"mode\": \""
           << (planner_status.mode == planning::GoalMode::kCenterHold ? "center_hold"
                                                                      : "approach_center")
           << "\",\n";
    output << "  \"distance_to_goal_m\": " << planner_status.distance_to_goal_m << ",\n";
    output << "  \"distance_to_center_m\": " << planner_status.distance_to_center_m << ",\n";
    output << "  \"yaw_error_rad\": " << planner_status.yaw_error_rad << ",\n";
    output << "  \"reached\": " << (planner_status.reached ? "true" : "false") << ",\n";
    output << "  \"cmd\": {\"vx\": " << cmd.vx_mps << ", \"vy\": " << cmd.vy_mps
           << ", \"wz\": " << cmd.wz_radps << "},\n";
    output << "  \"dwa_score\": {\n";
    output << "    \"goal\": " << planner_status.dwa_score.goal_score << ",\n";
    output << "    \"path\": " << planner_status.dwa_score.path_score << ",\n";
    output << "    \"smooth\": " << planner_status.dwa_score.smooth_score << ",\n";
    output << "    \"heading\": " << planner_status.dwa_score.heading_score << ",\n";
    output << "    \"clearance\": " << planner_status.dwa_score.clearance_score << ",\n";
    output << "    \"velocity\": " << planner_status.dwa_score.velocity_score << ",\n";
    output << "    \"dynamic\": " << planner_status.dwa_score.dynamic_risk_score << ",\n";
    output << "    \"dynamic_max_risk\": " << planner_status.dwa_score.dynamic_max_risk
           << ",\n";
    output << "    \"dynamic_integrated_risk\": "
           << planner_status.dwa_score.dynamic_integrated_risk << ",\n";
    output << "    \"dynamic_clearance_min\": "
           << planner_status.dwa_score.dynamic_clearance_min << ",\n";
    output << "    \"dynamic_risk_05\": " << planner_status.dwa_score.dynamic_risk_05
           << ",\n";
    output << "    \"dynamic_risk_10\": " << planner_status.dwa_score.dynamic_risk_10 << ",\n";
    output << "    \"total\": " << planner_status.dwa_score.total_score << "\n";
    output << "  }\n";
    output << "}\n";
  }
  {
    std::ofstream output(output_dir / "current_scan.pcd");
    output << "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n";
    output << "COUNT 1 1 1 1\nWIDTH " << scan.points.size() << "\nHEIGHT 1\n";
    output << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << scan.points.size() << "\nDATA ascii\n";
    for (const auto& point : scan.points) {
      output << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << "\n";
    }
  }
  {
    std::ofstream output(output_dir / "static_map.pcd");
    output << "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n";
    output << "COUNT 1 1 1 1\nWIDTH " << map.global_points.size() << "\nHEIGHT 1\n";
    output << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << map.global_points.size()
           << "\nDATA ascii\n";
    for (const auto& point : map.global_points) {
      output << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << "\n";
    }
  }
  {
    std::ofstream output(output_dir / "global_path.json");
    output << "{\n  \"points\": [\n";
    for (std::size_t index = 0; index < path.points.size(); ++index) {
      const auto& point = path.points[index];
      output << "    {\"x\": " << point.position.x << ", \"y\": " << point.position.y
             << ", \"heading\": " << point.heading_rad << ", \"speed\": "
             << point.target_speed_mps << "}";
      if (index + 1U != path.points.size()) {
        output << ',';
      }
      output << "\n";
    }
    output << "  ]\n}\n";
  }
}

void WriteMappingDebugSnapshot(const mapping::MappingEngine& mapping_engine,
                               const mapping::WaypointManager& waypoint_manager,
                               const data::Pose3f& pose,
                               const data::ChassisCmd& cmd,
                               const std::filesystem::path& output_dir) {
  std::filesystem::create_directories(output_dir);
  const auto global_points = mapping_engine.GlobalPointCloud();
  const auto mapping_result = mapping_engine.LatestResult();
  const auto goal = waypoint_manager.CurrentGoal();

  {
    std::ofstream output(output_dir / "pose.json");
    output << "{\n";
    output << "  \"x\": " << pose.position.x << ",\n";
    output << "  \"y\": " << pose.position.y << ",\n";
    output << "  \"yaw\": " << pose.rpy.z << "\n";
    output << "}\n";
  }
  {
    std::ofstream output(output_dir / "goal.json");
    output << "{\n";
    output << "  \"x\": " << goal.position.x << ",\n";
    output << "  \"y\": " << goal.position.y << ",\n";
    output << "  \"yaw\": " << goal.rpy.z << ",\n";
    output << "  \"waypoint_index\": " << waypoint_manager.current_index() << "\n";
    output << "}\n";
  }
  {
    std::ofstream output(output_dir / "cmd.json");
    output << "{\n";
    output << "  \"vx\": " << cmd.vx_mps << ",\n";
    output << "  \"vy\": " << cmd.vy_mps << ",\n";
    output << "  \"wz\": " << cmd.wz_radps << ",\n";
    output << "  \"brake\": " << (cmd.brake ? "true" : "false") << "\n";
    output << "}\n";
  }
  {
    std::ofstream output(output_dir / "mapping_status.json");
    output << "{\n";
    output << "  \"processed_frames\": " << mapping_result.processed_frames << ",\n";
    output << "  \"accumulated_points\": " << mapping_result.accumulated_points << ",\n";
    output << "  \"processing_latency_ns\": " << mapping_result.processing_latency_ns << "\n";
    output << "}\n";
  }
  {
    std::ofstream output(output_dir / "global_map.pcd");
    output << "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n";
    output << "COUNT 1 1 1 1\nWIDTH " << global_points.size() << "\nHEIGHT 1\n";
    output << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << global_points.size() << "\nDATA ascii\n";
    for (const auto& point : global_points) {
      output << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << "\n";
    }
  }
}

void WriteFsmDebugSnapshot(const fsm::NavFsmSnapshot& snapshot,
                           const std::filesystem::path& output_dir) {
  std::filesystem::create_directories(output_dir);
  {
    std::ofstream output(output_dir / "fsm_status.json");
    output << BuildFsmStatusJson(snapshot);
  }
  {
    std::ofstream output(output_dir / "fsm_event.json");
    output << BuildFsmEventJson(snapshot.last_event);
  }
}

void WriteSafetyEventDebugSnapshot(const data::SafetyEvent& event,
                                   const std::filesystem::path& output_dir) {
  std::filesystem::create_directories(output_dir);
  std::ofstream output(output_dir / "safety_event.json");
  output << BuildSafetyEventJson(event);
}

void WritePerfDebugSnapshot(const std::string& perf_json,
                            const std::filesystem::path& output_dir) {
  std::filesystem::create_directories(output_dir);
  std::filesystem::create_directories("logs/crash");
  {
    std::ofstream output(output_dir / "perf_status.json");
    output << perf_json;
  }
  {
    std::ofstream output("logs/crash/last_runtime_state.json");
    output << perf_json;
  }
}

}  // namespace

Runtime::Runtime() : manifest_(BuildPhase0Manifest()) {}

Runtime::~Runtime() { StopThreads(); }

common::Status Runtime::Initialize(const std::string& config_dir) {
  auto status = ValidatePhase0Contract();
  if (!status.ok()) {
    return status;
  }

  config::ConfigLoader loader;
  status = loader.LoadFromDirectory(config_dir, &loaded_config_);
  if (!status.ok()) {
    return status;
  }

  utils::Logger::Instance().Initialize(
      {utils::ParseLogLevel(loaded_config_.system.log_level)});

  status = ValidateLoadedConfig();
  if (!status.ok()) {
    return status;
  }

  status = ConfigurePipeline();
  if (!status.ok()) {
    return status;
  }

  LogStartupSummary();

  status = StartThreads();
  if (!status.ok()) {
    StopThreads();
    return status;
  }

  initialized_ = true;
  return common::Status::Ok();
}

int Runtime::Run(const std::atomic_bool& external_stop) {
  if (!initialized_) {
    return 1;
  }

  const auto start = common::Now();
  bool waiting_for_mapping_save = false;
  auto mapping_save_deadline = start;
  bool post_save_grace_started = false;
  auto post_save_deadline = start;
  while (!stop_requested_.load() && !external_stop.load()) {
    if (loaded_config_.system.auto_shutdown_ms > 0) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          common::Now() - start);
      if (elapsed.count() >= loaded_config_.system.auto_shutdown_ms) {
        if (waiting_for_mapping_save || post_save_grace_started) {
          if (map_saved_.load(std::memory_order_acquire)) {
            if (!post_save_grace_started) {
              post_save_grace_started = true;
              post_save_deadline = common::Now() + std::chrono::milliseconds(300);
            } else if (common::Now() >= post_save_deadline) {
              utils::LogInfo("runtime", "mapping save completed, stopping runtime");
              break;
            }
          } else if (common::Now() >= mapping_save_deadline) {
            utils::LogWarn("runtime", "mapping save wait timeout reached");
            break;
          }
        } else if (mapping_mode_ && !map_saved_.load(std::memory_order_acquire)) {
          if (!waiting_for_mapping_save) {
            utils::LogInfo("runtime", "auto shutdown reached, requesting MODE_SAVE");
            mapping_save_requested_.store(true, std::memory_order_release);
            waiting_for_mapping_save = true;
            mapping_save_deadline = common::Now() + std::chrono::milliseconds(1500);
          }
        } else {
          utils::LogInfo("runtime", "auto shutdown timeout reached");
          break;
        }
      }
    }
    common::SleepFor(std::chrono::milliseconds(50));
  }

  if ((external_stop.load() || stop_requested_.load(std::memory_order_acquire)) && mapping_mode_ &&
      !map_saved_.load(std::memory_order_acquire)) {
    mapping_save_requested_.store(true, std::memory_order_release);
    const auto deadline = common::Now() + std::chrono::milliseconds(500);
    while (common::Now() < deadline && !map_saved_.load(std::memory_order_acquire)) {
      common::SleepFor(std::chrono::milliseconds(20));
    }
  }

  StopThreads();
  initialized_ = false;
  return 0;
}

void Runtime::RequestStop() { stop_requested_.store(true); }

common::Status Runtime::ValidatePhase0Contract() const {
  if (manifest_.frames.size() != 5U) {
    return common::Status::InternalError("phase0 frame contract mismatch");
  }
  if (manifest_.main_chain.size() != 8U) {
    return common::Status::InternalError("phase0 main chain contract mismatch");
  }
  if (manifest_.debug_chain.size() != 3U) {
    return common::Status::InternalError("phase0 debug chain contract mismatch");
  }
  if (manifest_.threads.size() != kThreadBindings.size()) {
    return common::Status::InternalError("phase0 thread contract mismatch");
  }
  return common::Status::Ok();
}

common::Status Runtime::ValidateLoadedConfig() const {
  if (loaded_config_.system.bringup_mode != "none" &&
      loaded_config_.system.bringup_mode != "lidar_view") {
    return common::Status::InvalidArgument("unsupported bringup_mode");
  }
  if (loaded_config_.sensors.imu_source != "synthetic" &&
      loaded_config_.sensors.imu_source != "lidar_internal") {
    return common::Status::InvalidArgument("unsupported imu source");
  }
  if (loaded_config_.sensors.imu_source == "lidar_internal" &&
      (!loaded_config_.sensors.lidar_enabled ||
       loaded_config_.sensors.lidar_source != "unitree_sdk")) {
    return common::Status::InvalidArgument(
        "lidar_internal imu source requires unitree_sdk lidar source");
  }
  if (loaded_config_.frames.map != tf::kMapFrame ||
      loaded_config_.frames.odom != tf::kOdomFrame ||
      loaded_config_.frames.base_link != tf::kBaseLinkFrame ||
      loaded_config_.frames.laser_link != tf::kLaserFrame ||
      loaded_config_.frames.imu_link != tf::kImuFrame) {
    return common::Status::InvalidArgument(
        "frame config does not match phase0 fixed frame ids");
  }
  return common::Status::Ok();
}

common::Status Runtime::ConfigurePipeline() {
  if (pipeline_configured_.load(std::memory_order_acquire)) {
    return common::Status::Ok();
  }

  const int manual_mode_selector = loaded_config_.system.manual_mode_selector;
  mapping_mode_ = loaded_config_.mapping.enabled || manual_mode_selector == 0;
  nav_fsm_ = fsm::NavFsm();
  mapping_save_requested_.store(false, std::memory_order_release);
  map_saved_.store(false, std::memory_order_release);
  combat_pipeline_ready_.store(false, std::memory_order_release);
  last_stm32_rx_ns_.store(0, std::memory_order_release);
  latest_safety_event_.Publish({});
  fsm_snapshot_.Publish(nav_fsm_.snapshot());
  driver_cpu_usage_milli_.store(0, std::memory_order_release);
  sync_process_latency_ns_.store(0, std::memory_order_release);
  sync_preintegration_latency_ns_.store(0, std::memory_order_release);
  sync_deskew_latency_ns_.store(0, std::memory_order_release);
  localization_latency_ns_.store(0, std::memory_order_release);
  localization_matcher_latency_ns_.store(0, std::memory_order_release);
  localization_light_mode_.store(false, std::memory_order_release);
  mot_clustering_latency_ns_.store(0, std::memory_order_release);
  mot_total_latency_ns_.store(0, std::memory_order_release);
  mot_reduced_roi_active_.store(false, std::memory_order_release);
  planner_latency_ns_.store(0, std::memory_order_release);
  safety_tick_latency_ns_.store(0, std::memory_order_release);
  foxglove_publish_latency_ns_.store(0, std::memory_order_release);
  debug_encode_latency_ns_.store(0, std::memory_order_release);
  debug_scene_suppressed_.store(false, std::memory_order_release);

  sync::SyncConfig sync_config;
  sync_config.max_imu_packets_per_frame = data::SyncedFrame::kMaxImuPackets;
  auto status = sensor_sync_.Configure(sync_config);
  if (!status.ok()) {
    return status;
  }

  perception::PreprocessConfig preprocess_config;
  preprocess_config.self_mask_enabled = loaded_config_.sensors.lidar_self_mask.enabled;
  preprocess_config.self_mask_x_min_m = loaded_config_.sensors.lidar_self_mask.x_min_m;
  preprocess_config.self_mask_x_max_m = loaded_config_.sensors.lidar_self_mask.x_max_m;
  preprocess_config.self_mask_y_min_m = loaded_config_.sensors.lidar_self_mask.y_min_m;
  preprocess_config.self_mask_y_max_m = loaded_config_.sensors.lidar_self_mask.y_max_m;
  perception::LocalCostmapConfig local_costmap_config;
  status = local_costmap_builder_.Configure(local_costmap_config);
  if (!status.ok()) {
    return status;
  }
  if (mapping_mode_) {
    status = mapping_engine_.Initialize(loaded_config_.mapping);
    if (!status.ok()) {
      return status;
    }
    status = waypoint_manager_.Load(
        ResolveConfigAsset(loaded_config_.config_dir, loaded_config_.mapping.waypoint_path)
            .string());
    if (!status.ok()) {
      return status;
    }
    const auto initial_pose =
        MakeMapPose(common::Now(), static_cast<float>(loaded_config_.spawn.x_m),
                    static_cast<float>(loaded_config_.spawn.y_m),
                    static_cast<float>(loaded_config_.spawn.theta_rad));
    mapping_pose_.Publish(initial_pose);
    data::ChassisCmd mapping_cmd;
    mapping_cmd.brake = true;
    mapping_cmd_.Publish(mapping_cmd);
  }

  if (!mapping_mode_ || manual_mode_selector == 1) {
    if (mapping_mode_ && manual_mode_selector == 1) {
      const auto output_dir =
          ResolveConfigAsset(loaded_config_.config_dir, loaded_config_.mapping.output_dir);
      const bool saved_map_available =
          std::filesystem::exists(output_dir / "global_map.pcd") &&
          std::filesystem::exists(output_dir / "occupancy.bin") &&
          std::filesystem::exists(output_dir / "map_meta.json");
      if (saved_map_available) {
        status = InitializeCombatPipelineFromSavedMap();
      } else {
        status = InitializeCombatPipeline(loaded_config_.localization, loaded_config_.spawn);
      }
    } else {
      status = InitializeCombatPipeline(loaded_config_.localization, loaded_config_.spawn);
    }
    if (!status.ok()) {
      return status;
    }
  }

  status = tf_tree_.RegisterStaticTransform(
      tf::MakeTransform(tf::kBaseLinkFrame, tf::kLaserFrame,
                        loaded_config_.sensors.lidar_mount.x_m,
                        loaded_config_.sensors.lidar_mount.y_m,
                        loaded_config_.sensors.lidar_mount.z_m,
                        loaded_config_.sensors.lidar_mount.roll_rad,
                        loaded_config_.sensors.lidar_mount.pitch_rad,
                        loaded_config_.sensors.lidar_mount.yaw_rad));
  if (!status.ok()) {
    return status;
  }
  status = tf_tree_.RegisterStaticTransform(
      tf::MakeTransform(tf::kBaseLinkFrame, tf::kImuFrame,
                        loaded_config_.sensors.imu_mount.x_m,
                        loaded_config_.sensors.imu_mount.y_m,
                        loaded_config_.sensors.imu_mount.z_m,
                        loaded_config_.sensors.imu_mount.roll_rad,
                        loaded_config_.sensors.imu_mount.pitch_rad,
                        loaded_config_.sensors.imu_mount.yaw_rad));
  if (!status.ok()) {
    return status;
  }

  if (preprocess_config.self_mask_enabled) {
    const auto base_to_laser =
        tf_tree_.Lookup(tf::kBaseLinkFrame, tf::kLaserFrame, common::Now());
    if (!base_to_laser.has_value()) {
      return common::Status::InternalError("failed to lookup static base->laser transform");
    }
    const auto laser_to_base = tf::Inverse(*base_to_laser);
    const std::array<common::Vec2f, 4> base_polygon = {
        common::Vec2f{loaded_config_.sensors.lidar_self_mask.x_min_m,
                      loaded_config_.sensors.lidar_self_mask.y_min_m},
        common::Vec2f{loaded_config_.sensors.lidar_self_mask.x_max_m,
                      loaded_config_.sensors.lidar_self_mask.y_min_m},
        common::Vec2f{loaded_config_.sensors.lidar_self_mask.x_max_m,
                      loaded_config_.sensors.lidar_self_mask.y_max_m},
        common::Vec2f{loaded_config_.sensors.lidar_self_mask.x_min_m,
                      loaded_config_.sensors.lidar_self_mask.y_max_m},
    };
    for (std::size_t index = 0; index < base_polygon.size(); ++index) {
      preprocess_config.self_mask_polygon[index] =
          TransformPoint2D(laser_to_base, base_polygon[index]);
    }
    preprocess_config.self_mask_polygon_valid = true;
  }

  status = preprocess_.Configure(preprocess_config);
  if (!status.ok()) {
    return status;
  }

  if (loaded_config_.sensors.lidar_enabled) {
    drivers::lidar::L1DriverConfig lidar_config;
    lidar_config.source = loaded_config_.sensors.lidar_source;
    lidar_config.device_path = loaded_config_.sensors.lidar_port;
    lidar_config.baud_rate = loaded_config_.sensors.lidar_baud_rate;
    lidar_config.cloud_scan_num = loaded_config_.sensors.lidar_cloud_scan_num;
    lidar_config.frame_rate_hz = mapping_mode_ ? loaded_config_.mapping.loop_hz : 10.0;
    lidar_config.points_per_frame = mapping_mode_
                                        ? static_cast<std::size_t>(std::max(
                                              32, loaded_config_.mapping.synthetic_points_per_frame))
                                        : lidar_config.points_per_frame;
    lidar_config.radius_m =
        static_cast<float>(mapping_mode_ ? loaded_config_.mapping.synthetic_scan_radius_m
                                         : lidar_config.radius_m);
    if (!loaded_config_.localization.global_map_pcd_path.empty()) {
      lidar_config.synthetic_map_pcd_path =
          (std::filesystem::path(loaded_config_.config_dir) /
           loaded_config_.localization.global_map_pcd_path)
              .lexically_normal()
              .string();
    }
    status = lidar_driver_.Configure(lidar_config);
    if (!status.ok()) {
      return status;
    }
  }

  if (loaded_config_.sensors.imu_enabled) {
    if (loaded_config_.sensors.imu_source == "synthetic") {
      drivers::imu::ImuDriverConfig imu_config;
      imu_config.source = "synthetic";
      status = imu_driver_.Configure(imu_config);
      if (!status.ok()) {
        return status;
      }
    } else if (loaded_config_.sensors.imu_source != "lidar_internal") {
      return common::Status::InvalidArgument("unsupported imu source");
    }
  }

  if (loaded_config_.comm.stm32_enabled) {
    drivers::stm32::Stm32BridgeConfig stm32_config;
    stm32_config.serial.device_path = loaded_config_.comm.stm32_port;
    stm32_config.serial.baud_rate = loaded_config_.comm.stm32_baud_rate;
    stm32_config.serial.read_timeout_ms = 20;
    status = stm32_bridge_.Configure(stm32_config);
    if (status.ok()) {
      stm32_available_.store(true, std::memory_order_release);
    } else {
      stm32_available_.store(false, std::memory_order_release);
      utils::LogWarn("runtime",
                     std::string("stm32 unavailable, runtime will stay in offline mode: ") +
                         status.message);
    }
  }

  data::ChassisCmd safe_cmd;
  safe_cmd.brake = true;
  safety_cmd_.Publish(safe_cmd);

  if (loaded_config_.debug.websocket_enabled) {
    debug::FoxgloveServerOptions foxglove_options;
    foxglove_options.name = "rm_nav_runtime_mapping";
    foxglove_options.host = loaded_config_.debug.websocket_host;
    foxglove_options.port =
        static_cast<std::uint16_t>(std::max(0, loaded_config_.debug.websocket_port));
    status = foxglove_server_.Start(foxglove_options);
    if (!status.ok()) {
      utils::LogWarn("runtime",
                     std::string("failed to start foxglove runtime server: ") + status.message);
    } else {
      status = foxglove_server_.Advertise(BuildRuntimeMappingFoxgloveChannels());
      if (!status.ok()) {
        utils::LogWarn("runtime",
                       std::string("failed to advertise runtime foxglove channels: ") +
                           status.message);
        foxglove_server_.Stop();
      }
    }
  }

  pipeline_configured_.store(true, std::memory_order_release);
  return common::Status::Ok();
}

common::Status Runtime::StartThreads() {
  StopThreads();
  stop_requested_.store(false);

  for (const auto& binding : manifest_.threads) {
    WorkerThread worker;
    worker.binding = binding;
    worker.cpu_id = ResolveCpu(binding.domain);
    try {
      worker.thread = std::thread([this, binding, cpu_id = worker.cpu_id]() {
        ThreadMain(binding, cpu_id);
      });
    } catch (...) {
      return common::Status::InternalError("failed to create runtime thread");
    }
    workers_.push_back(std::move(worker));
  }

  return common::Status::Ok();
}

void Runtime::StopThreads() {
  stop_requested_.store(true);
  for (auto& worker : workers_) {
    if (worker.thread.joinable()) {
      worker.thread.join();
    }
  }
  workers_.clear();
  if (stm32_available_.load(std::memory_order_acquire)) {
    stm32_bridge_.Close();
  }
  if (mapping_mode_) {
    SaveMappingArtifacts();
  }
  if (foxglove_server_.is_running()) {
    foxglove_server_.Stop();
  }
}

void Runtime::ThreadMain(ThreadBinding binding, int cpu_id) {
  SetCurrentThreadName(binding.domain);
  utils::LogInfo("thread", std::string(binding.display_name) +
                               " started, affinity=" + BindCurrentThread(cpu_id));

  switch (binding.domain) {
    case ThreadDomain::kDriver:
      DriverThreadMain();
      break;
    case ThreadDomain::kSync:
      SyncThreadMain();
      break;
    case ThreadDomain::kPoseCore:
      PoseThreadMain();
      break;
    case ThreadDomain::kPerception:
      PerceptionThreadMain();
      break;
    case ThreadDomain::kPlanner:
      PlannerThreadMain();
      break;
    case ThreadDomain::kSafetyFsm:
      SafetyThreadMain();
      break;
    case ThreadDomain::kDebug:
      DebugThreadMain();
      break;
  }

  utils::LogInfo("thread", std::string(binding.display_name) + " stopping");
}

void Runtime::DriverThreadMain() {
  auto next_lidar_due = common::Now();
  auto next_heartbeat_due = common::Now();
  std::uint64_t last_sent_cmd_generation = 0;
  bool offline_logged = false;
  auto cpu_window_begin = common::Now();
  auto cpu_time_begin_ns = ThreadCpuTimeNs();

  while (!stop_requested_.load(std::memory_order_acquire)) {
    if (loaded_config_.sensors.imu_enabled) {
      std::optional<data::ImuPacket> packet;
      if (loaded_config_.sensors.imu_source == "lidar_internal") {
        packet = lidar_driver_.PollImuPacket();
      } else {
        packet = imu_driver_.PollPacket();
      }
      if (packet.has_value()) {
        sensor_sync_.PushImuPacket(*packet);
        driver_imu_packets_.fetch_add(1, std::memory_order_relaxed);
      }
    } else {
      common::SleepFor(std::chrono::milliseconds(2));
    }

    const auto now = common::Now();
    if (loaded_config_.sensors.lidar_enabled && now >= next_lidar_due) {
      auto frame = lidar_driver_.PollFrame();
      if (frame.has_value()) {
        sensor_sync_.PushLidarFrame(*frame);
        driver_lidar_frames_.fetch_add(1, std::memory_order_relaxed);
        const auto fsm_snapshot = fsm_snapshot_.ReadSnapshot();
        if (!stm32_available_.load(std::memory_order_acquire) &&
            fsm_snapshot.localization_active) {
          stm32_odom_.Publish(MakeSyntheticOdom(frame->frame_index, frame->stamp));
        }
        next_lidar_due = common::Now() + std::chrono::milliseconds(100);
      }
    }

    if (!stm32_available_.load(std::memory_order_acquire)) {
      LogOncePerThread("driver", "stm32 bridge offline, command output is disabled",
                       &offline_logged);
      continue;
    }

    if (now >= next_heartbeat_due) {
      const auto heartbeat_status = stm32_bridge_.SendHeartbeat();
      if (!heartbeat_status.ok()) {
        utils::LogWarn("driver", heartbeat_status.message);
      }
      next_heartbeat_due = now + std::chrono::milliseconds(200);
    }

    if (const auto cmd_generation = safety_cmd_.generation();
        cmd_generation != last_sent_cmd_generation) {
      const auto safe_cmd = safety_cmd_.ReadSnapshot();
      const auto send_status = stm32_bridge_.SendChassisCmd(safe_cmd);
      if (send_status.ok()) {
        last_sent_cmd_generation = cmd_generation;
      } else {
        utils::LogWarn("driver", send_status.message);
      }
    }

    stm32_bridge_.SpinOnce();
    if (auto odom = stm32_bridge_.TakeOdomState(); odom.has_value()) {
      stm32_odom_.Publish(*odom);
      last_stm32_rx_ns_.store(common::ToNanoseconds(odom->stamp), std::memory_order_release);
    }
    if (auto referee = stm32_bridge_.TakeRefereeState(); referee.has_value()) {
      referee_state_.Publish(*referee);
      last_stm32_rx_ns_.store(common::ToNanoseconds(referee->stamp), std::memory_order_release);
    }

    const auto now_window = common::Now();
    if (now_window - cpu_window_begin >= std::chrono::milliseconds(200)) {
      const auto cpu_time_now_ns = ThreadCpuTimeNs();
      const auto wall_delta_ns = common::ToNanoseconds(now_window - cpu_window_begin);
      const auto cpu_delta_ns = std::max<common::TimeNs>(0, cpu_time_now_ns - cpu_time_begin_ns);
      if (wall_delta_ns > 0) {
        driver_cpu_usage_milli_.store((cpu_delta_ns * 1000LL) / wall_delta_ns,
                                      std::memory_order_release);
      }
      cpu_window_begin = now_window;
      cpu_time_begin_ns = cpu_time_now_ns;
    }
  }
}

void Runtime::SyncThreadMain() {
  while (!stop_requested_.load(std::memory_order_acquire)) {
    const auto status = sensor_sync_.ProcessOnce();
    if (!status.ok()) {
      common::SleepFor(std::chrono::milliseconds(2));
      continue;
    }
    const auto perf = sensor_sync_.LatestPerf();
    sync_process_latency_ns_.store(perf.process_latency_ns, std::memory_order_release);
    sync_preintegration_latency_ns_.store(perf.preintegration_latency_ns,
                                          std::memory_order_release);
    sync_deskew_latency_ns_.store(perf.deskew_latency_ns, std::memory_order_release);

    auto synced_handle = sensor_sync_.TryPopSyncedFrameHandle();
    if (!synced_handle.has_value()) {
      continue;
    }

    const auto fsm_snapshot = fsm_snapshot_.ReadSnapshot();
    if (fsm_snapshot.mapping_active) {
      sync::SyncedFrameHandle mapping_frame;
      if (!CloneSyncedFrame(*synced_handle->get(), &mapping_frame)) {
        utils::LogWarn("sync", "fanout pool exhausted for mapping");
      } else if (!mapping_queue_.try_push(std::move(mapping_frame))) {
        utils::LogWarn("sync", "mapping queue is full");
      }
    }

    const bool bringup_mode = PerceptionBringupMode();
    if ((fsm_snapshot.localization_active &&
         combat_pipeline_ready_.load(std::memory_order_acquire)) ||
        bringup_mode) {
      if (!bringup_mode) {
        sync::SyncedFrameHandle localization_frame;
        if (!CloneSyncedFrame(*synced_handle->get(), &localization_frame)) {
          utils::LogWarn("sync", "fanout pool exhausted for localization");
        } else {
          localization_.EnqueueFrame(std::move(localization_frame));
        }
      }
      sync::SyncedFrameHandle preprocess_frame;
      if (!CloneSyncedFrame(*synced_handle->get(), &preprocess_frame)) {
        utils::LogWarn("sync", "fanout pool exhausted for preprocess");
      } else {
        preprocess_.EnqueueFrame(std::move(preprocess_frame));
      }
    }

    synced_frames_.fetch_add(1, std::memory_order_relaxed);
    synced_handle->reset();
  }
}

void Runtime::PoseThreadMain() {
  while (!stop_requested_.load(std::memory_order_acquire)) {
    const auto fsm_snapshot = fsm_snapshot_.ReadSnapshot();
    bool progressed = false;

    if (fsm_snapshot.mapping_active) {
      const auto odom = stm32_odom_.ReadSnapshot();
      if (odom.stamp != common::TimePoint{}) {
        mapping_pose_.Publish(MappingPoseFromOdom(odom));
      }

      sync::SyncedFrameHandle handle;
      if (mapping_queue_.try_pop(&handle)) {
        auto pose = mapping_pose_.ReadSnapshot();
        if (!pose.is_valid) {
          pose = MakeMapPose(common::Now(), static_cast<float>(loaded_config_.spawn.x_m),
                             static_cast<float>(loaded_config_.spawn.y_m),
                             static_cast<float>(loaded_config_.spawn.theta_rad));
          mapping_pose_.Publish(pose);
        }
        const auto status = mapping_engine_.Update(*handle.get(), pose);
        handle.reset();
        if (status.ok()) {
          mapped_frames_.fetch_add(1, std::memory_order_relaxed);
          progressed = true;
        }
      }
    }

    if (fsm_snapshot.localization_active &&
        combat_pipeline_ready_.load(std::memory_order_acquire)) {
      const auto odom = stm32_odom_.ReadSnapshot();
      if (odom.stamp != common::TimePoint{}) {
        localization_.SetLatestOdom(odom);
      }
      const auto status = localization_.ProcessOnce();
      if (status.ok()) {
        localized_frames_.fetch_add(1, std::memory_order_relaxed);
        const auto result = localization_.LatestResult();
        localization_latency_ns_.store(result.processing_latency_ns, std::memory_order_release);
        localization_matcher_latency_ns_.store(result.matcher_latency_ns,
                                               std::memory_order_release);
        localization_light_mode_.store(result.light_match_mode, std::memory_order_release);
        progressed = true;
      }
    }

    if (!progressed) {
      common::SleepFor(std::chrono::milliseconds(2));
    }
  }
}

void Runtime::PerceptionThreadMain() {
  while (!stop_requested_.load(std::memory_order_acquire)) {
    const auto fsm_snapshot = fsm_snapshot_.ReadSnapshot();
    const bool bringup_mode = PerceptionBringupMode();
    if ((!fsm_snapshot.localization_active ||
         !combat_pipeline_ready_.load(std::memory_order_acquire)) &&
        !bringup_mode) {
      common::SleepFor(std::chrono::milliseconds(10));
      continue;
    }

    const auto preprocess_status = preprocess_.ProcessOnce();
    if (!preprocess_status.ok()) {
      common::SleepFor(std::chrono::milliseconds(2));
    }

    auto filtered_frame = preprocess_.TryPopFilteredFrameHandle();
    if (!filtered_frame.has_value()) {
      continue;
    }

    auto pose = localization_.LatestResult().map_to_base;
    if (bringup_mode && !pose.is_valid) {
      pose = MakeMapPose(filtered_frame->get()->stamp,
                         static_cast<float>(loaded_config_.spawn.x_m),
                         static_cast<float>(loaded_config_.spawn.y_m),
                         static_cast<float>(loaded_config_.spawn.theta_rad));
    }
    if (pose.is_valid) {
      latest_filtered_scan_.Publish(*filtered_frame->get());
      mot_manager_.UpdateAndPublish(*filtered_frame->get(), pose);
      const auto obstacles = mot_manager_.LatestObstacles();
      local_costmap_builder_.BuildAndPublish(*filtered_frame->get(), pose,
                                             obstacles.obstacles);
      const auto mot_perf = mot_manager_.LatestPerf();
      mot_clustering_latency_ns_.store(mot_perf.clustering_latency_ns,
                                       std::memory_order_release);
      mot_total_latency_ns_.store(mot_perf.total_update_latency_ns,
                                  std::memory_order_release);
      mot_reduced_roi_active_.store(mot_perf.reduced_roi_active, std::memory_order_release);
      perceived_frames_.fetch_add(1, std::memory_order_relaxed);
    }
    filtered_frame->reset();
  }
}

void Runtime::PlannerThreadMain() {
  const auto period = LoopPeriodFromHz(loaded_config_.planner.loop_hz, 50);
  auto next_tick = common::Now();

  while (!stop_requested_.load(std::memory_order_acquire)) {
    const auto fsm_snapshot = fsm_snapshot_.ReadSnapshot();
    if (fsm_snapshot.mapping_active) {
      auto pose = mapping_pose_.ReadSnapshot();
      if (!pose.is_valid) {
        pose = MakeMapPose(common::Now(), static_cast<float>(loaded_config_.spawn.x_m),
                           static_cast<float>(loaded_config_.spawn.y_m),
                           static_cast<float>(loaded_config_.spawn.theta_rad));
        mapping_pose_.Publish(pose);
      }

      waypoint_manager_.AdvanceIfReached(
          pose, static_cast<float>(loaded_config_.mapping.route_reach_tolerance_m));
      const auto goal = waypoint_manager_.CurrentGoal();
      auto cmd = BuildWaypointCmd(pose, goal, loaded_config_);
      cmd.stamp = common::Now();
      mapping_cmd_.Publish(cmd);
      planned_cycles_.fetch_add(1, std::memory_order_relaxed);

      if (!stm32_available_.load(std::memory_order_acquire)) {
        mapping_pose_.Publish(IntegratePose(
            pose, cmd, static_cast<double>(common::ToNanoseconds(period)) / 1.0e9));
      }
      planner_latency_ns_.store(0, std::memory_order_release);
    } else if (fsm_snapshot.localization_active &&
               combat_pipeline_ready_.load(std::memory_order_acquire)) {
      const auto pose = localization_.LatestResult().map_to_base;
      const auto costmap = local_costmap_builder_.LatestCostmap();
      const auto obstacles = mot_manager_.LatestObstacles();
      if (pose.is_valid && costmap.width != 0U) {
        planner_.PlanAndPublish(pose, costmap, obstacles.obstacles);
        planner_latency_ns_.store(planner_.LatestStatus().planning_latency_ns,
                                  std::memory_order_release);
        planned_cycles_.fetch_add(1, std::memory_order_relaxed);
      }
    }
    next_tick += period;
    common::SleepUntil(next_tick);
  }
}

void Runtime::SafetyThreadMain() {
  const auto period = LoopPeriodFromHz(loaded_config_.safety.loop_hz, 20);
  auto next_tick = common::Now();
  data::ChassisCmd last_cmd;
  last_cmd.brake = true;
  std::uint64_t last_referee_generation = referee_state_.generation();

  while (!stop_requested_.load(std::memory_order_acquire)) {
    const auto tick_begin_ns = common::NowNs();
    const auto referee_generation = referee_state_.generation();
    const bool referee_changed = referee_generation != last_referee_generation;
    last_referee_generation = referee_generation;

    const auto now = common::Now();
    const auto fsm_context = BuildFsmContext(referee_changed);
    const auto snapshot = nav_fsm_.Update(now, fsm_context);
    fsm_snapshot_.Publish(snapshot);

    if (snapshot.state == fsm::NavState::kModeSave &&
        !map_saved_.load(std::memory_order_acquire)) {
      const auto save_status = SaveMappingArtifacts();
      if (save_status.ok()) {
        const auto init_status = InitializeCombatPipelineFromSavedMap();
        if (!init_status.ok()) {
          utils::LogWarn("fsm",
                         std::string("combat pipeline init after save failed: ") +
                             init_status.message);
        }
      } else if (save_status.code != common::StatusCode::kNotReady) {
        utils::LogWarn("fsm", std::string("mapping save failed: ") + save_status.message);
      }
    }

    const auto safe_cmd = SelectSafeCmd(snapshot, now);
    const auto localization_result = localization_.LatestResult();
    const auto planner_cmd = planner_.LatestCmd();
    const auto costmap = local_costmap_builder_.LatestCostmap();
    const auto obstacles = mot_manager_.LatestObstacles();
    const bool deadman_triggered =
        snapshot.localization_active && PlannerCmdTimedOut(planner_cmd, now, loaded_config_.safety);
    const auto collision_type =
        snapshot.localization_active && localization_result.map_to_base.is_valid &&
                localization_result.status.pose_trusted && !planner_cmd.brake &&
                !deadman_triggered
            ? DetectCommandCollision(localization_result.map_to_base, costmap,
                                     obstacles.obstacles,
                                     ClampCommandForMode(planner_cmd, snapshot, loaded_config_, now),
                                     loaded_config_.safety)
            : CommandCollisionType::kNone;

    if (!fsm_context.heartbeat_ok) {
      data::SafetyEvent safety_event;
      safety_event.stamp = now;
      safety_event.code = data::SafetyEventCode::kSensorTimeout;
      safety_event.severity = data::SafetySeverity::kCritical;
      safety_event.message = "stm32 heartbeat timeout";
      latest_safety_event_.Publish(safety_event);
    } else if (snapshot.failsafe_active) {
      data::SafetyEvent safety_event;
      safety_event.stamp = now;
      safety_event.code = data::SafetyEventCode::kFailsafeOverride;
      safety_event.severity = data::SafetySeverity::kCritical;
      safety_event.message = "fsm forced chassis command override";
      latest_safety_event_.Publish(safety_event);
    } else if (deadman_triggered) {
      data::SafetyEvent safety_event;
      safety_event.stamp = now;
      safety_event.code = data::SafetyEventCode::kDeadmanTimeout;
      safety_event.severity = data::SafetySeverity::kCritical;
      safety_event.message = "planner command deadman timeout";
      latest_safety_event_.Publish(safety_event);
    } else if (collision_type == CommandCollisionType::kStatic) {
      data::SafetyEvent safety_event;
      safety_event.stamp = now;
      safety_event.code = data::SafetyEventCode::kStaticCollision;
      safety_event.severity = data::SafetySeverity::kCritical;
      safety_event.message = "command gate blocked by static obstacle";
      latest_safety_event_.Publish(safety_event);
    } else if (collision_type == CommandCollisionType::kDynamic) {
      data::SafetyEvent safety_event;
      safety_event.stamp = now;
      safety_event.code = data::SafetyEventCode::kDynamicCollision;
      safety_event.severity = data::SafetySeverity::kCritical;
      safety_event.message = "command gate blocked by dynamic obstacle";
      latest_safety_event_.Publish(safety_event);
    } else if (fsm_context.safety_triggered) {
      data::SafetyEvent safety_event;
      safety_event.stamp = now;
      safety_event.code = data::SafetyEventCode::kObstacleTooClose;
      safety_event.severity = data::SafetySeverity::kCritical;
      safety_event.message = "dynamic obstacle too close";
      latest_safety_event_.Publish(safety_event);
    } else if (fsm_context.localization_degraded) {
      data::SafetyEvent safety_event;
      safety_event.stamp = now;
      safety_event.code = data::SafetyEventCode::kPoseLost;
      safety_event.severity = data::SafetySeverity::kWarning;
      safety_event.message = "localization degraded";
      latest_safety_event_.Publish(safety_event);
    } else if (fsm_context.planner_failed) {
      data::SafetyEvent safety_event;
      safety_event.stamp = now;
      safety_event.code = data::SafetyEventCode::kPlannerStall;
      safety_event.severity = data::SafetySeverity::kWarning;
      safety_event.message = "planner has no path";
      latest_safety_event_.Publish(safety_event);
    }

    if (safe_cmd.stamp != last_cmd.stamp || safe_cmd.brake != last_cmd.brake ||
        safe_cmd.vx_mps != last_cmd.vx_mps || safe_cmd.vy_mps != last_cmd.vy_mps ||
        safe_cmd.wz_radps != last_cmd.wz_radps) {
      safety_cmd_.Publish(safe_cmd);
      last_cmd = safe_cmd;
    }
    safety_tick_latency_ns_.store(common::NowNs() - tick_begin_ns, std::memory_order_release);
    safety_cycles_.fetch_add(1, std::memory_order_relaxed);
    next_tick += period;
    common::SleepUntil(next_tick);
  }
}

void Runtime::DebugThreadMain() {
  const auto period = LoopPeriodFromHz(loaded_config_.debug.publish_hz, 250);
  const auto scalar_period = LoopPeriodFromHz(loaded_config_.debug.scalar_publish_hz, 50);
  const auto pointcloud_period =
      LoopPeriodFromHz(loaded_config_.debug.pointcloud_publish_hz, 200);
  auto next_log = common::Now();
  auto next_scalar_publish = common::Now();
  auto next_pointcloud_publish = common::Now();
  std::uint64_t last_safety_event_generation = latest_safety_event_.generation();
  while (!stop_requested_.load(std::memory_order_acquire)) {
    const auto now = common::Now();
    if (now >= next_log) {
      std::ostringstream stream;
      const auto fsm_snapshot = fsm_snapshot_.ReadSnapshot();
      stream << "pipeline stats lidar=" << driver_lidar_frames_.load()
             << " imu=" << driver_imu_packets_.load()
             << " sync=" << synced_frames_.load()
             << " pose=" << localized_frames_.load()
             << " mapping=" << mapped_frames_.load()
             << " perception=" << perceived_frames_.load()
             << " planner=" << planned_cycles_.load()
             << " safety=" << safety_cycles_.load()
             << " dropped(sync imu/lidar/frame)="
             << sensor_sync_.dropped_imu_packets() << '/'
             << sensor_sync_.dropped_lidar_frames() << '/'
             << sensor_sync_.dropped_synced_frames()
             << " dropped(localization/preprocess)="
             << localization_.dropped_frames() << '/' << preprocess_.dropped_frames()
             << " fsm(state=" << fsm::ToString(fsm_snapshot.state)
             << ", event=" << fsm::ToString(fsm_snapshot.last_event.code) << ")";
      if (fsm_snapshot.mapping_active) {
        const auto mapping_result = mapping_engine_.LatestResult();
        const auto pose = mapping_pose_.ReadSnapshot();
        const auto cmd = mapping_cmd_.ReadSnapshot();
        const auto goal = waypoint_manager_.CurrentGoal();
        stream << " map(mode=warmup"
               << ", frames=" << mapping_result.processed_frames
               << ", points=" << mapping_result.accumulated_points
               << ", pose=" << pose.position.x << "/" << pose.position.y
               << ", goal=" << goal.position.x << "/" << goal.position.y
               << ", cmd=" << cmd.vx_mps << "/" << cmd.vy_mps << "/" << cmd.wz_radps
               << ")";
      } else if (fsm_snapshot.localization_active) {
        const auto localization_result = localization_.LatestResult();
        const auto planner_status = planner_.LatestStatus();
        stream << " loc(score=" << localization_result.status.match_score
               << ", iter=" << localization_result.status.iterations
               << ", fail=" << localization_result.status.consecutive_failures
               << ", trusted=" << (localization_result.status.pose_trusted ? "true" : "false")
               << ", map=" << (localization_.map_loaded() ? "loaded" : "missing") << ")"
               << " nav(mode="
               << (planner_status.mode == planning::GoalMode::kCenterHold ? "hold" : "goto")
               << ", dist=" << planner_status.distance_to_goal_m
               << ", reached=" << (planner_status.reached ? "true" : "false")
               << ", cmd=" << planner_.LatestCmd().vx_mps << "/" << planner_.LatestCmd().vy_mps
               << "/" << planner_.LatestCmd().wz_radps << ")";
      }
      stream << " perf(sync_ms="
             << static_cast<double>(sync_process_latency_ns_.load(std::memory_order_acquire)) /
                    1.0e6
             << ", match_ms="
             << static_cast<double>(
                    localization_matcher_latency_ns_.load(std::memory_order_acquire)) /
                    1.0e6
             << ", cluster_ms="
             << static_cast<double>(mot_clustering_latency_ns_.load(std::memory_order_acquire)) /
                    1.0e6
             << ", dwa_ms="
             << static_cast<double>(planner_latency_ns_.load(std::memory_order_acquire)) / 1.0e6
             << ", safety_ms="
             << static_cast<double>(safety_tick_latency_ns_.load(std::memory_order_acquire)) /
                    1.0e6
             << ", driver_cpu=" << driver_cpu_usage_milli_.load(std::memory_order_acquire) / 10.0
             << "%)";
      utils::LogInfo("debug", stream.str());
      if (loaded_config_.debug.websocket_enabled) {
        const auto debug_begin_ns = common::NowNs();
        const bool allow_scalar_publish = now >= next_scalar_publish;
        const bool allow_pointcloud_publish = now >= next_pointcloud_publish;
        const bool high_debug_load =
            foxglove_publish_latency_ns_.load(std::memory_order_acquire) >
                static_cast<common::TimeNs>(loaded_config_.debug.high_load_debug_threshold_ms) *
                    1000000LL ||
            debug_encode_latency_ns_.load(std::memory_order_acquire) >
                static_cast<common::TimeNs>(loaded_config_.debug.high_load_debug_threshold_ms) *
                    1000000LL;
        const bool suppress_scene =
            high_debug_load && loaded_config_.debug.drop_pointcloud_on_high_load;
        debug_scene_suppressed_.store(suppress_scene, std::memory_order_release);

        if (allow_scalar_publish) {
          next_scalar_publish = now + scalar_period;
          const auto scalar_begin_ns = common::NowNs();
          WriteFsmDebugSnapshot(fsm_snapshot, RuntimeDebugOutputDir());
          auto current_pose = fsm_snapshot.mapping_active ? mapping_pose_.ReadSnapshot()
                                                          : localization_.LatestResult().map_to_base;
          if (PerceptionBringupMode() && !current_pose.is_valid) {
            current_pose = MakeMapPose(now, static_cast<float>(loaded_config_.spawn.x_m),
                                       static_cast<float>(loaded_config_.spawn.y_m),
                                       static_cast<float>(loaded_config_.spawn.theta_rad));
          }
          const auto current_cmd =
              fsm_snapshot.mapping_active ? mapping_cmd_.ReadSnapshot() : safety_cmd_.ReadSnapshot();
          const auto perf_json = BuildPerfStatusJson(
              driver_cpu_usage_milli_.load(std::memory_order_acquire),
              sync_process_latency_ns_.load(std::memory_order_acquire),
              sync_preintegration_latency_ns_.load(std::memory_order_acquire),
              sync_deskew_latency_ns_.load(std::memory_order_acquire),
              localization_latency_ns_.load(std::memory_order_acquire),
              localization_matcher_latency_ns_.load(std::memory_order_acquire),
              localization_light_mode_.load(std::memory_order_acquire),
              mot_clustering_latency_ns_.load(std::memory_order_acquire),
              mot_total_latency_ns_.load(std::memory_order_acquire),
              mot_reduced_roi_active_.load(std::memory_order_acquire),
              planner_latency_ns_.load(std::memory_order_acquire),
              safety_tick_latency_ns_.load(std::memory_order_acquire),
              foxglove_publish_latency_ns_.load(std::memory_order_acquire),
              debug_encode_latency_ns_.load(std::memory_order_acquire),
              debug_scene_suppressed_.load(std::memory_order_acquire));
          WritePerfDebugSnapshot(perf_json, RuntimeDebugOutputDir());
          PublishFoxgloveJson(&foxglove_server_, kRuntimeFsmStatusChannelId,
                              BuildFsmStatusJson(fsm_snapshot), now);
          PublishFoxgloveJson(&foxglove_server_, kRuntimeFsmEventChannelId,
                              BuildFsmEventJson(fsm_snapshot.last_event), now);
          PublishFoxgloveJson(&foxglove_server_, kRuntimePoseChannelId,
                              BuildPoseJson(current_pose), now);
          PublishFoxgloveJson(&foxglove_server_, kRuntimeCmdChannelId,
                              BuildCmdJson(current_cmd), now);
          PublishFoxgloveJson(&foxglove_server_, kRuntimePerfStatusChannelId, perf_json, now);
          std::filesystem::create_directories("logs/watchdog");
          std::ofstream heartbeat("logs/watchdog/process_heartbeat.json");
          heartbeat << "{\n  \"stamp_ns\": " << common::ToNanoseconds(now)
                    << ",\n  \"state\": \"" << fsm::ToString(fsm_snapshot.state)
                    << "\"\n}\n";
          debug_encode_latency_ns_.store(common::NowNs() - scalar_begin_ns,
                                         std::memory_order_release);
        }

        if (allow_pointcloud_publish && !suppress_scene && fsm_snapshot.mapping_active) {
          next_pointcloud_publish = now + pointcloud_period;
          const auto publish_begin_ns = common::NowNs();
          auto current_pose = mapping_pose_.ReadSnapshot();
          if (!current_pose.is_valid) {
            current_pose = MakeMapPose(now, static_cast<float>(loaded_config_.spawn.x_m),
                                       static_cast<float>(loaded_config_.spawn.y_m),
                                       static_cast<float>(loaded_config_.spawn.theta_rad));
          }
          WriteMappingDebugSnapshot(mapping_engine_, waypoint_manager_,
                                    mapping_pose_.ReadSnapshot(), mapping_cmd_.ReadSnapshot(),
                                    RuntimeDebugOutputDir());
          const auto mapping_status_json =
              BuildRuntimeMappingStatusJson(mapping_engine_, waypoint_manager_);
          const auto waypoint_json = BuildScalarPoseJson(
              waypoint_manager_.CurrentGoal(), waypoint_manager_.current_index(),
              waypoint_manager_.waypoint_count());
          const auto partial_map_scene_json = BuildSceneUpdateJson(
              "runtime_partial_map", tf::kMapFrame,
              DownsampleScenePoints(mapping_engine_.GlobalPointCloud(), 1500U), 0.08F, 0.04F,
              {0.15F, 0.75F, 0.32F, 0.80F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeMappingStatusChannelId,
                              mapping_status_json, now);
          PublishFoxgloveJson(&foxglove_server_, kRuntimeCurrentWaypointChannelId,
                              waypoint_json, now);
          PublishFoxgloveJson(&foxglove_server_, kRuntimePartialMapSceneChannelId,
                              partial_map_scene_json, now);
          const auto costmap_scene_json = BuildSceneUpdateJson(
              "runtime_local_costmap", tf::kMapFrame,
              CostmapOccupiedPoints(local_costmap_builder_.LatestCostmap(), 1200U), 0.10F, 0.04F,
              {0.95F, 0.38F, 0.12F, 0.82F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeLocalCostmapSceneChannelId,
                              costmap_scene_json, now);
          const auto footprint_scene_json = BuildSceneUpdateJson(
              "runtime_footprint", tf::kMapFrame,
              FootprintOutlinePoints(loaded_config_.sensors.lidar_self_mask, current_pose, 12U),
              0.05F, 0.08F, {0.10F, 0.75F, 0.95F, 0.95F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeFootprintSceneChannelId,
                              footprint_scene_json, now);
          const auto laser_link_scene_json = BuildSceneUpdateJson(
              "runtime_laser_link", tf::kMapFrame,
              LaserLinkMarkerPoints(loaded_config_.sensors.lidar_mount, current_pose), 0.06F,
              0.10F, {1.00F, 0.95F, 0.15F, 0.98F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeLaserLinkSceneChannelId,
                              laser_link_scene_json, now);
          const auto base_link_scene_json = BuildSceneUpdateJson(
              "runtime_base_link", tf::kMapFrame, BaseLinkMarkerPoints(current_pose), 0.07F,
              0.11F, {0.15F, 0.95F, 0.35F, 0.98F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeBaseLinkSceneChannelId,
                              base_link_scene_json, now);
          const auto current_scan_scene_json = BuildSceneUpdateJson(
              "runtime_current_scan", tf::kMapFrame,
              CurrentScanScenePoints(latest_filtered_scan_.ReadSnapshot(), current_pose,
                                     loaded_config_.sensors.lidar_mount, 1600U),
              0.04F, 0.04F, {0.92F, 0.92F, 0.92F, 0.75F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeCurrentScanSceneChannelId,
                              current_scan_scene_json, now);
          foxglove_publish_latency_ns_.store(common::NowNs() - publish_begin_ns,
                                             std::memory_order_release);
        } else if (allow_pointcloud_publish && !suppress_scene &&
                   fsm_snapshot.localization_active) {
          next_pointcloud_publish = now + pointcloud_period;
          const auto publish_begin_ns = common::NowNs();
          auto current_pose = localization_.LatestResult().map_to_base;
          WriteDebugSnapshot(localization_, planner_, RuntimeDebugOutputDir());
          const auto costmap_scene_json = BuildSceneUpdateJson(
              "runtime_local_costmap", tf::kMapFrame,
              CostmapOccupiedPoints(local_costmap_builder_.LatestCostmap(), 1200U), 0.10F, 0.04F,
              {0.95F, 0.38F, 0.12F, 0.82F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeLocalCostmapSceneChannelId,
                              costmap_scene_json, now);
          const auto footprint_scene_json = BuildSceneUpdateJson(
              "runtime_footprint", tf::kMapFrame,
              FootprintOutlinePoints(loaded_config_.sensors.lidar_self_mask, current_pose, 12U),
              0.05F, 0.08F, {0.10F, 0.75F, 0.95F, 0.95F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeFootprintSceneChannelId,
                              footprint_scene_json, now);
          const auto laser_link_scene_json = BuildSceneUpdateJson(
              "runtime_laser_link", tf::kMapFrame,
              LaserLinkMarkerPoints(loaded_config_.sensors.lidar_mount, current_pose), 0.06F,
              0.10F, {1.00F, 0.95F, 0.15F, 0.98F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeLaserLinkSceneChannelId,
                              laser_link_scene_json, now);
          const auto base_link_scene_json = BuildSceneUpdateJson(
              "runtime_base_link", tf::kMapFrame, BaseLinkMarkerPoints(current_pose), 0.07F,
              0.11F, {0.15F, 0.95F, 0.35F, 0.98F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeBaseLinkSceneChannelId,
                              base_link_scene_json, now);
          const auto current_scan_scene_json = BuildSceneUpdateJson(
              "runtime_current_scan", tf::kMapFrame,
              CurrentScanScenePoints(latest_filtered_scan_.ReadSnapshot(), current_pose,
                                     loaded_config_.sensors.lidar_mount, 1600U),
              0.04F, 0.04F, {0.92F, 0.92F, 0.92F, 0.75F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeCurrentScanSceneChannelId,
                              current_scan_scene_json, now);
          foxglove_publish_latency_ns_.store(common::NowNs() - publish_begin_ns,
                                             std::memory_order_release);
        } else if (allow_pointcloud_publish && !suppress_scene && PerceptionBringupMode()) {
          next_pointcloud_publish = now + pointcloud_period;
          const auto publish_begin_ns = common::NowNs();
          const auto current_pose = MakeMapPose(now, static_cast<float>(loaded_config_.spawn.x_m),
                                                static_cast<float>(loaded_config_.spawn.y_m),
                                                static_cast<float>(loaded_config_.spawn.theta_rad));
          const auto costmap_scene_json = BuildSceneUpdateJson(
              "runtime_local_costmap", tf::kMapFrame,
              CostmapOccupiedPoints(local_costmap_builder_.LatestCostmap(), 1200U), 0.10F, 0.04F,
              {0.95F, 0.38F, 0.12F, 0.82F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeLocalCostmapSceneChannelId,
                              costmap_scene_json, now);
          const auto footprint_scene_json = BuildSceneUpdateJson(
              "runtime_footprint", tf::kMapFrame,
              FootprintOutlinePoints(loaded_config_.sensors.lidar_self_mask, current_pose, 12U),
              0.05F, 0.08F, {0.10F, 0.75F, 0.95F, 0.95F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeFootprintSceneChannelId,
                              footprint_scene_json, now);
          const auto laser_link_scene_json = BuildSceneUpdateJson(
              "runtime_laser_link", tf::kMapFrame,
              LaserLinkMarkerPoints(loaded_config_.sensors.lidar_mount, current_pose), 0.06F,
              0.10F, {1.00F, 0.95F, 0.15F, 0.98F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeLaserLinkSceneChannelId,
                              laser_link_scene_json, now);
          const auto base_link_scene_json = BuildSceneUpdateJson(
              "runtime_base_link", tf::kMapFrame, BaseLinkMarkerPoints(current_pose), 0.07F,
              0.11F, {0.15F, 0.95F, 0.35F, 0.98F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeBaseLinkSceneChannelId,
                              base_link_scene_json, now);
          const auto current_scan_scene_json = BuildSceneUpdateJson(
              "runtime_current_scan", tf::kMapFrame,
              CurrentScanScenePoints(latest_filtered_scan_.ReadSnapshot(), current_pose,
                                     loaded_config_.sensors.lidar_mount, 1600U),
              0.04F, 0.04F, {0.92F, 0.92F, 0.92F, 0.75F});
          PublishFoxgloveJson(&foxglove_server_, kRuntimeCurrentScanSceneChannelId,
                              current_scan_scene_json, now);
          foxglove_publish_latency_ns_.store(common::NowNs() - publish_begin_ns,
                                             std::memory_order_release);
        }

        if (latest_safety_event_.generation() != last_safety_event_generation) {
          last_safety_event_generation = latest_safety_event_.generation();
          const auto safety_event = latest_safety_event_.ReadSnapshot();
          WriteSafetyEventDebugSnapshot(safety_event, RuntimeDebugOutputDir());
          PublishFoxgloveJson(&foxglove_server_, kRuntimeSafetyEventChannelId,
                              BuildSafetyEventJson(safety_event), now);
        }
        foxglove_publish_latency_ns_.store(common::NowNs() - debug_begin_ns,
                                           std::memory_order_release);
      }
      next_log = now + period;
    }
    common::SleepFor(std::chrono::milliseconds(20));
  }
}

bool Runtime::CloneSyncedFrame(const data::SyncedFrame& source,
                               sync::SyncedFrameHandle* handle) {
  if (handle == nullptr) {
    return false;
  }
  auto acquired = fanout_pool_.Acquire();
  if (!acquired) {
    return false;
  }
  *acquired.get() = source;
  *handle = std::move(acquired);
  return true;
}

common::Status Runtime::InitializeCombatPipeline(
    const config::LocalizationConfig& localization_config,
    const config::SpawnConfig& spawn_config) {
  auto status =
      localization_.Initialize(loaded_config_.config_dir, localization_config, spawn_config,
                               &tf_tree_);
  if (!status.ok()) {
    return status;
  }
  status = planner_.Initialize(loaded_config_.planner, localization_.static_map());
  if (!status.ok()) {
    return status;
  }
  combat_pipeline_ready_.store(true, std::memory_order_release);
  return common::Status::Ok();
}

common::Status Runtime::InitializeCombatPipelineFromSavedMap() {
  if (combat_pipeline_ready_.load(std::memory_order_acquire)) {
    return common::Status::Ok();
  }

  config::LocalizationConfig localization_config = loaded_config_.localization;
  localization_config.enabled = true;
  const auto output_dir =
      ResolveConfigAsset(loaded_config_.config_dir, loaded_config_.mapping.output_dir);
  localization_config.global_map_pcd_path = (output_dir / "global_map.pcd").string();
  localization_config.occupancy_path = (output_dir / "occupancy.bin").string();
  localization_config.map_meta_path = (output_dir / "map_meta.json").string();

  config::SpawnConfig spawn_config = loaded_config_.spawn;
  const auto latest_mapping_pose = mapping_pose_.ReadSnapshot();
  if (latest_mapping_pose.is_valid) {
    spawn_config.x_m = latest_mapping_pose.position.x;
    spawn_config.y_m = latest_mapping_pose.position.y;
    spawn_config.theta_rad = latest_mapping_pose.rpy.z;
  }
  return InitializeCombatPipeline(localization_config, spawn_config);
}

fsm::NavFsmContext Runtime::BuildFsmContext(bool referee_changed) const {
  fsm::NavFsmContext context;
  const int manual_mode_selector = loaded_config_.system.manual_mode_selector;
  const bool map_saved = map_saved_.load(std::memory_order_acquire);
  const bool combat_ready = combat_pipeline_ready_.load(std::memory_order_acquire);
  context.save_requested = mapping_save_requested_.load(std::memory_order_acquire);
  context.map_saved = map_saved;
  context.combat_ready = combat_ready;
  context.map_loaded = context.combat_ready && localization_.map_loaded();
  context.referee_changed = referee_changed;

  if (manual_mode_selector == 0) {
    context.mapping_enabled = true;
    context.combat_ready = false;
    context.map_loaded = false;
  } else if (manual_mode_selector == 1) {
    context.mapping_enabled = false;
    context.save_requested = false;
  } else {
    context.mapping_enabled = mapping_mode_ && !map_saved;
  }

  if (stm32_available_.load(std::memory_order_acquire)) {
    const auto last_rx_ns = last_stm32_rx_ns_.load(std::memory_order_acquire);
    context.heartbeat_ok =
        last_rx_ns != 0 &&
        (common::NowNs() - last_rx_ns) <=
            static_cast<common::TimeNs>(loaded_config_.safety.heartbeat_timeout_ms) * 1000000LL;
  }

  if (context.combat_ready) {
    const auto localization_result = localization_.LatestResult();
    const auto planner_status = planner_.LatestStatus();
    const auto latest_cmd = planner_.LatestCmd();
    context.localization_degraded =
        localization_result.map_to_base.is_valid &&
        (!localization_result.status.pose_trusted ||
         localization_result.status.consecutive_failures >=
             static_cast<std::uint32_t>(
                 std::max(1, loaded_config_.localization.relocalization_failure_threshold)));
    context.planner_failed =
        planned_cycles_.load(std::memory_order_relaxed) > 0U &&
        (!planner_status.reached && !planner_status.path_available);
    if (latest_cmd.stamp != common::TimePoint{}) {
      const auto cmd_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  common::Now() - latest_cmd.stamp)
                                  .count();
      if (cmd_age_ms > loaded_config_.safety.deadman_timeout_ms) {
        context.planner_failed = true;
      }
    }
    context.center_reached = planner_status.reached;
    context.center_drifted =
        planner_status.mode == planning::GoalMode::kCenterHold &&
        planner_status.distance_to_center_m >
            static_cast<float>(loaded_config_.planner.recenter_threshold_m);

    const auto pose = localization_result.map_to_base;
    if (pose.is_valid) {
      const auto obstacles = mot_manager_.LatestObstacles();
      const float threshold =
          static_cast<float>(loaded_config_.safety.emergency_stop_distance_m);
      for (const auto& obstacle : obstacles.obstacles) {
        const float dx = obstacle.pose.position.x - pose.position.x;
        const float dy = obstacle.pose.position.y - pose.position.y;
        const float distance = std::sqrt(dx * dx + dy * dy) - obstacle.radius_m;
        if (distance <= threshold) {
          context.safety_triggered = true;
          break;
        }
      }
    }
  }

  return context;
}

data::ChassisCmd Runtime::SelectSafeCmd(const fsm::NavFsmSnapshot& snapshot,
                                        common::TimePoint stamp) const {
  data::ChassisCmd safe_cmd;
  safe_cmd.stamp = stamp;
  safe_cmd.brake = true;

  switch (snapshot.state) {
    case fsm::NavState::kModeWarmup: {
      safe_cmd = mapping_cmd_.ReadSnapshot();
      if (!mapping_pose_.ReadSnapshot().is_valid) {
        safe_cmd = {};
        safe_cmd.stamp = stamp;
        safe_cmd.brake = true;
      }
      break;
    }
    case fsm::NavState::kModeCombat:
    case fsm::NavState::kGotoCenter:
    case fsm::NavState::kHoldCenter:
    case fsm::NavState::kRecenter:
    case fsm::NavState::kRecovery: {
      if (!combat_pipeline_ready_.load(std::memory_order_acquire)) {
        break;
      }
      const auto localization_result = localization_.LatestResult();
      const auto pose = localization_result.map_to_base;
      const auto costmap = local_costmap_builder_.LatestCostmap();
      const auto obstacles = mot_manager_.LatestObstacles();
      const auto planner_cmd = planner_.LatestCmd();
      if (!pose.is_valid || costmap.width == 0U) {
        break;
      }
      if (snapshot.failsafe_active) {
        break;
      }
      if (!localization_result.status.pose_trusted) {
        break;
      }
      if (PlannerCmdTimedOut(planner_cmd, stamp, loaded_config_.safety)) {
        break;
      }

      safe_cmd = ClampCommandForMode(planner_cmd, snapshot, loaded_config_, stamp);
      if (snapshot.recovery_active) {
        safe_cmd.vx_mps *= static_cast<float>(loaded_config_.safety.recovery_speed_scale);
        safe_cmd.vy_mps *= static_cast<float>(loaded_config_.safety.recovery_speed_scale);
        safe_cmd.wz_radps *= static_cast<float>(loaded_config_.safety.recovery_yaw_scale);
      }
      if (safe_cmd.brake ||
          DetectCommandCollision(pose, costmap, obstacles.obstacles, safe_cmd,
                                 loaded_config_.safety) != CommandCollisionType::kNone) {
        safe_cmd = {};
        safe_cmd.stamp = stamp;
        safe_cmd.brake = true;
        break;
      }
      if (snapshot.recovery_active) {
        safe_cmd = ClampCommandForMode(safe_cmd, snapshot, loaded_config_, stamp);
      }
      safe_cmd.brake = false;
      break;
    }
    case fsm::NavState::kBoot:
    case fsm::NavState::kSelfCheck:
    case fsm::NavState::kIdle:
    case fsm::NavState::kModeSave:
    case fsm::NavState::kFailsafe:
      break;
  }

  return safe_cmd;
}

std::string Runtime::RuntimeDebugOutputDir() const { return "logs/debug/foxglove"; }

common::Status Runtime::SaveMappingArtifacts(localization::StaticMap* exported_map) {
  if (!mapping_mode_) {
    return common::Status::NotReady("mapping mode is disabled");
  }
  if (map_saved_.load(std::memory_order_acquire)) {
    return common::Status::Ok();
  }
  if (mapping_engine_.LatestResult().processed_frames == 0U) {
    return common::Status::NotReady("no mapping frames available");
  }
  localization::StaticMap local_exported_map;
  const auto status = mapping_engine_.SaveMap(
      ResolveConfigAsset(loaded_config_.config_dir, loaded_config_.mapping.output_dir).string(),
      exported_map != nullptr ? exported_map : &local_exported_map);
  if (!status.ok()) {
    utils::LogWarn("mapping", std::string("failed to save mapping artifacts: ") + status.message);
    return status;
  }
  map_saved_.store(true, std::memory_order_release);
  utils::LogInfo("mapping",
                 std::string("saved warmup map to ") +
                     (exported_map != nullptr ? exported_map->global_map_path
                                              : local_exported_map.global_map_path));
  return common::Status::Ok();
}

bool Runtime::PerceptionBringupMode() const {
  if (loaded_config_.system.bringup_mode == "lidar_view") {
    return true;
  }
  return loaded_config_.sensors.lidar_source == "unitree_sdk" &&
         !loaded_config_.localization.enabled;
}

void Runtime::LogOncePerThread(const char* component, const std::string& message,
                               bool* already_logged) {
  if (already_logged != nullptr && !*already_logged) {
    utils::LogInfo(component, message);
    *already_logged = true;
  }
}

int Runtime::ResolveCpu(ThreadDomain domain) const {
  const auto& affinity = loaded_config_.system.thread_affinity;
  switch (domain) {
    case ThreadDomain::kDriver:
      return affinity.driver_cpu;
    case ThreadDomain::kSync:
      return affinity.sync_cpu;
    case ThreadDomain::kPoseCore:
      return affinity.pose_core_cpu;
    case ThreadDomain::kPerception:
      return affinity.perception_cpu;
    case ThreadDomain::kPlanner:
      return affinity.planner_cpu;
    case ThreadDomain::kSafetyFsm:
      return affinity.safety_fsm_cpu;
    case ThreadDomain::kDebug:
      return affinity.debug_cpu;
  }
  return -1;
}

void Runtime::LogStartupSummary() const {
  config::ConfigLoader loader;
  utils::LogInfo("runtime",
                 std::string("version=") + RM_NAV_VERSION +
                     ", config_version=" + loaded_config_.system.version);
  utils::LogInfo("runtime",
                 std::string("loaded config files: ") + JoinFiles(loaded_config_.loaded_files));
  utils::LogInfo("runtime",
                 std::string("loaded modules: ") + JoinModules(manifest_));
  utils::LogInfo("runtime", loader.BuildSummary(loaded_config_));
  utils::LogInfo("runtime",
                 std::string("pipeline configured, stm32=") +
                     (stm32_available_.load(std::memory_order_acquire) ? "online"
                                                                       : "offline") +
                     ", mode=" + (mapping_mode_ ? std::string("mapping")
                                                : std::string("navigation")));

  for (const auto& thread_binding : manifest_.threads) {
    const int cpu_id = ResolveCpu(thread_binding.domain);
    utils::LogInfo(
        "runtime",
        std::string(thread_binding.display_name) + " requested_cpu=" +
            (cpu_id >= 0 ? std::to_string(cpu_id) : std::string("unbound")));
  }
}

data::Pose3f Runtime::MappingPoseFromOdom(const data::OdomState& odom) const {
  const float origin_yaw = static_cast<float>(loaded_config_.spawn.theta_rad);
  const float cos_yaw = std::cos(origin_yaw);
  const float sin_yaw = std::sin(origin_yaw);
  const float map_x = static_cast<float>(loaded_config_.spawn.x_m) +
                      cos_yaw * odom.x_m - sin_yaw * odom.y_m;
  const float map_y = static_cast<float>(loaded_config_.spawn.y_m) +
                      sin_yaw * odom.x_m + cos_yaw * odom.y_m;
  return MakeMapPose(odom.stamp, map_x, map_y, NormalizeAngle(origin_yaw + odom.yaw_rad));
}

}  // namespace rm_nav::app
