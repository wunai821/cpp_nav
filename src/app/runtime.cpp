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
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/debug/foxglove_server.hpp"
#include "rm_nav/data/tf_types.hpp"
#include "rm_nav/tf/frame_ids.hpp"
#include "rm_nav/tf/transform_query.hpp"
#include "rm_nav/utils/logger.hpp"

namespace rm_nav::app {
namespace {

Runtime::MappingSaveFailureTag ToRuntimeFailureTag(mapping::MapSaveFailureKind kind) {
  switch (kind) {
    case mapping::MapSaveFailureKind::kWriteFailed:
      return Runtime::MappingSaveFailureTag::kWriteFailed;
    case mapping::MapSaveFailureKind::kValidationFailed:
      return Runtime::MappingSaveFailureTag::kValidationFailed;
    case mapping::MapSaveFailureKind::kStorageSwitchFailed:
      return Runtime::MappingSaveFailureTag::kStorageSwitchFailed;
    case mapping::MapSaveFailureKind::kNone:
    default:
      return Runtime::MappingSaveFailureTag::kNone;
  }
}

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

const char* ToString(app::Runtime::MappingSaveTrigger trigger) {
  switch (trigger) {
    case app::Runtime::MappingSaveTrigger::kLoopClosure:
      return "loop_closure";
    case app::Runtime::MappingSaveTrigger::kAutoShutdown:
      return "auto_shutdown";
    case app::Runtime::MappingSaveTrigger::kExternalStop:
      return "external_stop";
    case app::Runtime::MappingSaveTrigger::kNone:
    default:
      return "none";
  }
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

const char* ToString(localization::RelocalizationPhase phase) {
  switch (phase) {
    case localization::RelocalizationPhase::kTracking:
      return "tracking";
    case localization::RelocalizationPhase::kLostLock:
      return "lost_lock";
    case localization::RelocalizationPhase::kSearching:
      return "searching";
    case localization::RelocalizationPhase::kStabilizing:
      return "stabilizing";
    default:
      return "unknown";
  }
}

const char* ToString(localization::RelocalizationRecoveryAction action) {
  switch (action) {
    case localization::RelocalizationRecoveryAction::kNone:
      return "none";
    case localization::RelocalizationRecoveryAction::kSlowSpin:
      return "slow_spin";
    case localization::RelocalizationRecoveryAction::kBackoffSpin:
      return "backoff_spin";
    case localization::RelocalizationRecoveryAction::kFixedPoseSweep:
      return "fixed_pose_sweep";
    default:
      return "unknown";
  }
}

const char* ToString(planning::GoalMode mode) {
  switch (mode) {
    case planning::GoalMode::kApproachCenter:
      return "approach_center";
    case planning::GoalMode::kCenterHold:
      return "center_hold";
    default:
      return "unknown";
  }
}

std::string CurrentMapLabel(bool mapping_active, const localization::StaticMap& map) {
  if (mapping_active) {
    return "warmup_live";
  }
  return map.version_label.empty() ? "unknown" : map.version_label;
}

std::string SafetyGateReasonSummary(const safety::SafetyResult& result) {
  if (result.has_event && !result.event.message.empty()) {
    return std::string(result.event.message);
  }
  if (result.gate_reason != safety::CommandGateReason::kNone) {
    return safety::ToString(result.gate_reason);
  }
  if (result.authority != safety::SafetyCommandAuthority::kAllow) {
    return safety::ToString(result.authority);
  }
  if (result.gate_limited) {
    return "command_limited";
  }
  return "none";
}

std::string CurrentDegradedSummary(const localization::LocalizationResult& localization_result,
                                   const planning::PlannerStatus& planner_status,
                                   const safety::SafetyResult& safety_result) {
  if (safety_result.state == safety::SafetyState::kHold ||
      safety_result.state == safety::SafetyState::kFailsafe) {
    return std::string("safety_") + safety::ToString(safety_result.state);
  }
  if (safety_result.authority == safety::SafetyCommandAuthority::kFreeze ||
      safety_result.authority == safety::SafetyCommandAuthority::kFailsafe) {
    return std::string("safety_") + safety::ToString(safety_result.authority);
  }
  if (localization_result.status.degraded_mode != "none") {
    return localization_result.status.degraded_mode;
  }
  if (planner_status.degraded_mode != "none") {
    return planner_status.degraded_mode;
  }
  if (safety_result.gate_limited) {
    return "command_gate_limited";
  }
  return "none";
}

std::string WhyRobotNotMoving(const fsm::NavFsmSnapshot& snapshot,
                              const localization::LocalizationResult& localization_result,
                              const planning::PlannerStatus& planner_status,
                              const safety::SafetyResult& safety_result) {
  if (snapshot.state == fsm::NavState::kBoot || snapshot.state == fsm::NavState::kSelfCheck ||
      snapshot.state == fsm::NavState::kIdle) {
    return "fsm_not_in_navigation_mode";
  }
  if (snapshot.failsafe_active) {
    return "fsm_failsafe";
  }
  if (safety_result.state == safety::SafetyState::kHold ||
      safety_result.state == safety::SafetyState::kFailsafe ||
      safety_result.gate_reason != safety::CommandGateReason::kNone) {
    return SafetyGateReasonSummary(safety_result);
  }
  if (!localization_result.status.pose_trusted) {
    return localization_result.status.rejection_reason;
  }
  if (!planner_status.global_plan_succeeded || !planner_status.local_plan_succeeded) {
    return planner_status.failure_reason;
  }
  if (planner_status.reached) {
    return "goal_reached_center_hold";
  }
  return "none";
}

std::string WhyRobotSlowing(const fsm::NavFsmSnapshot& snapshot,
                            const planning::PlannerStatus& planner_status,
                            const safety::SafetyResult& safety_result) {
  if (snapshot.recovery_active) {
    return "fsm_recovery_scale";
  }
  if (safety_result.authority == safety::SafetyCommandAuthority::kLimited) {
    return "safety_limited_cmd";
  }
  if (safety_result.gate_limited) {
    return "command_gate_limit";
  }
  if (planner_status.fallback_cmd_used) {
    return "path_follower_fallback";
  }
  if (planner_status.mode == planning::GoalMode::kCenterHold) {
    return "center_hold";
  }
  return "none";
}

void WritePointCloudPcd(const std::filesystem::path& path,
                        const std::vector<data::PointXYZI>& points) {
  std::ofstream output(path);
  output << "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n";
  output << "COUNT 1 1 1 1\nWIDTH " << points.size() << "\nHEIGHT 1\n";
  output << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size() << "\nDATA ascii\n";
  for (const auto& point : points) {
    output << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << "\n";
  }
}

void WriteDynamicSuppressionDebugSnapshot(
    const mapping::MapBuilder3D::DynamicSuppressionDebugSnapshot& dynamic_debug,
    const std::filesystem::path& output_dir) {
  {
    std::ofstream output(output_dir / "mapping_dynamic_debug.json");
    output << "{\n";
    output << "  \"frame_index\": " << dynamic_debug.frame_index << ",\n";
    output << "  \"suppression_enabled\": "
           << (dynamic_debug.suppression_enabled ? "true" : "false") << ",\n";
    output << "  \"pending_points\": " << dynamic_debug.pending_points.size() << ",\n";
    output << "  \"accepted_points\": " << dynamic_debug.accepted_points.size() << ",\n";
    output << "  \"rejected_points\": " << dynamic_debug.rejected_points.size() << ",\n";
    output << "  \"rejected_near_field\": " << dynamic_debug.rejected_near_field << ",\n";
    output << "  \"rejected_known_obstacle_mask\": "
           << dynamic_debug.rejected_known_obstacle_mask << ",\n";
    output << "  \"rejected_duplicate_voxel\": "
           << dynamic_debug.rejected_duplicate_voxel << ",\n";
    output << "  \"stale_pending_evictions\": " << dynamic_debug.stale_pending_evictions
           << "\n";
    output << "}\n";
  }

  WritePointCloudPcd(output_dir / "mapping_dynamic_pending.pcd",
                     dynamic_debug.pending_points);
  WritePointCloudPcd(output_dir / "mapping_dynamic_accepted.pcd",
                     dynamic_debug.accepted_points);
  WritePointCloudPcd(output_dir / "mapping_dynamic_rejected.pcd",
                     dynamic_debug.rejected_points);
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

bool MapArtifactsExist(const std::filesystem::path& directory) {
  return std::filesystem::exists(directory / "global_map.pcd") &&
         std::filesystem::exists(directory / "occupancy.bin") &&
         std::filesystem::exists(directory / "map_meta.json");
}

bool SafetyEventEquals(const data::SafetyEvent& left, const data::SafetyEvent& right) {
  return left.code == right.code && left.severity == right.severity &&
         left.message == right.message;
}

bool RefereeStartSignalActive(const data::RefereeState& referee) {
  return referee.is_online && referee.game_stage != 0U && referee.remaining_time_s > 0U;
}

data::Pose3f SpawnGoalPose(const config::SpawnConfig& spawn_config, common::TimePoint stamp) {
  data::Pose3f pose;
  pose.stamp = stamp;
  pose.reference_frame = tf::kMapFrame;
  pose.child_frame = tf::kBaseLinkFrame;
  pose.position.x = static_cast<float>(spawn_config.x_m);
  pose.position.y = static_cast<float>(spawn_config.y_m);
  pose.rpy.z = static_cast<float>(spawn_config.theta_rad);
  pose.is_valid = true;
  return pose;
}

bool ResolveCombatLocalizationConfig(const config::LoadedConfig& loaded_config,
                                     config::LocalizationConfig* localization_config,
                                     std::string* source_label) {
  if (localization_config == nullptr) {
    return false;
  }

  const auto active_dir =
      ResolveConfigAsset(loaded_config.config_dir, loaded_config.mapping.active_dir);
  if (MapArtifactsExist(active_dir)) {
    localization_config->global_map_pcd_path = (active_dir / "global_map.pcd").string();
    localization_config->occupancy_path = (active_dir / "occupancy.bin").string();
    localization_config->map_meta_path = (active_dir / "map_meta.json").string();
    if (source_label != nullptr) {
      *source_label = "active";
    }
    return true;
  }

  if (source_label != nullptr) {
    *source_label = "missing";
  }
  return false;
}

bool ManualWarmupSelected(int manual_mode_selector) { return manual_mode_selector == 0; }

bool ManualCombatSelected(int manual_mode_selector) { return manual_mode_selector == 1; }

bool AutomaticFsmModeSelected(int manual_mode_selector) { return manual_mode_selector < 0; }

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
    R"({"type":"object","properties":{"processed_frames":{"type":"integer"},"accumulated_points":{"type":"integer"},"processing_latency_ns":{"type":"integer"},"pose_source":{"type":"string"},"frontend_score":{"type":"number"},"frontend_iterations":{"type":"integer"},"frontend_converged":{"type":"boolean"},"frontend_fallback":{"type":"boolean"},"frontend_latency_ns":{"type":"integer"},"frontend_reference_points":{"type":"integer"},"external_x":{"type":"number"},"external_y":{"type":"number"},"external_yaw":{"type":"number"},"predicted_x":{"type":"number"},"predicted_y":{"type":"number"},"predicted_yaw":{"type":"number"},"optimized_x":{"type":"number"},"optimized_y":{"type":"number"},"optimized_yaw":{"type":"number"},"save_trigger":{"type":"string"},"current_waypoint_index":{"type":"integer"},"waypoint_count":{"type":"integer"}},"required":["processed_frames","accumulated_points","processing_latency_ns","pose_source","frontend_score","frontend_iterations","frontend_converged","frontend_fallback","frontend_latency_ns","frontend_reference_points","external_x","external_y","external_yaw","predicted_x","predicted_y","predicted_yaw","optimized_x","optimized_y","optimized_yaw","save_trigger","current_waypoint_index","waypoint_count"]})";
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

std::string BuildRuntimeMappingStatusJson(
    const mapping::MappingEngine& mapping_engine, const mapping::WaypointManager& waypoint_manager,
    app::Runtime::MappingSaveTrigger save_trigger) {
  const auto result = mapping_engine.LatestResult();
  std::ostringstream output;
  output << "{\n";
  output << "  \"processed_frames\": " << result.processed_frames << ",\n";
  output << "  \"accumulated_points\": " << result.accumulated_points << ",\n";
  output << "  \"processing_latency_ns\": " << result.processing_latency_ns << ",\n";
  output << "  \"pose_source\": \"" << result.pose_source << "\",\n";
  output << "  \"frontend_score\": " << result.frontend_score << ",\n";
  output << "  \"frontend_iterations\": " << result.frontend_iterations << ",\n";
  output << "  \"frontend_converged\": "
         << (result.frontend_converged ? "true" : "false") << ",\n";
  output << "  \"frontend_fallback\": "
         << (result.frontend_fallback ? "true" : "false") << ",\n";
  output << "  \"frontend_latency_ns\": " << result.frontend_latency_ns << ",\n";
  output << "  \"frontend_reference_points\": " << result.frontend_reference_points << ",\n";
  output << "  \"external_x\": " << result.external_pose.position.x << ",\n";
  output << "  \"external_y\": " << result.external_pose.position.y << ",\n";
  output << "  \"external_yaw\": " << result.external_pose.rpy.z << ",\n";
  output << "  \"predicted_x\": " << result.predicted_pose.position.x << ",\n";
  output << "  \"predicted_y\": " << result.predicted_pose.position.y << ",\n";
  output << "  \"predicted_yaw\": " << result.predicted_pose.rpy.z << ",\n";
  output << "  \"optimized_x\": " << result.map_to_base.position.x << ",\n";
  output << "  \"optimized_y\": " << result.map_to_base.position.y << ",\n";
  output << "  \"optimized_yaw\": " << result.map_to_base.rpy.z << ",\n";
  output << "  \"save_trigger\": \"" << ToString(save_trigger) << "\",\n";
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

data::LidarFrame BuildDebugScanSnapshot(const data::LidarFrame& source,
                                        std::size_t max_points) {
  data::LidarFrame snapshot;
  snapshot.stamp = source.stamp;
  snapshot.scan_begin_stamp = source.scan_begin_stamp;
  snapshot.scan_end_stamp = source.scan_end_stamp;
  snapshot.frame_id = source.frame_id;
  snapshot.frame_index = source.frame_index;
  snapshot.is_deskewed = source.is_deskewed;
  auto& points = snapshot.points.MutableView();
  points = DownsampleScenePoints(source.points.view(), max_points);
  return snapshot;
}

void EnsureRuntimeDebugDirectories(const std::filesystem::path& output_dir) {
  std::filesystem::create_directories(output_dir);
  std::filesystem::create_directories("logs/watchdog");
  std::filesystem::create_directories("logs/crash");
}

void WriteTextFile(const std::filesystem::path& path, const std::string& text) {
  std::ofstream output(path, std::ios::out | std::ios::trunc);
  output << text;
}

void WriteTextFileIfChanged(const std::filesystem::path& path, const std::string& text,
                            std::string* last_text) {
  if (last_text != nullptr && *last_text == text) {
    return;
  }
  WriteTextFile(path, text);
  if (last_text != nullptr) {
    *last_text = text;
  }
}

std::string BuildHeartbeatJson(common::TimePoint stamp, fsm::NavState state) {
  std::ostringstream output;
  output << "{\n  \"stamp_ns\": " << common::ToNanoseconds(stamp)
         << ",\n  \"state\": \"" << fsm::ToString(state) << "\"\n}\n";
  return output.str();
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
                        const planning::RecoveryPlannerStatus& recovery_status,
                        const safety::SafetyResult& safety_result,
                        const std::filesystem::path& output_dir) {
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
    output << "  \"map_version\": \"" << localization.static_map().version_label << "\",\n";
    output << "  \"matcher\": \"" << result.matcher_name << "\",\n";
    output << "  \"score\": " << result.status.match_score << ",\n";
    output << "  \"iterations\": " << result.status.iterations << ",\n";
    output << "  \"converged\": " << (result.status.converged ? "true" : "false") << ",\n";
    output << "  \"failures\": " << result.status.consecutive_failures << ",\n";
    output << "  \"jump_m\": " << result.status.pose_jump_m << ",\n";
    output << "  \"jump_yaw_rad\": " << result.status.yaw_jump_rad << ",\n";
    output << "  \"rejection_reason\": \"" << result.status.rejection_reason << "\",\n";
    output << "  \"degraded_mode\": \"" << result.status.degraded_mode << "\",\n";
    output << "  \"relocalization_active\": "
           << (result.relocalization.active ? "true" : "false") << ",\n";
    output << "  \"relocalization_attempted\": "
           << (result.relocalization.attempted ? "true" : "false") << ",\n";
    output << "  \"relocalization_succeeded\": "
           << (result.relocalization.succeeded ? "true" : "false") << ",\n";
    output << "  \"relocalization_phase\": \"" << ToString(result.relocalization.phase)
           << "\",\n";
    output << "  \"relocalization_recovery_action\": \""
           << ToString(result.relocalization.recovery_action) << "\",\n";
    output << "  \"relocalization_ambiguity_rejected\": "
           << (result.relocalization.ambiguity_rejected ? "true" : "false") << ",\n";
    output << "  \"relocalization_stabilization_failed\": "
           << (result.relocalization.stabilization_failed ? "true" : "false") << ",\n";
    output << "  \"relocalization_secondary_check_performed\": "
           << (result.relocalization.secondary_check_performed ? "true" : "false") << ",\n";
    output << "  \"relocalization_secondary_check_passed\": "
           << (result.relocalization.secondary_check_passed ? "true" : "false") << ",\n";
    output << "  \"relocalization_map_to_odom_blending\": "
           << (result.relocalization.map_to_odom_blending_active ? "true" : "false") << ",\n";
    output << "  \"relocalization_candidates\": " << result.relocalization.candidates_tested
           << ",\n";
    output << "  \"relocalization_failed_attempts\": " << result.relocalization.failed_attempts
           << ",\n";
    output << "  \"relocalization_lost_lock_elapsed_ns\": "
           << result.relocalization.lost_lock_elapsed_ns << ",\n";
    output << "  \"relocalization_stabilization_observations\": "
           << result.relocalization.stabilization_observations << ",\n";
    output << "  \"relocalization_stabilization_elapsed_ns\": "
           << result.relocalization.stabilization_elapsed_ns << ",\n";
    output << "  \"relocalization_score\": " << result.relocalization.best_score << "\n";
    output << "}\n";
  }
  {
    std::ofstream output(output_dir / "planner_status.json");
    output << "{\n";
    output << "  \"mode\": \"" << ToString(planner_status.mode) << "\",\n";
    output << "  \"map_version\": \"" << planner_status.map_version << "\",\n";
    output << "  \"distance_to_goal_m\": " << planner_status.distance_to_goal_m << ",\n";
    output << "  \"distance_to_center_m\": " << planner_status.distance_to_center_m << ",\n";
    output << "  \"yaw_error_rad\": " << planner_status.yaw_error_rad << ",\n";
    output << "  \"global_plan_succeeded\": "
           << (planner_status.global_plan_succeeded ? "true" : "false") << ",\n";
    output << "  \"local_plan_succeeded\": "
           << (planner_status.local_plan_succeeded ? "true" : "false") << ",\n";
    output << "  \"reached\": " << (planner_status.reached ? "true" : "false") << ",\n";
    output << "  \"settling\": " << (planner_status.settling ? "true" : "false") << ",\n";
    output << "  \"hold_drifted\": " << (planner_status.hold_drifted ? "true" : "false")
           << ",\n";
    output << "  \"fallback_cmd_used\": "
           << (planner_status.fallback_cmd_used ? "true" : "false") << ",\n";
    output << "  \"temporary_goal_active\": "
           << (planner_status.temporary_goal_active ? "true" : "false") << ",\n";
    output << "  \"clearance_weight_scale\": " << planner_status.clearance_weight_scale
           << ",\n";
    output << "  \"degraded_mode\": \"" << planner_status.degraded_mode << "\",\n";
    output << "  \"failure_reason\": \"" << planner_status.failure_reason << "\",\n";
    output << "  \"hold_frames_in_goal\": " << planner_status.hold_frames_in_goal << ",\n";
    output << "  \"hold_settle_elapsed_ns\": " << planner_status.hold_settle_elapsed_ns
           << ",\n";
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
    output << "    \"dynamic_nearest_predicted_distance\": "
           << planner_status.dwa_score.dynamic_nearest_predicted_distance << ",\n";
    output << "    \"dynamic_risk_05\": " << planner_status.dwa_score.dynamic_risk_05
           << ",\n";
    output << "    \"dynamic_risk_10\": " << planner_status.dwa_score.dynamic_risk_10 << ",\n";
    output << "    \"dynamic_high_risk_penalty\": "
           << planner_status.dwa_score.dynamic_high_risk_penalty << ",\n";
    output << "    \"dynamic_crossing_penalty\": "
           << planner_status.dwa_score.dynamic_crossing_penalty << ",\n";
    output << "    \"dynamic_max_risk_level\": "
           << planner_status.dwa_score.dynamic_max_risk_level << ",\n";
    output << "    \"total\": " << planner_status.dwa_score.total_score << "\n";
    output << "  }\n";
    output << "}\n";
  }
  {
    std::ofstream output(output_dir / "recovery_status.json");
    output << "{\n";
    output << "  \"active\": " << (recovery_status.active ? "true" : "false") << ",\n";
    output << "  \"tier\": \"" << fsm::ToString(recovery_status.tier) << "\",\n";
    output << "  \"cause\": \"" << fsm::ToString(recovery_status.cause) << "\",\n";
    output << "  \"action\": \"" << fsm::ToString(recovery_status.action) << "\",\n";
    output << "  \"strategy\": \"" << recovery_status.strategy << "\",\n";
    output << "  \"detail\": \"" << recovery_status.detail << "\",\n";
    output << "  \"failure_streak\": " << recovery_status.failure_streak << ",\n";
    output << "  \"cycles_in_tier\": " << recovery_status.cycles_in_tier << ",\n";
    output << "  \"clearance_weight_scale\": " << recovery_status.clearance_weight_scale
           << ",\n";
    output << "  \"requires_relocalization\": "
           << (recovery_status.requires_relocalization ? "true" : "false") << ",\n";
    output << "  \"cooldown_active\": "
           << (recovery_status.cooldown_active ? "true" : "false") << ",\n";
    output << "  \"complete\": " << (recovery_status.complete ? "true" : "false") << ",\n";
    output << "  \"exhausted\": " << (recovery_status.exhausted ? "true" : "false") << ",\n";
    output << "  \"temporary_goal_valid\": "
           << (recovery_status.temporary_goal_valid ? "true" : "false") << ",\n";
    output << "  \"temporary_goal\": {\"x\": " << recovery_status.temporary_goal.position.x
           << ", \"y\": " << recovery_status.temporary_goal.position.y << ", \"yaw\": "
           << recovery_status.temporary_goal.rpy.z << "},\n";
    output << "  \"cmd\": {\"vx\": " << recovery_status.command.vx_mps << ", \"vy\": "
           << recovery_status.command.vy_mps << ", \"wz\": " << recovery_status.command.wz_radps
           << ", \"brake\": " << (recovery_status.command.brake ? "true" : "false") << "}\n";
    output << "}\n";
  }
  {
    std::ofstream output(output_dir / "safety_gate_status.json");
    output << "{\n";
    output << "  \"state\": \"" << safety::ToString(safety_result.state) << "\",\n";
    output << "  \"authority\": \"" << safety::ToString(safety_result.authority) << "\",\n";
    output << "  \"gate_reason\": \"" << safety::ToString(safety_result.gate_reason) << "\",\n";
    output << "  \"gate_limited\": " << (safety_result.gate_limited ? "true" : "false")
           << ",\n";
    output << "  \"event_message\": \"" << safety_result.event.message << "\",\n";
    output << "  \"communication_ok\": " << (safety_result.communication_ok ? "true" : "false")
           << ",\n";
    output << "  \"chassis_feedback_ok\": "
           << (safety_result.chassis_feedback_ok ? "true" : "false") << ",\n";
    output << "  \"costmap_valid\": " << (safety_result.costmap_valid ? "true" : "false")
           << ",\n";
    output << "  \"costmap_fresh\": " << (safety_result.costmap_fresh ? "true" : "false")
           << ",\n";
    output << "  \"localization_quality_ok\": "
           << (safety_result.localization_quality_ok ? "true" : "false") << ",\n";
    output << "  \"planner_status_ok\": "
           << (safety_result.planner_status_ok ? "true" : "false") << ",\n";
    output << "  \"mode_transition_active\": "
           << (safety_result.mode_transition_active ? "true" : "false") << ",\n";
    output << "  \"planner_cmd_timed_out\": "
           << (safety_result.planner_cmd_timed_out ? "true" : "false") << ",\n";
    output << "  \"obstacle_too_close\": "
           << (safety_result.obstacle_too_close ? "true" : "false") << ",\n";
    output << "  \"allow_cmd\": {\"vx\": " << safety_result.allow_cmd.vx_mps << ", \"vy\": "
           << safety_result.allow_cmd.vy_mps << ", \"wz\": " << safety_result.allow_cmd.wz_radps
           << ", \"brake\": " << (safety_result.allow_cmd.brake ? "true" : "false") << "},\n";
    output << "  \"limited_cmd\": {\"vx\": " << safety_result.limited_cmd.vx_mps << ", \"vy\": "
           << safety_result.limited_cmd.vy_mps << ", \"wz\": "
           << safety_result.limited_cmd.wz_radps << ", \"brake\": "
           << (safety_result.limited_cmd.brake ? "true" : "false") << "},\n";
    output << "  \"freeze_cmd\": {\"vx\": " << safety_result.freeze_cmd.vx_mps << ", \"vy\": "
           << safety_result.freeze_cmd.vy_mps << ", \"wz\": " << safety_result.freeze_cmd.wz_radps
           << ", \"brake\": " << (safety_result.freeze_cmd.brake ? "true" : "false") << "},\n";
    output << "  \"failsafe_cmd\": {\"vx\": " << safety_result.failsafe_cmd.vx_mps
           << ", \"vy\": " << safety_result.failsafe_cmd.vy_mps << ", \"wz\": "
           << safety_result.failsafe_cmd.wz_radps << ", \"brake\": "
           << (safety_result.failsafe_cmd.brake ? "true" : "false") << "},\n";
    output << "  \"final_cmd\": {\"vx\": " << safety_result.gated_cmd.vx_mps << ", \"vy\": "
           << safety_result.gated_cmd.vy_mps << ", \"wz\": " << safety_result.gated_cmd.wz_radps
           << ", \"brake\": " << (safety_result.gated_cmd.brake ? "true" : "false") << "}\n";
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
                               app::Runtime::MappingSaveTrigger save_trigger,
                               const std::filesystem::path& output_dir) {
  const auto global_points = mapping_engine.GlobalPointCloud();
  const auto mapping_result = mapping_engine.LatestResult();
  const auto dynamic_debug = mapping_engine.LatestDynamicSuppressionDebug();
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
    output << "  \"processing_latency_ns\": " << mapping_result.processing_latency_ns << ",\n";
    output << "  \"save_trigger\": \"" << ToString(save_trigger) << "\"\n";
    output << "}\n";
  }
  {
    WritePointCloudPcd(output_dir / "global_map.pcd", global_points);
  }
  WriteDynamicSuppressionDebugSnapshot(dynamic_debug, output_dir);
}

void WriteFsmDebugSnapshot(const std::string& fsm_status_json,
                           const std::string& fsm_event_json,
                           const std::filesystem::path& output_dir,
                           std::string* last_fsm_status_json,
                           std::string* last_fsm_event_json) {
  WriteTextFileIfChanged(output_dir / "fsm_status.json", fsm_status_json, last_fsm_status_json);
  WriteTextFileIfChanged(output_dir / "fsm_event.json", fsm_event_json, last_fsm_event_json);
}

void WriteSafetyEventDebugSnapshot(const data::SafetyEvent& event,
                                   const std::filesystem::path& output_dir) {
  WriteTextFile(output_dir / "safety_event.json", BuildSafetyEventJson(event));
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
      {utils::ParseLogLevel(loaded_config_.system.log_level),
       loaded_config_.system.log_file_path, true, loaded_config_.system.console_io_only,
       static_cast<std::size_t>(std::max(1, loaded_config_.system.log_max_queue_size))});

  status = ValidateLoadedConfig();
  if (!status.ok()) {
    return status;
  }

  EnsureRuntimeDebugDirectories(RuntimeDebugOutputDir());

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
    if (!waiting_for_mapping_save && mapping_mode_ &&
        !map_saved_.load(std::memory_order_acquire) &&
        mapping_loop_save_requested_.load(std::memory_order_acquire)) {
      utils::LogInfo("runtime", "loop closure accepted, requesting MODE_SAVE");
      mapping_save_requested_.store(true, std::memory_order_release);
      mapping_save_trigger_.store(static_cast<int>(MappingSaveTrigger::kLoopClosure),
                                  std::memory_order_release);
      waiting_for_mapping_save = true;
      mapping_save_deadline = common::Now() + std::chrono::milliseconds(1500);
    }

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
        } else if (mapping_mode_ && !map_saved_.load(std::memory_order_acquire) &&
                   MappingSaveEligible()) {
          if (!waiting_for_mapping_save) {
            utils::LogInfo("runtime", "auto shutdown reached, requesting MODE_SAVE");
            mapping_save_requested_.store(true, std::memory_order_release);
            mapping_save_trigger_.store(static_cast<int>(MappingSaveTrigger::kAutoShutdown),
                                        std::memory_order_release);
            waiting_for_mapping_save = true;
            mapping_save_deadline = common::Now() + std::chrono::milliseconds(1500);
          }
        } else if (mapping_mode_ && !map_saved_.load(std::memory_order_acquire)) {
          utils::LogInfo("runtime",
                         "auto shutdown reached before start signal; stopping without saving");
          break;
        } else {
          utils::LogInfo("runtime", "auto shutdown timeout reached");
          break;
        }
      }
    }
    common::SleepFor(std::chrono::milliseconds(50));
  }

  if ((external_stop.load() || stop_requested_.load(std::memory_order_acquire)) && mapping_mode_ &&
      !map_saved_.load(std::memory_order_acquire) && MappingSaveEligible()) {
    mapping_save_requested_.store(true, std::memory_order_release);
    mapping_save_trigger_.store(static_cast<int>(MappingSaveTrigger::kExternalStop),
                                std::memory_order_release);
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
  const bool manual_warmup = ManualWarmupSelected(manual_mode_selector);
  const bool manual_combat = ManualCombatSelected(manual_mode_selector);
  const bool automatic_fsm = AutomaticFsmModeSelected(manual_mode_selector);
  config::LocalizationConfig combat_localization_config = loaded_config_.localization;
  std::string combat_map_source;
  const bool combat_map_available = ResolveCombatLocalizationConfig(
      loaded_config_, &combat_localization_config, &combat_map_source);
  mapping_mode_ = loaded_config_.system.bringup_mode == "none" &&
                  (manual_warmup || (automatic_fsm && !combat_map_available));
  combat_mode_requested_ = loaded_config_.system.bringup_mode == "none" &&
                           (manual_combat || (automatic_fsm && combat_map_available));
  nav_fsm_ = fsm::NavFsm();
  mapping_save_requested_.store(false, std::memory_order_release);
  mapping_loop_save_requested_.store(false, std::memory_order_release);
  map_saved_.store(false, std::memory_order_release);
  mapping_save_failure_tag_.store(static_cast<int>(MappingSaveFailureTag::kNone),
                                  std::memory_order_release);
  mapping_save_trigger_.store(static_cast<int>(MappingSaveTrigger::kNone),
                              std::memory_order_release);
  combat_pipeline_ready_.store(false, std::memory_order_release);
  combat_map_unavailable_.store(false, std::memory_order_release);
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
  preprocess_config.expected_input_points =
      mapping_mode_
          ? static_cast<std::size_t>(std::max(32, loaded_config_.mapping.synthetic_points_per_frame))
          : static_cast<std::size_t>(720);
  perception::LocalCostmapConfig local_costmap_config;
  local_costmap_config.width = loaded_config_.costmap.width;
  local_costmap_config.height = loaded_config_.costmap.height;
  local_costmap_config.resolution_m =
      static_cast<float>(loaded_config_.costmap.resolution_m);
  local_costmap_config.obstacle_layer_height_m =
      static_cast<float>(loaded_config_.costmap.obstacle_layer_height_m);
  local_costmap_config.inflation_radius_m =
      static_cast<float>(loaded_config_.costmap.inflation_radius_m);
  local_costmap_config.dynamic_obstacle_inflation_m =
      static_cast<float>(loaded_config_.costmap.dynamic_obstacle_inflation_m);
  status = local_costmap_builder_.Configure(local_costmap_config);
  if (!status.ok()) {
    return status;
  }
  perception::MotConfig mot_config;
  mot_config.cluster_tolerance_m = static_cast<float>(loaded_config_.mot.cluster_tolerance_m);
  mot_config.min_cluster_points = loaded_config_.mot.min_cluster_points;
  mot_config.max_cluster_points = loaded_config_.mot.max_cluster_points;
  mot_config.roi_radius_m = static_cast<float>(loaded_config_.mot.roi_radius_m);
  mot_config.reduced_roi_radius_m =
      static_cast<float>(loaded_config_.mot.reduced_roi_radius_m);
  mot_config.association_distance_m =
      static_cast<float>(loaded_config_.mot.association_distance_m);
  mot_config.max_missed_frames = loaded_config_.mot.max_missed_frames;
  mot_config.process_noise = static_cast<float>(loaded_config_.mot.process_noise);
  mot_config.measurement_noise = static_cast<float>(loaded_config_.mot.measurement_noise);
  mot_config.initial_confidence = static_cast<float>(loaded_config_.mot.initial_confidence);
  mot_config.confirmation_confidence =
      static_cast<float>(loaded_config_.mot.confirmation_confidence);
  mot_config.clustering_budget_ms = loaded_config_.mot.clustering_budget_ms;
  status = mot_manager_.Configure(mot_config);
  if (!status.ok()) {
    return status;
  }
  status = recovery_planner_.Configure(loaded_config_.planner);
  if (!status.ok()) {
    return status;
  }
  MatchModeConfig match_mode_config;
  match_mode_config.enabled = loaded_config_.system.match_mode_enabled;
  match_mode_config.low_hp_threshold = loaded_config_.system.match_low_hp_threshold;
  match_mode_config.spawn_wait_ms = loaded_config_.system.match_spawn_wait_ms;
  match_mode_config.spawn_reach_tolerance_m =
      static_cast<float>(loaded_config_.planner.center_radius_m);
  match_mode_config.spawn_pose =
      SpawnGoalPose(loaded_config_.spawn, common::Now());
  match_mode_config.center_reach_tolerance_m =
      static_cast<float>(loaded_config_.planner.center_radius_m);
  match_mode_config.center_pose.stamp = common::Now();
  match_mode_config.center_pose.reference_frame = tf::kMapFrame;
  match_mode_config.center_pose.child_frame = tf::kBaseLinkFrame;
  match_mode_config.center_pose.position.x =
      static_cast<float>(loaded_config_.planner.center_goal_x_m);
  match_mode_config.center_pose.position.y =
      static_cast<float>(loaded_config_.planner.center_goal_y_m);
  match_mode_config.center_pose.is_valid = true;
  status = match_mode_controller_.Configure(match_mode_config);
  if (!status.ok()) {
    return status;
  }
  latest_match_mode_decision_.Publish(match_mode_controller_.LatestDecision());
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

  if (combat_mode_requested_) {
    if (!combat_map_available) {
      combat_map_unavailable_.store(true, std::memory_order_release);
      utils::LogWarn("runtime", "no usable combat map found; runtime will enter failsafe");
      status = common::Status::Ok();
    } else {
      combat_map_unavailable_.store(false, std::memory_order_release);
      status = InitializeCombatPipeline(combat_localization_config, loaded_config_.spawn);
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
  status = safety_manager_.Configure(loaded_config_.safety);
  if (!status.ok()) {
    return status;
  }

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
  if (!workers_.empty()) {
    StopThreads();
  } else {
    stop_requested_.store(false);
  }

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
    bool made_progress = false;
    const auto wait_begin = common::Now();
    const bool heartbeat_due = stm32_available_.load(std::memory_order_acquire) &&
                               wait_begin >= next_heartbeat_due;
    const bool cmd_due =
        stm32_available_.load(std::memory_order_acquire) &&
        safety_cmd_.generation() != last_sent_cmd_generation;
    if (!heartbeat_due && !cmd_due) {
      if (loaded_config_.sensors.lidar_enabled &&
          (loaded_config_.sensors.lidar_source == "unitree_sdk" ||
           loaded_config_.sensors.lidar_source == "synthetic" ||
           loaded_config_.sensors.imu_source == "lidar_internal")) {
        made_progress = lidar_driver_.WaitForData(std::chrono::milliseconds(2));
      }
      if (!made_progress && stm32_available_.load(std::memory_order_acquire)) {
        made_progress = stm32_bridge_.WaitForRx(std::chrono::milliseconds(2));
      }
    }
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
        made_progress = true;
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
        made_progress = true;
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
    } else {
      if (now >= next_heartbeat_due) {
        const auto heartbeat_status = stm32_bridge_.SendHeartbeat();
        if (!heartbeat_status.ok()) {
          utils::LogWarn("driver", heartbeat_status.message);
        } else {
          made_progress = true;
        }
        next_heartbeat_due = now + std::chrono::milliseconds(200);
      }

      if (const auto cmd_generation = safety_cmd_.generation();
          cmd_generation != last_sent_cmd_generation) {
        const auto safe_cmd = safety_cmd_.ReadSnapshot();
        const auto send_status = stm32_bridge_.SendChassisCmd(safe_cmd);
        if (send_status.ok()) {
          last_sent_cmd_generation = cmd_generation;
          made_progress = true;
        } else {
          utils::LogWarn("driver", send_status.message);
        }
      }

      if (stm32_bridge_.WaitForRx(std::chrono::milliseconds(0))) {
        stm32_bridge_.SpinOnce();
        made_progress = true;
        if (auto odom = stm32_bridge_.TakeOdomState(); odom.has_value()) {
          stm32_odom_.Publish(*odom);
          last_stm32_rx_ns_.store(common::ToNanoseconds(odom->stamp), std::memory_order_release);
          made_progress = true;
        }
        if (auto referee = stm32_bridge_.TakeRefereeState(); referee.has_value()) {
          referee_state_.Publish(*referee);
          last_stm32_rx_ns_.store(common::ToNanoseconds(referee->stamp),
                                  std::memory_order_release);
          made_progress = true;
        }
      }
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

    if (!made_progress) {
      common::SleepFor(std::chrono::milliseconds(2));
    }
  }
}

void Runtime::SyncThreadMain() {
  while (!stop_requested_.load(std::memory_order_acquire)) {
    if (!sensor_sync_.WaitForLidarFrame(std::chrono::milliseconds(10))) {
      continue;
    }
    const auto status = sensor_sync_.ProcessOnce();
    if (!status.ok()) {
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
      } else {
        pending_mapping_frames_.fetch_add(1, std::memory_order_release);
        mapping_queue_wait_cv_.notify_one();
      }
    }

    const bool bringup_mode = PerceptionBringupMode();
    if (fsm_snapshot.mapping_active ||
        (fsm_snapshot.localization_active &&
         combat_pipeline_ready_.load(std::memory_order_acquire)) ||
        bringup_mode) {
      if (!bringup_mode) {
        if (fsm_snapshot.localization_active &&
            combat_pipeline_ready_.load(std::memory_order_acquire)) {
          sync::SyncedFrameHandle localization_frame;
          if (!CloneSyncedFrame(*synced_handle->get(), &localization_frame)) {
            utils::LogWarn("sync", "fanout pool exhausted for localization");
          } else {
            localization_.EnqueueFrame(std::move(localization_frame));
          }
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
      auto external_pose =
          odom.stamp != common::TimePoint{} ? MappingPoseFromOdom(odom) : mapping_pose_.ReadSnapshot();
      if (!external_pose.is_valid) {
        external_pose = MakeMapPose(common::Now(), static_cast<float>(loaded_config_.spawn.x_m),
                                    static_cast<float>(loaded_config_.spawn.y_m),
                                    static_cast<float>(loaded_config_.spawn.theta_rad));
        mapping_pose_.Publish(external_pose);
      }
      sync::SyncedFrameHandle handle;
      if (mapping_queue_.try_pop(&handle)) {
        const auto pending = pending_mapping_frames_.load(std::memory_order_acquire);
        if (pending > 0U) {
          pending_mapping_frames_.fetch_sub(1, std::memory_order_acq_rel);
        }
        const auto known_dynamic_obstacles = mot_manager_.LatestObstacles();
        const auto status =
            mapping_engine_.Update(*handle.get(), external_pose, known_dynamic_obstacles.obstacles);
        handle.reset();
        if (status.ok()) {
          if (loaded_config_.mapping.save_on_loop_closure_success &&
              mapping_engine_.LatestLoopCorrection().accepted) {
            mapping_loop_save_requested_.store(true, std::memory_order_release);
          }
          mapping_pose_.Publish(mapping_engine_.LatestResult().map_to_base);
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
      if (fsm_snapshot.mapping_active) {
        std::unique_lock<std::mutex> lock(mapping_queue_wait_mutex_);
        mapping_queue_wait_cv_.wait_for(lock, std::chrono::milliseconds(10), [this]() {
          return pending_mapping_frames_.load(std::memory_order_acquire) > 0U ||
                 stop_requested_.load(std::memory_order_acquire);
        });
      } else if (fsm_snapshot.localization_active &&
                 combat_pipeline_ready_.load(std::memory_order_acquire)) {
        localization_.WaitForInput(std::chrono::milliseconds(10));
      } else {
        common::SleepFor(std::chrono::milliseconds(2));
      }
    }
  }
}

void Runtime::PerceptionThreadMain() {
  while (!stop_requested_.load(std::memory_order_acquire)) {
    const auto fsm_snapshot = fsm_snapshot_.ReadSnapshot();
    const bool bringup_mode = PerceptionBringupMode();
    const bool mapping_mode = fsm_snapshot.mapping_active;
    if ((!fsm_snapshot.localization_active ||
         !combat_pipeline_ready_.load(std::memory_order_acquire)) &&
        !mapping_mode &&
        !bringup_mode) {
      common::SleepFor(std::chrono::milliseconds(10));
      continue;
    }

    const auto preprocess_status = preprocess_.ProcessOnce();
    if (!preprocess_status.ok()) {
      preprocess_.WaitForInput(std::chrono::milliseconds(10));
    }

    auto filtered_frame = preprocess_.TryPopFilteredFrameHandle();
    if (!filtered_frame.has_value()) {
      continue;
    }

    auto pose = mapping_mode ? mapping_pose_.ReadSnapshot() : localization_.LatestResult().map_to_base;
    if ((bringup_mode || mapping_mode) && !pose.is_valid) {
      pose = MakeMapPose(filtered_frame->get()->stamp,
                         static_cast<float>(loaded_config_.spawn.x_m),
                         static_cast<float>(loaded_config_.spawn.y_m),
                         static_cast<float>(loaded_config_.spawn.theta_rad));
    }
    if (pose.is_valid) {
      if (loaded_config_.debug.websocket_enabled) {
        latest_filtered_scan_.Publish(
            BuildDebugScanSnapshot(*filtered_frame->get(),
                                   static_cast<std::size_t>(std::max(
                                       1, loaded_config_.debug.debug_scan_max_points))));
      }
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
        const auto referee = referee_state_.ReadSnapshot();
        const auto planner_status = planner_.LatestStatus();
        const auto match_decision =
            match_mode_controller_.Update(common::Now(), referee, pose, planner_status);
        latest_match_mode_decision_.Publish(match_decision);
        planning::PlanningOverrides planning_overrides;
        const planning::PlanningOverrides* overrides = nullptr;
        if (fsm_snapshot.recovery_active) {
          const auto recovery_status = latest_recovery_status_.ReadSnapshot();
          planning_overrides.clearance_weight_scale =
              std::max(1.0F, recovery_status.clearance_weight_scale);
          planning_overrides.temporary_goal = recovery_status.temporary_goal;
          planning_overrides.temporary_goal_valid = recovery_status.temporary_goal_valid;
          overrides = &planning_overrides;
        }
        if (match_decision.override_goal) {
          planner_.PlanAndPublishToGoal(pose, match_decision.goal_pose, costmap,
                                        obstacles.obstacles, overrides);
        } else {
          planner_.PlanAndPublish(pose, costmap, obstacles.obstacles, overrides);
        }
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
  data::SafetyEvent last_event = latest_safety_event_.ReadSnapshot();
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
        if (!combat_pipeline_ready_.load(std::memory_order_acquire)) {
          const auto fallback_status = InitializeCombatPipelineFromSavedMap();
          if (fallback_status.ok()) {
            utils::LogWarn("fsm", "mapping save failed, fell back to existing map");
          }
        }
        const auto failure_tag = static_cast<MappingSaveFailureTag>(
            mapping_save_failure_tag_.load(std::memory_order_acquire));
        switch (failure_tag) {
          case MappingSaveFailureTag::kWriteFailed:
            utils::LogWarn("fsm",
                           std::string("mapping save write failed: ") + save_status.message);
            break;
          case MappingSaveFailureTag::kValidationFailed:
            utils::LogWarn("fsm",
                           std::string("mapping validation failed: ") + save_status.message);
            break;
          case MappingSaveFailureTag::kStorageSwitchFailed:
            utils::LogWarn("fsm",
                           std::string("mapping storage switch failed: ") + save_status.message);
            break;
          case MappingSaveFailureTag::kNone:
          default:
            utils::LogWarn("fsm", std::string("mapping save failed: ") + save_status.message);
            break;
        }
      }
    }

    const auto safe_cmd_candidate = SelectCommandCandidate(snapshot, now);
    const auto localization_result = localization_.LatestResult();
    const auto planner_status = planner_.LatestStatus();
    const auto costmap = local_costmap_builder_.LatestCostmap();
    const auto obstacles = mot_manager_.LatestObstacles();
    const auto odom_snapshot = stm32_odom_.ReadSnapshot();
    const auto current_pose =
        snapshot.mapping_active ? mapping_pose_.ReadSnapshot() : localization_result.map_to_base;
    const bool warmup_start_authorized = RefereeStartAuthorized(true);
    const bool combat_start_authorized = RefereeStartAuthorized(false);

    safety::SafetyInput safety_input;
    safety_input.stamp = now;
    safety_input.start_signal_active =
        snapshot.mapping_active ? warmup_start_authorized
                                : (snapshot.localization_active || snapshot.center_hold_active ||
                                   snapshot.recovery_active || combat_mode_requested_)
                                      ? combat_start_authorized
                                      : (warmup_start_authorized && combat_start_authorized);
    safety_input.arming_ready =
        snapshot.mapping_active
            ? current_pose.is_valid
            : (combat_pipeline_ready_.load(std::memory_order_acquire) &&
               localization_result.status.map_loaded && current_pose.is_valid);
    safety_input.navigation_requested =
        snapshot.mapping_active || snapshot.localization_active || snapshot.center_hold_active ||
        snapshot.recovery_active;
    safety_input.costmap_required = !snapshot.mapping_active;
    safety_input.mode_transition_active = snapshot.state != snapshot.previous_state;
    safety_input.force_failsafe = snapshot.failsafe_active;
    safety_input.planner_path_available =
        snapshot.mapping_active ? !waypoint_manager_.empty() : planner_status.path_available;
    safety_input.planner_global_plan_succeeded =
        snapshot.mapping_active || planner_status.global_plan_succeeded;
    safety_input.planner_local_plan_succeeded =
        snapshot.mapping_active || planner_status.local_plan_succeeded;
    safety_input.localization_degraded =
        !snapshot.mapping_active && fsm_context.localization_degraded;
    safety_input.localization_pose_trusted =
        snapshot.mapping_active || localization_result.status.pose_trusted;
    safety_input.localization_match_score = localization_result.status.match_score;
    safety_input.planner_failed = !snapshot.mapping_active && fsm_context.planner_failed;
    safety_input.goal_reached = !snapshot.mapping_active && planner_status.reached;
    safety_input.mission_timeout_enabled = !snapshot.mapping_active;
    safety_input.last_communication_rx_ns =
        loaded_config_.comm.stm32_enabled && stm32_available_.load(std::memory_order_acquire)
            ? last_stm32_rx_ns_.load(std::memory_order_acquire)
            : common::NowNs();
    safety_input.last_chassis_feedback_stamp =
        loaded_config_.comm.stm32_enabled && stm32_available_.load(std::memory_order_acquire)
            ? odom_snapshot.stamp
            : now;
    safety_input.current_pose = &current_pose;
    safety_input.costmap = &costmap;
    safety_input.obstacles = &obstacles.obstacles;
    safety_input.proposed_cmd = &safe_cmd_candidate;

    safety::SafetyResult safety_result;
    auto safety_status = safety_manager_.Evaluate(safety_input, &safety_result);
    if (!safety_status.ok()) {
      safety_result = {};
      safety_result.authority = safety::SafetyCommandAuthority::kFailsafe;
      safety_result.gated_cmd.stamp = now;
      safety_result.gated_cmd.brake = true;
      safety_result.failsafe_cmd = safety_result.gated_cmd;
      safety_result.has_event = true;
      safety_result.event.stamp = now;
      safety_result.event.code = data::SafetyEventCode::kFailsafeOverride;
      safety_result.event.severity = data::SafetySeverity::kCritical;
      safety_result.event.message = "safety manager evaluation failed";
    }

    if (safety_result.has_event && !SafetyEventEquals(last_event, safety_result.event)) {
      latest_safety_event_.Publish(safety_result.event);
      last_event = safety_result.event;
    }
    latest_safety_result_.Publish(safety_result);

    if (safety_result.gated_cmd.stamp != last_cmd.stamp ||
        safety_result.gated_cmd.brake != last_cmd.brake ||
        safety_result.gated_cmd.vx_mps != last_cmd.vx_mps ||
        safety_result.gated_cmd.vy_mps != last_cmd.vy_mps ||
        safety_result.gated_cmd.wz_radps != last_cmd.wz_radps) {
      safety_cmd_.Publish(safety_result.gated_cmd);
      last_cmd = safety_result.gated_cmd;
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
  const auto watchdog_period =
      LoopPeriodFromHz(loaded_config_.debug.watchdog_write_hz, 1000);
  const std::filesystem::path debug_output_dir(RuntimeDebugOutputDir());
  auto next_log = common::Now();
  auto next_scalar_publish = common::Now();
  auto next_pointcloud_publish = common::Now();
  auto next_watchdog_write = common::Now();
  std::uint64_t last_safety_event_generation = latest_safety_event_.generation();
  bool mapping_snapshot_written = false;
  std::uint64_t last_mapping_snapshot_frames = 0;
  std::uint64_t last_mapping_cmd_generation = mapping_cmd_.generation();
  std::uint64_t last_mapping_pose_generation = mapping_pose_.generation();
  int last_mapping_save_trigger = mapping_save_trigger_.load(std::memory_order_acquire);
  bool localization_snapshot_written = false;
  std::uint64_t last_localized_snapshot_frames = 0;
  std::uint64_t last_planner_snapshot_cycles = 0;
  std::uint64_t last_recovery_snapshot_generation = latest_recovery_status_.generation();
  std::uint64_t last_safety_snapshot_generation = latest_safety_result_.generation();
  std::string last_fsm_status_json;
  std::string last_fsm_event_json;
  std::string last_perf_status_json;
  std::string last_runtime_state_json;
  std::string last_watchdog_json;
  while (!stop_requested_.load(std::memory_order_acquire)) {
    const auto now = common::Now();
    if (now >= next_log) {
      std::ostringstream stream;
      const auto fsm_snapshot = fsm_snapshot_.ReadSnapshot();
      const auto localization_result = localization_.LatestResult();
      const auto planner_status = planner_.LatestStatus();
      const auto safety_result = latest_safety_result_.ReadSnapshot();
      const auto recovery_status = latest_recovery_status_.ReadSnapshot();
      const auto match_decision = latest_match_mode_decision_.ReadSnapshot();
      const auto planner_cmd = planner_.LatestCmd();
      const auto save_trigger = static_cast<MappingSaveTrigger>(
          mapping_save_trigger_.load(std::memory_order_acquire));
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
             << ", event=" << fsm::ToString(fsm_snapshot.last_event.code)
             << ", map=" << CurrentMapLabel(fsm_snapshot.mapping_active, localization_.static_map())
             << ")";
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
               << ", save_trigger=" << ToString(save_trigger)
               << ")";
      } else if (fsm_snapshot.localization_active) {
        stream << " loc(matcher=" << localization_result.matcher_name
               << ", score=" << localization_result.status.match_score
               << ", iter=" << localization_result.status.iterations
               << ", fail=" << localization_result.status.consecutive_failures
               << ", trusted=" << (localization_result.status.pose_trusted ? "true" : "false")
               << ", reject=" << localization_result.status.rejection_reason
               << ", relocal=" << ToString(localization_result.relocalization.phase)
               << "/"
               << (localization_result.relocalization.succeeded
                       ? "success"
                       : (localization_result.relocalization.attempted ? "failed" : "idle"))
               << ", map=" << (localization_.map_loaded() ? "loaded" : "missing") << ")"
               << " nav(mode=" << ToString(planner_status.mode)
               << ", dist=" << planner_status.distance_to_goal_m
               << ", reached=" << (planner_status.reached ? "true" : "false")
               << ", global=" << (planner_status.global_plan_succeeded ? "ok" : "fail")
               << ", local=" << (planner_status.local_plan_succeeded ? "ok" : "fail")
               << ", temp_goal=" << (planner_status.temporary_goal_active ? "true" : "false")
               << ", clearance_scale=" << planner_status.clearance_weight_scale
               << ", reason=" << planner_status.failure_reason
               << ", cmd=" << planner_cmd.vx_mps << "/" << planner_cmd.vy_mps << "/"
               << planner_cmd.wz_radps << ")"
               << " safety(state=" << safety::ToString(safety_result.state)
               << ", authority=" << safety::ToString(safety_result.authority)
               << ", gate=" << safety::ToString(safety_result.gate_reason)
               << ", limited=" << (safety_result.gate_limited ? "true" : "false")
               << ", reason=" << SafetyGateReasonSummary(safety_result) << ")"
               << " recovery(tier=" << fsm::ToString(recovery_status.tier)
               << ", cause=" << fsm::ToString(recovery_status.cause)
               << ", action=" << fsm::ToString(recovery_status.action)
               << ", strategy=" << recovery_status.strategy << ")"
               << " match(phase=" << ToString(match_decision.phase)
               << ", reason=" << match_decision.reason
               << ", wait_ms=" << match_decision.wait_remaining_ns / 1000000LL
               << ", rearm=" << (match_decision.rearm_required ? "true" : "false") << ")"
               << " why(no_move="
               << WhyRobotNotMoving(fsm_snapshot, localization_result, planner_status,
                                    safety_result)
               << ", slow="
               << WhyRobotSlowing(fsm_snapshot, planner_status, safety_result) << ")"
               << " degrade="
               << CurrentDegradedSummary(localization_result, planner_status, safety_result);
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
      if (now >= next_watchdog_write) {
        next_watchdog_write = now + watchdog_period;
        const auto watchdog_json = BuildHeartbeatJson(now, fsm_snapshot.state);
        WriteTextFileIfChanged("logs/watchdog/process_heartbeat.json", watchdog_json,
                               &last_watchdog_json);
      }
      const bool allow_scalar_publish = now >= next_scalar_publish;
      if (allow_scalar_publish) {
        next_scalar_publish = now + scalar_period;
        const auto scalar_begin_ns = common::NowNs();
        const auto fsm_status_json = BuildFsmStatusJson(fsm_snapshot);
        const auto fsm_event_json = BuildFsmEventJson(fsm_snapshot.last_event);
        WriteFsmDebugSnapshot(fsm_status_json, fsm_event_json, debug_output_dir,
                              &last_fsm_status_json, &last_fsm_event_json);
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
        WriteTextFileIfChanged(debug_output_dir / "perf_status.json", perf_json,
                               &last_perf_status_json);
        WriteTextFileIfChanged("logs/crash/last_runtime_state.json", perf_json,
                               &last_runtime_state_json);
        if (loaded_config_.debug.websocket_enabled) {
          PublishFoxgloveJson(&foxglove_server_, kRuntimeFsmStatusChannelId, fsm_status_json,
                              now);
          PublishFoxgloveJson(&foxglove_server_, kRuntimeFsmEventChannelId, fsm_event_json,
                              now);
          PublishFoxgloveJson(&foxglove_server_, kRuntimePoseChannelId,
                              BuildPoseJson(current_pose), now);
          PublishFoxgloveJson(&foxglove_server_, kRuntimeCmdChannelId,
                              BuildCmdJson(current_cmd), now);
          PublishFoxgloveJson(&foxglove_server_, kRuntimePerfStatusChannelId, perf_json, now);
        }
        debug_encode_latency_ns_.store(common::NowNs() - scalar_begin_ns,
                                       std::memory_order_release);
      }
      if (loaded_config_.debug.websocket_enabled) {
        const auto debug_begin_ns = common::NowNs();
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

        if (allow_pointcloud_publish && !suppress_scene && fsm_snapshot.mapping_active) {
          next_pointcloud_publish = now + pointcloud_period;
          const auto publish_begin_ns = common::NowNs();
          auto current_pose = mapping_pose_.ReadSnapshot();
          if (!current_pose.is_valid) {
            current_pose = MakeMapPose(now, static_cast<float>(loaded_config_.spawn.x_m),
                                       static_cast<float>(loaded_config_.spawn.y_m),
                                       static_cast<float>(loaded_config_.spawn.theta_rad));
          }
          const auto mapped_frames = mapped_frames_.load(std::memory_order_acquire);
          const auto mapping_cmd_generation = mapping_cmd_.generation();
          const auto mapping_pose_generation = mapping_pose_.generation();
          const int mapping_save_trigger = static_cast<int>(save_trigger);
          if (!loaded_config_.debug.write_snapshots_only_on_change ||
              !mapping_snapshot_written ||
              mapped_frames != last_mapping_snapshot_frames ||
              mapping_cmd_generation != last_mapping_cmd_generation ||
              mapping_pose_generation != last_mapping_pose_generation ||
              mapping_save_trigger != last_mapping_save_trigger) {
            WriteMappingDebugSnapshot(mapping_engine_, waypoint_manager_,
                                      mapping_pose_.ReadSnapshot(), mapping_cmd_.ReadSnapshot(),
                                      save_trigger, debug_output_dir);
            mapping_snapshot_written = true;
            last_mapping_snapshot_frames = mapped_frames;
            last_mapping_cmd_generation = mapping_cmd_generation;
            last_mapping_pose_generation = mapping_pose_generation;
            last_mapping_save_trigger = mapping_save_trigger;
          }
          const auto mapping_status_json =
              BuildRuntimeMappingStatusJson(mapping_engine_, waypoint_manager_, save_trigger);
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
          const auto localized_frames = localized_frames_.load(std::memory_order_acquire);
          const auto planned_cycles = planned_cycles_.load(std::memory_order_acquire);
          const auto recovery_generation = latest_recovery_status_.generation();
          const auto safety_generation = latest_safety_result_.generation();
          if (!loaded_config_.debug.write_snapshots_only_on_change ||
              !localization_snapshot_written ||
              localized_frames != last_localized_snapshot_frames ||
              planned_cycles != last_planner_snapshot_cycles ||
              recovery_generation != last_recovery_snapshot_generation ||
              safety_generation != last_safety_snapshot_generation) {
            WriteDebugSnapshot(localization_, planner_, latest_recovery_status_.ReadSnapshot(),
                               latest_safety_result_.ReadSnapshot(), debug_output_dir);
            localization_snapshot_written = true;
            last_localized_snapshot_frames = localized_frames;
            last_planner_snapshot_cycles = planned_cycles;
            last_recovery_snapshot_generation = recovery_generation;
            last_safety_snapshot_generation = safety_generation;
          }
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
  std::string source_label;
  if (!ResolveCombatLocalizationConfig(loaded_config_, &localization_config, &source_label)) {
    combat_map_unavailable_.store(true, std::memory_order_release);
    return common::Status::Unavailable("no usable combat map available");
  }
  combat_map_unavailable_.store(false, std::memory_order_release);

  config::SpawnConfig spawn_config = loaded_config_.spawn;
  const auto latest_mapping_pose = mapping_pose_.ReadSnapshot();
  if (latest_mapping_pose.is_valid) {
    spawn_config.x_m = latest_mapping_pose.position.x;
    spawn_config.y_m = latest_mapping_pose.position.y;
    spawn_config.theta_rad = latest_mapping_pose.rpy.z;
  }
  return InitializeCombatPipeline(localization_config, spawn_config);
}

bool Runtime::RefereeStartRequired(bool for_mapping) const {
  (void)for_mapping;
  if (loaded_config_.system.match_mode_enabled) {
    return true;
  }
  if (loaded_config_.system.bringup_mode != "none" || !loaded_config_.comm.stm32_enabled ||
      !stm32_available_.load(std::memory_order_acquire)) {
    return false;
  }
  return false;
}

bool Runtime::RefereeStartAuthorized(bool for_mapping) const {
  if (!RefereeStartRequired(for_mapping)) {
    return true;
  }
  return RefereeStartSignalActive(referee_state_.ReadSnapshot());
}

bool Runtime::MappingSaveEligible() const {
  if (!mapping_mode_) {
    return false;
  }
  if (mapped_frames_.load(std::memory_order_relaxed) > 0U) {
    return true;
  }
  const auto snapshot = fsm_snapshot_.ReadSnapshot();
  return snapshot.mapping_active ||
         mapping_save_requested_.load(std::memory_order_acquire) ||
         mapping_loop_save_requested_.load(std::memory_order_acquire);
}

fsm::NavFsmContext Runtime::BuildFsmContext(bool referee_changed) const {
  fsm::NavFsmContext context;
  const bool map_saved = map_saved_.load(std::memory_order_acquire);
  const bool warmup_start_authorized = RefereeStartAuthorized(true);
  const bool combat_start_authorized = RefereeStartAuthorized(false);
  const bool combat_ready =
      combat_pipeline_ready_.load(std::memory_order_acquire) && combat_start_authorized;
  context.save_requested =
      mapping_mode_ && mapping_save_requested_.load(std::memory_order_acquire) && !map_saved;
  context.map_saved = map_saved;
  context.combat_ready = combat_ready;
  context.map_loaded = context.combat_ready && localization_.map_loaded();
  context.referee_changed = referee_changed;
  context.map_unavailable = combat_map_unavailable_.load(std::memory_order_acquire);
  context.combat_requested = combat_mode_requested_ && combat_start_authorized;
  const auto save_failure_tag = static_cast<MappingSaveFailureTag>(
      mapping_save_failure_tag_.load(std::memory_order_acquire));
  context.map_save_write_failed = save_failure_tag == MappingSaveFailureTag::kWriteFailed;
  context.map_save_validation_failed =
      save_failure_tag == MappingSaveFailureTag::kValidationFailed;
  context.map_save_storage_failed =
      save_failure_tag == MappingSaveFailureTag::kStorageSwitchFailed;
  context.mapping_enabled = mapping_mode_ && !map_saved && warmup_start_authorized;

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
    const auto match_decision = latest_match_mode_decision_.ReadSnapshot();
    const auto safety_result = latest_safety_result_.ReadSnapshot();
    const auto recovery_status = latest_recovery_status_.ReadSnapshot();
    const auto latest_cmd = planner_.LatestCmd();
    context.map_label = CurrentMapLabel(false, localization_.static_map());
    context.localization_matcher = localization_.CurrentMatcherLabel();
    context.localization_reason =
        localization_result.status.pose_trusted ? "none" : localization_result.status.rejection_reason;
    context.planner_reason = planner_status.failure_reason;
    context.safety_reason = SafetyGateReasonSummary(safety_result);
    context.degraded_mode =
        CurrentDegradedSummary(localization_result, planner_status, safety_result);
    context.recovery_tier = recovery_status.tier;
    context.recovery_cause = recovery_status.cause;
    context.recovery_action = recovery_status.action;
    context.recovery_complete = recovery_status.complete;
    context.recovery_exhausted = recovery_status.exhausted;
    context.recovery_needs_relocalization = recovery_status.requires_relocalization;
    context.recovery_strategy = recovery_status.strategy;
    if (loaded_config_.system.match_mode_enabled) {
      context.recovery_strategy +=
          std::string("|match=") + ToString(match_decision.phase) + ":" +
          match_decision.reason;
    }
    context.localization_degraded =
        localization_result.map_to_base.is_valid &&
        (!localization_result.status.pose_trusted ||
         localization_result.status.consecutive_failures >=
             static_cast<std::uint32_t>(
                 std::max(1, loaded_config_.localization.relocalization_failure_threshold)));
    context.planner_failed = planned_cycles_.load(std::memory_order_relaxed) > 0U &&
                             (planner_status.failure_reason != "none" ||
                              (!planner_status.reached && !planner_status.path_available));
    if (latest_cmd.stamp != common::TimePoint{}) {
      const auto cmd_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  common::Now() - latest_cmd.stamp)
                                  .count();
      if (cmd_age_ms > loaded_config_.safety.deadman_timeout_ms) {
        context.planner_failed = true;
      }
    }
    context.center_reached = planner_status.reached && match_decision.count_as_center_goal;
    context.center_drifted = planner_status.hold_drifted;

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
  } else {
    context.map_label = CurrentMapLabel(mapping_mode_, localization_.static_map());
    context.localization_matcher = localization_.CurrentMatcherLabel();
  }

  return context;
}

data::ChassisCmd Runtime::SelectCommandCandidate(const fsm::NavFsmSnapshot& snapshot,
                                                 common::TimePoint stamp) {
  data::ChassisCmd candidate_cmd;
  candidate_cmd.stamp = stamp;
  candidate_cmd.brake = true;

  switch (snapshot.state) {
    case fsm::NavState::kModeWarmup: {
      candidate_cmd = mapping_cmd_.ReadSnapshot();
      if (!mapping_pose_.ReadSnapshot().is_valid) {
        candidate_cmd = {};
        candidate_cmd.stamp = stamp;
        candidate_cmd.brake = true;
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
      if (snapshot.recovery_active) {
        const auto planner_status = planner_.LatestStatus();
        const auto planner_path = planner_.LatestPath();
        const auto planner_cmd = planner_.LatestCmd();
        const auto costmap = local_costmap_builder_.LatestCostmap();
        const auto obstacles = mot_manager_.LatestObstacles();
        const auto current_pose = localization_result.map_to_base;
        planning::RecoveryPlannerStatus recovery_status;
        if (localization_result.relocalization.active &&
            localization_result.relocalization.recovery_action !=
                localization::RelocalizationRecoveryAction::kNone) {
          recovery_status.active = true;
          recovery_status.tier = fsm::RecoveryTier::kHeavy;
          recovery_status.cause = fsm::RecoveryCause::kLocalizationLost;
          recovery_status.action = fsm::RecoveryAction::kStopAndRelocalize;
          recovery_status.requires_relocalization = true;
          recovery_status.strategy = "l3_localization_override";
          recovery_status.detail = ToString(localization_result.relocalization.recovery_action);
          recovery_status.command =
              ClampCommandForMode(localization_result.relocalization.recovery_cmd, snapshot,
                                  loaded_config_, stamp);
          latest_recovery_status_.Publish(recovery_status);
          candidate_cmd = recovery_status.command;
        } else {
          planning::RecoveryPlannerInput recovery_input;
          recovery_input.stamp = stamp;
          recovery_input.current_pose = &current_pose;
          recovery_input.global_path = &planner_path;
          recovery_input.costmap = &costmap;
          recovery_input.obstacles = &obstacles.obstacles;
          recovery_input.localization_result = &localization_result;
          recovery_input.planner_status = &planner_status;
          recovery_input.nominal_cmd = &planner_cmd;
          if (recovery_planner_.Plan(recovery_input, &candidate_cmd, &recovery_status).ok()) {
            recovery_status.command =
                ClampCommandForMode(candidate_cmd, snapshot, loaded_config_, stamp);
            latest_recovery_status_.Publish(recovery_status);
            candidate_cmd = recovery_status.command;
          } else {
            candidate_cmd = {};
            candidate_cmd.stamp = stamp;
            candidate_cmd.brake = true;
          }
        }
      } else if (localization_result.relocalization.active &&
                 localization_result.relocalization.recovery_action !=
                     localization::RelocalizationRecoveryAction::kNone) {
        candidate_cmd =
            ClampCommandForMode(localization_result.relocalization.recovery_cmd, snapshot,
                                loaded_config_, stamp);
      } else {
        const auto planner_cmd = planner_.LatestCmd();
        candidate_cmd = ClampCommandForMode(planner_cmd, snapshot, loaded_config_, stamp);
      }
      break;
    }
    case fsm::NavState::kBoot:
    case fsm::NavState::kSelfCheck:
    case fsm::NavState::kIdle:
    case fsm::NavState::kModeSave:
    case fsm::NavState::kFailsafe:
      break;
  }

  if (!snapshot.recovery_active) {
    recovery_planner_.Reset();
    latest_recovery_status_.Publish(planning::RecoveryPlannerStatus{});
  }

  return candidate_cmd;
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
  mapping::MapSaveFailureKind failure_kind{mapping::MapSaveFailureKind::kNone};
  const auto status = mapping_engine_.SaveMap(
      ResolveConfigAsset(loaded_config_.config_dir, loaded_config_.mapping.active_dir).string(),
      exported_map != nullptr ? exported_map : &local_exported_map, &failure_kind);
  if (!status.ok()) {
    mapping_save_failure_tag_.store(static_cast<int>(ToRuntimeFailureTag(failure_kind)),
                                    std::memory_order_release);
    if (failure_kind == mapping::MapSaveFailureKind::kValidationFailed) {
      config::LocalizationConfig fallback_config = loaded_config_.localization;
      std::string source_label;
      if (!ResolveCombatLocalizationConfig(loaded_config_, &fallback_config, &source_label) ||
          source_label == "missing") {
        combat_map_unavailable_.store(true, std::memory_order_release);
      }
    }
    utils::LogWarn("mapping", std::string("failed to save mapping artifacts: ") + status.message);
    return status;
  }
  map_saved_.store(true, std::memory_order_release);
  mapping_save_requested_.store(false, std::memory_order_release);
  mapping_loop_save_requested_.store(false, std::memory_order_release);
  mapping_save_failure_tag_.store(static_cast<int>(MappingSaveFailureTag::kNone),
                                  std::memory_order_release);
  mapping_save_trigger_.store(static_cast<int>(MappingSaveTrigger::kNone),
                              std::memory_order_release);
  combat_map_unavailable_.store(false, std::memory_order_release);
  utils::LogInfo("mapping",
                 std::string("saved warmup map to ") +
                     (exported_map != nullptr ? exported_map->global_map_path
                                              : local_exported_map.global_map_path));
  return common::Status::Ok();
}

bool Runtime::PerceptionBringupMode() const {
  return loaded_config_.system.bringup_mode == "lidar_view";
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
