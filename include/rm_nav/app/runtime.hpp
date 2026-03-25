#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rm_nav/app/bootstrap.hpp"
#include "rm_nav/app/match_mode_controller.hpp"
#include "rm_nav/app/odom_feedback_yaw_adapter.hpp"
#include "rm_nav/common/double_buffer.hpp"
#include "rm_nav/common/status.hpp"
#include "rm_nav/config/config_loader.hpp"
#include "rm_nav/debug/foxglove_server.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/odom_state.hpp"
#include "rm_nav/data/referee_state.hpp"
#include "rm_nav/data/safety_event.hpp"
#include "rm_nav/drivers/imu/imu_driver.hpp"
#include "rm_nav/drivers/lidar/l1_driver.hpp"
#include "rm_nav/drivers/stm32/stm32_bridge.hpp"
#include "rm_nav/fsm/nav_fsm.hpp"
#include "rm_nav/localization/localization_engine.hpp"
#include "rm_nav/mapping/mapping_engine.hpp"
#include "rm_nav/mapping/waypoint_manager.hpp"
#include "rm_nav/perception/local_costmap_builder.hpp"
#include "rm_nav/perception/mot_manager.hpp"
#include "rm_nav/perception/preprocess_pipeline.hpp"
#include "rm_nav/planning/planner_coordinator.hpp"
#include "rm_nav/planning/recovery_planner.hpp"
#include "rm_nav/safety/safety_manager.hpp"
#include "rm_nav/sync/sensor_sync_buffer.hpp"
#include "rm_nav/tf/tf_tree_lite.hpp"

namespace rm_nav::app {

class Runtime {
 public:
  enum class MappingSaveFailureTag {
    kNone = 0,
    kWriteFailed,
    kValidationFailed,
    kStorageSwitchFailed,
  };

  enum class MappingSaveTrigger {
    kNone = 0,
    kLoopClosure,
    kAutoShutdown,
    kExternalStop,
  };

  Runtime();
  ~Runtime();

  const RuntimeManifest& manifest() const { return manifest_; }
  common::Status Initialize(const std::string& config_dir);
  int Run(const std::atomic_bool& external_stop);
  void RequestStop();
  common::Status ValidatePhase0Contract() const;

 private:
  struct WorkerThread {
    ThreadBinding binding{};
    int cpu_id{-1};
    std::thread thread{};
  };

  common::Status ValidateLoadedConfig() const;
  common::Status ConfigurePipeline();
  common::Status StartThreads();
  void StopThreads();
  void ThreadMain(ThreadBinding binding, int cpu_id);
  void DriverThreadMain();
  void SyncThreadMain();
  void PoseThreadMain();
  void PerceptionThreadMain();
  void PlannerThreadMain();
  void SafetyThreadMain();
  void DebugThreadMain();
  common::Status SaveMappingArtifacts(
      localization::StaticMap* exported_map = nullptr);
  bool PerceptionBringupMode() const;
  common::Status InitializeCombatPipeline(
      const config::LocalizationConfig& localization_config,
      const config::SpawnConfig& spawn_config);
  common::Status InitializeCombatPipelineFromSavedMap();
  bool RefereeStartRequired(bool for_mapping) const;
  bool RefereeStartAuthorized(bool for_mapping) const;
  bool MappingSaveEligible() const;
  fsm::NavFsmContext BuildFsmContext(bool referee_changed) const;
  data::ChassisCmd SelectCommandCandidate(const fsm::NavFsmSnapshot& snapshot,
                                          common::TimePoint stamp);
  std::string RuntimeDebugOutputDir() const;
  bool CloneSyncedFrame(const data::SyncedFrame& source,
                        sync::SyncedFrameHandle* handle);
  void LogOncePerThread(const char* component, const std::string& message,
                        bool* already_logged);
  int ResolveCpu(ThreadDomain domain) const;
  void LogStartupSummary() const;
  data::Pose3f MappingPoseFromOdom(const data::OdomState& odom) const;
  data::ChassisCmd CommandForStm32OutputFrame(const data::ChassisCmd& cmd) const;
  float Stm32FeedbackYawSeed() const;

  config::LoadedConfig loaded_config_{};
  RuntimeManifest manifest_{};
  std::vector<WorkerThread> workers_{};
  std::atomic_bool stop_requested_{false};
  std::atomic_bool stm32_available_{false};
  std::atomic_bool pipeline_configured_{false};

  drivers::lidar::L1Driver lidar_driver_{};
  drivers::imu::ImuDriver imu_driver_{};
  drivers::stm32::Stm32Bridge stm32_bridge_{};
  sync::SensorSync sensor_sync_{};
  sync::SyncedFramePool fanout_pool_{};
  tf::TfTreeLite tf_tree_{};
  localization::LocalizationEngine localization_{};
  mapping::MappingEngine mapping_engine_{};
  mapping::WaypointManager waypoint_manager_{};
  perception::PreprocessPipeline preprocess_{};
  perception::LocalCostmapBuilder local_costmap_builder_{};
  perception::MotManager mot_manager_{};
  planning::PlannerCoordinator planner_{};
  planning::RecoveryPlanner recovery_planner_{};
  safety::SafetyManager safety_manager_{};
  debug::FoxgloveServer foxglove_server_{};
  MatchModeController match_mode_controller_{};
  OdomFeedbackYawAdapter stm32_odom_yaw_adapter_{};
  common::SpscRingQueue<sync::SyncedFrameHandle, 16> mapping_queue_{};

  fsm::NavFsm nav_fsm_{};
  common::DoubleBuffer<fsm::NavFsmSnapshot> fsm_snapshot_{};
  common::DoubleBuffer<data::ChassisCmd> safety_cmd_{};
  common::DoubleBuffer<data::ChassisCmd> mapping_cmd_{};
  common::DoubleBuffer<data::Pose3f> mapping_pose_{};
  common::DoubleBuffer<data::LidarFrame> latest_filtered_scan_{};
  common::DoubleBuffer<data::OdomState> stm32_odom_{};
  common::DoubleBuffer<data::RefereeState> referee_state_{};
  common::DoubleBuffer<MatchModeDecision> latest_match_mode_decision_{};
  common::DoubleBuffer<data::SafetyEvent> latest_safety_event_{};
  common::DoubleBuffer<safety::SafetyResult> latest_safety_result_{};
  common::DoubleBuffer<planning::RecoveryPlannerStatus> latest_recovery_status_{};
  mutable std::mutex mapping_queue_wait_mutex_{};
  std::condition_variable mapping_queue_wait_cv_{};
  std::atomic<std::uint32_t> pending_mapping_frames_{0};

  std::atomic<std::uint64_t> driver_lidar_frames_{0};
  std::atomic<std::uint64_t> driver_imu_packets_{0};
  std::atomic<std::uint64_t> synced_frames_{0};
  std::atomic<std::uint64_t> localized_frames_{0};
  std::atomic<std::uint64_t> mapped_frames_{0};
  std::atomic<std::uint64_t> perceived_frames_{0};
  std::atomic<std::uint64_t> planned_cycles_{0};
  std::atomic<std::uint64_t> safety_cycles_{0};
  std::atomic<common::TimeNs> driver_cpu_usage_milli_{0};
  std::atomic<common::TimeNs> sync_process_latency_ns_{0};
  std::atomic<common::TimeNs> sync_preintegration_latency_ns_{0};
  std::atomic<common::TimeNs> sync_deskew_latency_ns_{0};
  std::atomic<common::TimeNs> localization_latency_ns_{0};
  std::atomic<common::TimeNs> localization_matcher_latency_ns_{0};
  std::atomic_bool localization_light_mode_{false};
  std::atomic<common::TimeNs> mot_clustering_latency_ns_{0};
  std::atomic<common::TimeNs> mot_total_latency_ns_{0};
  std::atomic_bool mot_reduced_roi_active_{false};
  std::atomic<common::TimeNs> planner_latency_ns_{0};
  std::atomic<common::TimeNs> safety_tick_latency_ns_{0};
  std::atomic<common::TimeNs> foxglove_publish_latency_ns_{0};
  std::atomic<common::TimeNs> debug_encode_latency_ns_{0};
  std::atomic_bool debug_scene_suppressed_{false};
  std::atomic<common::TimeNs> last_stm32_rx_ns_{0};
  std::atomic_bool mapping_save_requested_{false};
  std::atomic_bool mapping_loop_save_requested_{false};
  std::atomic_bool map_saved_{false};
  std::atomic<int> mapping_save_failure_tag_{static_cast<int>(MappingSaveFailureTag::kNone)};
  std::atomic<int> mapping_save_trigger_{static_cast<int>(MappingSaveTrigger::kNone)};
  std::atomic_bool combat_pipeline_ready_{false};
  std::atomic_bool combat_map_unavailable_{false};
  bool mapping_mode_{false};
  bool combat_mode_requested_{false};
  bool initialized_{false};
};

}  // namespace rm_nav::app
