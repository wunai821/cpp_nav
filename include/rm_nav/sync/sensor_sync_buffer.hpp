#pragma once

#include <array>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>

#include "rm_nav/common/double_buffer.hpp"
#include "rm_nav/common/object_pool.hpp"
#include "rm_nav/common/ring_queue.hpp"
#include "rm_nav/common/status.hpp"
#include "rm_nav/data/imu_packet.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/sync/deskew.hpp"
#include "rm_nav/sync/imu_preintegrator.hpp"

namespace rm_nav::sync {

struct SyncConfig {
  int max_imu_packets_per_frame{128};
  common::TimeNs imu_slice_margin_ns{1000000};
};

using SyncedFramePool = common::ObjectPool<data::SyncedFrame, 8>;
using SyncedFrameHandle = SyncedFramePool::Handle;

struct SyncPerfSnapshot {
  common::TimeNs process_latency_ns{0};
  common::TimeNs preintegration_latency_ns{0};
  common::TimeNs deskew_latency_ns{0};
  std::size_t imu_packet_count{0};
};

class SensorSync {
 public:
  common::Status Configure(const SyncConfig& config);
  common::Status PushImuPacket(const data::ImuPacket& packet);
  common::Status PushLidarFrame(const data::LidarFrame& frame);
  common::Status ProcessOnce();
  bool WaitForLidarFrame(std::chrono::milliseconds timeout) const;
  std::optional<SyncedFrameHandle> TryPopSyncedFrameHandle();
  std::optional<data::SyncedFrame> TryPopSyncedFrame();
  SyncPerfSnapshot LatestPerf() const { return latest_perf_.ReadSnapshot(); }

  std::uint64_t dropped_imu_packets() const { return dropped_imu_packets_; }
  std::uint64_t dropped_lidar_frames() const { return dropped_lidar_frames_; }
  std::uint64_t dropped_synced_frames() const { return dropped_synced_frames_; }

 private:
  static constexpr std::size_t kImuQueueCapacity = 512;
  static constexpr std::size_t kLidarQueueCapacity = 16;

  void DrainImuQueue();
  common::Status BuildSyncedFrame(const data::LidarFrame& lidar_frame,
                                  data::SyncedFrame* synced_frame);

  SyncConfig config_{};
  ImuPreintegrator preintegrator_{};
  Deskew deskew_{};
  common::SpscRingQueue<data::ImuPacket, kImuQueueCapacity> imu_queue_{};
  common::SpscRingQueue<data::LidarFrame, kLidarQueueCapacity> lidar_queue_{};
  common::SpscRingQueue<SyncedFrameHandle, 8> output_queue_{};
  SyncedFramePool synced_frame_pool_{};
  common::DoubleBuffer<SyncPerfSnapshot> latest_perf_{};
  mutable std::mutex lidar_wait_mutex_{};
  mutable std::condition_variable lidar_wait_cv_{};
  std::atomic<std::uint32_t> pending_lidar_frames_{0};
  std::array<data::ImuPacket, data::SyncedFrame::kMaxImuPackets * 2> imu_backlog_{};
  std::size_t imu_backlog_head_{0};
  std::size_t imu_backlog_size_{0};
  std::uint64_t dropped_imu_packets_{0};
  std::uint64_t dropped_lidar_frames_{0};
  std::uint64_t dropped_synced_frames_{0};
  bool configured_{false};
};

}  // namespace rm_nav::sync
