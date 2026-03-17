#include "rm_nav/sync/sensor_sync_buffer.hpp"

#include <utility>

namespace rm_nav::sync {

common::Status SensorSync::Configure(const SyncConfig& config) {
  if (config.max_imu_packets_per_frame <= 0 ||
      config.max_imu_packets_per_frame >
          static_cast<int>(data::SyncedFrame::kMaxImuPackets)) {
    return common::Status::InvalidArgument("invalid sync imu packet limit");
  }
  config_ = config;
  imu_backlog_size_ = 0;
  dropped_imu_packets_ = 0;
  dropped_lidar_frames_ = 0;
  dropped_synced_frames_ = 0;
  configured_ = true;
  return common::Status::Ok();
}

common::Status SensorSync::PushImuPacket(const data::ImuPacket& packet) {
  if (!configured_) {
    return common::Status::NotReady("sensor sync is not configured");
  }
  if (!imu_queue_.try_push(packet)) {
    ++dropped_imu_packets_;
  }
  return common::Status::Ok();
}

common::Status SensorSync::PushLidarFrame(const data::LidarFrame& frame) {
  if (!configured_) {
    return common::Status::NotReady("sensor sync is not configured");
  }
  if (!lidar_queue_.try_push(frame)) {
    ++dropped_lidar_frames_;
  }
  return common::Status::Ok();
}

common::Status SensorSync::ProcessOnce() {
  if (!configured_) {
    return common::Status::NotReady("sensor sync is not configured");
  }

  const auto begin_ns = common::NowNs();
  DrainImuQueue();

  data::LidarFrame lidar_frame;
  if (!lidar_queue_.try_pop(&lidar_frame)) {
    return common::Status::NotReady("no lidar frame available");
  }

  auto handle = synced_frame_pool_.Acquire();
  if (!handle) {
    ++dropped_synced_frames_;
    return common::Status::Unavailable("synced frame pool is exhausted");
  }

  const auto status = BuildSyncedFrame(lidar_frame, handle.get());
  if (!status.ok()) {
    return status;
  }

  SyncPerfSnapshot perf;
  perf.process_latency_ns = common::NowNs() - begin_ns;
  perf.preintegration_latency_ns = handle.get()->preint.integration_latency_ns;
  perf.deskew_latency_ns = handle.get()->deskew_latency_ns;
  perf.imu_packet_count = handle.get()->imu_packet_count;
  latest_perf_.Publish(perf);

  if (!output_queue_.try_push(std::move(handle))) {
    ++dropped_synced_frames_;
    return common::Status::Unavailable("sync output queue is full");
  }
  return common::Status::Ok();
}

std::optional<SyncedFrameHandle> SensorSync::TryPopSyncedFrameHandle() {
  SyncedFrameHandle handle;
  if (!output_queue_.try_pop(&handle)) {
    return std::nullopt;
  }
  return std::optional<SyncedFrameHandle>(std::move(handle));
}

std::optional<data::SyncedFrame> SensorSync::TryPopSyncedFrame() {
  auto handle = TryPopSyncedFrameHandle();
  if (!handle.has_value()) {
    return std::nullopt;
  }
  data::SyncedFrame frame = *handle->get();
  handle->reset();
  return frame;
}

void SensorSync::DrainImuQueue() {
  data::ImuPacket packet;
  while (imu_queue_.try_pop(&packet)) {
    if (imu_backlog_size_ >= imu_backlog_.size()) {
      for (std::size_t index = 1; index < imu_backlog_size_; ++index) {
        imu_backlog_[index - 1U] = imu_backlog_[index];
      }
      --imu_backlog_size_;
      ++dropped_imu_packets_;
    }
    imu_backlog_[imu_backlog_size_++] = packet;
  }
}

common::Status SensorSync::BuildSyncedFrame(const data::LidarFrame& lidar_frame,
                                            data::SyncedFrame* synced_frame) {
  if (synced_frame == nullptr) {
    return common::Status::InvalidArgument("synced frame output is null");
  }

  synced_frame->stamp = lidar_frame.scan_end_stamp;
  synced_frame->lidar = lidar_frame;
  synced_frame->ClearImuPackets();
  synced_frame->preint = {};

  const common::TimePoint slice_begin = lidar_frame.scan_begin_stamp -
                                        std::chrono::nanoseconds(config_.imu_slice_margin_ns);
  const common::TimePoint slice_end = lidar_frame.scan_end_stamp +
                                      std::chrono::nanoseconds(config_.imu_slice_margin_ns);

  std::size_t kept_count = 0;
  std::size_t write_index = 0;
  for (std::size_t read_index = 0; read_index < imu_backlog_size_; ++read_index) {
    const data::ImuPacket& packet = imu_backlog_[read_index];
    const bool in_slice = packet.stamp >= slice_begin && packet.stamp <= slice_end &&
                          kept_count < static_cast<std::size_t>(config_.max_imu_packets_per_frame);
    if (in_slice) {
      synced_frame->imu_packets[kept_count++] = packet;
    }
    if (packet.stamp > slice_begin) {
      imu_backlog_[write_index++] = packet;
    }
  }
  imu_backlog_size_ = write_index;
  synced_frame->imu_packet_count = kept_count;

  const auto preintegration_begin_ns = common::NowNs();
  auto status = preintegrator_.Integrate(synced_frame->imu_packets.data(), kept_count,
                                         lidar_frame.scan_begin_stamp,
                                         lidar_frame.scan_end_stamp,
                                         &synced_frame->preint);
  synced_frame->preint.integration_latency_ns = common::NowNs() - preintegration_begin_ns;
  if (!status.ok() && status.code != common::StatusCode::kNotReady) {
    return status;
  }

  const auto deskew_begin_ns = common::NowNs();
  data::LidarFrame deskewed;
  status = deskew_.Apply(lidar_frame, synced_frame->preint, &deskewed);
  if (!status.ok()) {
    return status;
  }
  synced_frame->deskew_latency_ns = common::NowNs() - deskew_begin_ns;
  synced_frame->lidar = std::move(deskewed);
  synced_frame->stamp = synced_frame->lidar.stamp;
  synced_frame->sync_latency_ns = common::NowNs() - common::ToNanoseconds(synced_frame->stamp);
  return common::Status::Ok();
}

}  // namespace rm_nav::sync
