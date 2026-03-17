#pragma once

#include <cstdint>
#include <optional>

#include "rm_nav/common/types.hpp"
#include "rm_nav/common/object_pool.hpp"
#include "rm_nav/common/ring_queue.hpp"
#include "rm_nav/common/status.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/sync/sensor_sync_buffer.hpp"

namespace rm_nav::perception {

struct PreprocessConfig {
  float min_range_m{0.2F};
  float max_range_m{8.0F};
  float min_height_m{-0.5F};
  float max_height_m{1.5F};
  float ground_z_max_m{-0.05F};
  float voxel_size_m{0.15F};
  bool self_mask_enabled{false};
  float self_mask_x_min_m{-0.3F};
  float self_mask_x_max_m{0.3F};
  float self_mask_y_min_m{-0.25F};
  float self_mask_y_max_m{0.25F};
  std::array<common::Vec2f, 4> self_mask_polygon{};
  bool self_mask_polygon_valid{false};
  std::size_t max_filtered_frames{8};
};

using FilteredFramePool = common::ObjectPool<data::LidarFrame, 8>;
using FilteredFrameHandle = FilteredFramePool::Handle;

class PreprocessPipeline {
 public:
  common::Status Configure(const PreprocessConfig& config);
  common::Status EnqueueFrame(sync::SyncedFrameHandle frame);
  common::Status ProcessOnce();
  std::optional<FilteredFrameHandle> TryPopFilteredFrameHandle();
  common::Status Run(const data::SyncedFrame& input,
                     data::LidarFrame* filtered);

  std::uint64_t dropped_frames() const { return dropped_frames_; }

 private:
  PreprocessConfig config_{};
  common::SpscRingQueue<sync::SyncedFrameHandle, 16> input_queue_{};
  common::SpscRingQueue<FilteredFrameHandle, 8> output_queue_{};
  FilteredFramePool frame_pool_{};
  std::uint64_t dropped_frames_{0};
  bool configured_{false};
};

}  // namespace rm_nav::perception
