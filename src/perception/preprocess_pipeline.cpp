#include "rm_nav/perception/preprocess_pipeline.hpp"

#include <cmath>
#include <cstdint>
#include <unordered_set>

namespace rm_nav::perception {
namespace {

std::uint64_t VoxelKey(int vx, int vy, int vz) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(vx)) << 42U) ^
         (static_cast<std::uint64_t>(static_cast<std::uint32_t>(vy)) << 21U) ^
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(vz));
}

bool PassesFilters(const PreprocessConfig& config, const data::PointXYZI& point) {
  const float range_m = std::sqrt(point.x * point.x + point.y * point.y);
  if (range_m < config.min_range_m || range_m > config.max_range_m) {
    return false;
  }
  if (point.z < config.min_height_m || point.z > config.max_height_m) {
    return false;
  }
  if (point.z <= config.ground_z_max_m) {
    return false;
  }
  return !(point.x == 0.0F && point.y == 0.0F && point.z == 0.0F);
}

}  // namespace

common::Status PreprocessPipeline::Configure(const PreprocessConfig& config) {
  if (config.min_range_m < 0.0F || config.max_range_m <= config.min_range_m ||
      config.max_height_m <= config.min_height_m || config.voxel_size_m <= 0.0F) {
    return common::Status::InvalidArgument("invalid preprocess config");
  }
  config_ = config;
  dropped_frames_ = 0;
  configured_ = true;
  return common::Status::Ok();
}

common::Status PreprocessPipeline::EnqueueFrame(sync::SyncedFrameHandle frame) {
  if (!configured_) {
    return common::Status::NotReady("preprocess pipeline is not configured");
  }
  if (!input_queue_.try_push(std::move(frame))) {
    ++dropped_frames_;
    return common::Status::Unavailable("preprocess input queue is full");
  }
  return common::Status::Ok();
}

common::Status PreprocessPipeline::ProcessOnce() {
  sync::SyncedFrameHandle input;
  if (!input_queue_.try_pop(&input)) {
    return common::Status::NotReady("no synced frame for preprocess");
  }

  auto filtered = frame_pool_.Acquire();
  if (!filtered) {
    ++dropped_frames_;
    input.reset();
    return common::Status::Unavailable("filtered frame pool is exhausted");
  }

  const auto status = Run(*input.get(), filtered.get());
  input.reset();
  if (!status.ok()) {
    filtered.reset();
    return status;
  }
  if (!output_queue_.try_push(std::move(filtered))) {
    ++dropped_frames_;
    return common::Status::Unavailable("preprocess output queue is full");
  }
  return common::Status::Ok();
}

std::optional<FilteredFrameHandle> PreprocessPipeline::TryPopFilteredFrameHandle() {
  FilteredFrameHandle handle;
  if (!output_queue_.try_pop(&handle)) {
    return std::nullopt;
  }
  return std::optional<FilteredFrameHandle>(std::move(handle));
}

common::Status PreprocessPipeline::Run(const data::SyncedFrame& input,
                                       data::LidarFrame* filtered) {
  if (filtered == nullptr) {
    return common::Status::InvalidArgument("filtered frame output is null");
  }

  *filtered = input.lidar;
  filtered->points.clear();
  filtered->points.reserve(input.lidar.points.size());

  std::unordered_set<std::uint64_t> occupied_voxels;
  occupied_voxels.reserve(input.lidar.points.size());

  for (const auto& point : input.lidar.points) {
    if (!PassesFilters(config_, point)) {
      continue;
    }
    const int vx = static_cast<int>(std::floor(point.x / config_.voxel_size_m));
    const int vy = static_cast<int>(std::floor(point.y / config_.voxel_size_m));
    const int vz = static_cast<int>(std::floor(point.z / config_.voxel_size_m));
    const std::uint64_t key = VoxelKey(vx, vy, vz);
    if (!occupied_voxels.insert(key).second) {
      continue;
    }
    filtered->points.push_back(point);
  }
  filtered->is_deskewed = input.lidar.is_deskewed;
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
