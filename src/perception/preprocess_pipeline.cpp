#include "rm_nav/perception/preprocess_pipeline.hpp"

#include <cmath>

#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::perception {
namespace {

data::PointXYZI TransformPointToBase(const data::PointXYZI& point,
                                     const config::ExtrinsicConfig& mount) {
  const float cr = std::cos(mount.roll_rad);
  const float sr = std::sin(mount.roll_rad);
  const float cp = std::cos(mount.pitch_rad);
  const float sp = std::sin(mount.pitch_rad);
  const float cy = std::cos(mount.yaw_rad);
  const float sy = std::sin(mount.yaw_rad);

  data::PointXYZI transformed = point;
  transformed.x = mount.x_m + (cy * cp) * point.x + (cy * sp * sr - sy * cr) * point.y +
                  (cy * sp * cr + sy * sr) * point.z;
  transformed.y = mount.y_m + (sy * cp) * point.x + (sy * sp * sr + cy * cr) * point.y +
                  (sy * sp * cr - cy * sr) * point.z;
  transformed.z = mount.z_m + (-sp) * point.x + (cp * sr) * point.y + (cp * cr) * point.z;
  return transformed;
}

}  // namespace

common::Status PreprocessPipeline::Configure(const PreprocessConfig& config) {
  if (config.min_range_m < 0.0F || config.max_range_m <= config.min_range_m ||
      config.max_height_m <= config.min_height_m || config.voxel_size_m <= 0.0F ||
      config.blind_zone_radius_m < 0.0F || config.ground_margin_m < 0.0F ||
      config.self_mask_x_max_m < config.self_mask_x_min_m ||
      config.self_mask_y_max_m < config.self_mask_y_min_m) {
    return common::Status::InvalidArgument("invalid preprocess config");
  }
  config_ = config;
  expected_filtered_points_ = std::max<std::size_t>(config_.expected_input_points, 32U);
  if (config_.self_mask_enabled && !config_.self_mask_polygon_valid) {
    config_.self_mask_polygon = {{{config_.self_mask_x_min_m, config_.self_mask_y_min_m},
                                  {config_.self_mask_x_max_m, config_.self_mask_y_min_m},
                                  {config_.self_mask_x_max_m, config_.self_mask_y_max_m},
                                  {config_.self_mask_x_min_m, config_.self_mask_y_max_m}}};
    config_.self_mask_polygon_valid = true;
  }
  dropped_frames_ = 0;
  auto status = range_crop_filter_.Configure(config_);
  if (!status.ok()) {
    return status;
  }
  status = ground_filter_.Configure(config_);
  if (!status.ok()) {
    return status;
  }
  status = voxel_filter_.Configure(config_);
  if (!status.ok()) {
    return status;
  }
  cropped_points_.clear();
  nonground_points_.clear();
  transformed_points_.clear();
  cropped_points_.reserve(expected_filtered_points_);
  nonground_points_.reserve(expected_filtered_points_);
  transformed_points_.reserve(expected_filtered_points_);
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
  pending_inputs_.fetch_add(1, std::memory_order_release);
  input_wait_cv_.notify_one();
  return common::Status::Ok();
}

bool PreprocessPipeline::WaitForInput(std::chrono::milliseconds timeout) const {
  if (pending_inputs_.load(std::memory_order_acquire) > 0U) {
    return true;
  }
  std::unique_lock<std::mutex> lock(input_wait_mutex_);
  return input_wait_cv_.wait_for(lock, timeout, [this]() {
    return pending_inputs_.load(std::memory_order_acquire) > 0U;
  });
}

common::Status PreprocessPipeline::ProcessOnce() {
  sync::SyncedFrameHandle input;
  if (!input_queue_.try_pop(&input)) {
    return common::Status::NotReady("no synced frame for preprocess");
  }
  const auto pending = pending_inputs_.load(std::memory_order_acquire);
  if (pending > 0U) {
    pending_inputs_.fetch_sub(1, std::memory_order_acq_rel);
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
  filtered->frame_id = tf::kBaseLinkFrame;
  filtered->points.clear();
  filtered->points.MutableView().reserve(expected_filtered_points_);
  transformed_points_.clear();
  cropped_points_.clear();
  nonground_points_.clear();
  transformed_points_.reserve(std::max(expected_filtered_points_, input.lidar.points.size()));
  if (input.lidar.frame_id == tf::kBaseLinkFrame) {
    transformed_points_.insert(transformed_points_.end(), input.lidar.points.begin(),
                               input.lidar.points.end());
  } else {
    for (const auto& point : input.lidar.points) {
      transformed_points_.push_back(TransformPointToBase(point, config_.lidar_mount));
    }
  }
  auto status = range_crop_filter_.Apply(transformed_points_, &cropped_points_);
  if (!status.ok()) {
    return status;
  }
  status = ground_filter_.Apply(cropped_points_, &nonground_points_);
  if (!status.ok()) {
    return status;
  }
  status = voxel_filter_.Apply(nonground_points_, &filtered->points.MutableView());
  if (!status.ok()) {
    return status;
  }
  filtered->is_deskewed = input.lidar.is_deskewed;
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
