#include "rm_nav/perception/preprocess_pipeline.hpp"

namespace rm_nav::perception {

common::Status PreprocessPipeline::Configure(const PreprocessConfig& config) {
  if (config.min_range_m < 0.0F || config.max_range_m <= config.min_range_m ||
      config.max_height_m <= config.min_height_m || config.voxel_size_m <= 0.0F ||
      config.blind_zone_radius_m < 0.0F || config.ground_margin_m < 0.0F ||
      config.self_mask_x_max_m < config.self_mask_x_min_m ||
      config.self_mask_y_max_m < config.self_mask_y_min_m) {
    return common::Status::InvalidArgument("invalid preprocess config");
  }
  config_ = config;
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
  std::vector<data::PointXYZI> cropped_points;
  std::vector<data::PointXYZI> nonground_points;
  auto status = range_crop_filter_.Apply(input.lidar, &cropped_points);
  if (!status.ok()) {
    return status;
  }
  status = ground_filter_.Apply(cropped_points, &nonground_points);
  if (!status.ok()) {
    return status;
  }
  status = voxel_filter_.Apply(nonground_points, &filtered->points);
  if (!status.ok()) {
    return status;
  }
  filtered->is_deskewed = input.lidar.is_deskewed;
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
