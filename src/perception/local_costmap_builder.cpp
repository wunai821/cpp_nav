#include "rm_nav/perception/local_costmap_builder.hpp"

namespace rm_nav::perception {

common::Status LocalCostmapBuilder::Configure(const LocalCostmapConfig& config) {
  if (config.width == 0U || config.height == 0U || config.resolution_m <= 0.0F ||
      config.inflation_radius_m < 0.0F || config.dynamic_obstacle_inflation_m < 0.0F) {
    return common::Status::InvalidArgument("invalid local costmap config");
  }
  config_ = config;
  auto status = obstacle_layer_.Configure(config_);
  if (!status.ok()) {
    return status;
  }
  status = inflation_layer_.Configure(config_);
  if (!status.ok()) {
    return status;
  }
  status = dynamic_layer_.Configure(config_);
  if (!status.ok()) {
    return status;
  }
  configured_ = true;
  return common::Status::Ok();
}

common::Status LocalCostmapBuilder::Build(const data::LidarFrame& filtered_frame,
                                          const data::Pose3f& pose,
                                          data::GridMap2D* costmap) {
  const std::vector<data::DynamicObstacle> no_obstacles;
  return Build(filtered_frame, pose, no_obstacles, costmap);
}

common::Status LocalCostmapBuilder::Build(const data::LidarFrame& filtered_frame,
                                          const data::Pose3f& pose,
                                          const std::vector<data::DynamicObstacle>& obstacles,
                                          data::GridMap2D* costmap) {
  if (costmap == nullptr) {
    return common::Status::InvalidArgument("costmap output is null");
  }
  if (!configured_) {
    const auto status = Configure({});
    if (!status.ok()) {
      return status;
    }
  }

  data::GridMap2D static_layer;
  static_layer.stamp = filtered_frame.stamp;
  static_layer.resolution_m = config_.resolution_m;
  static_layer.width = config_.width;
  static_layer.height = config_.height;
  static_layer.origin = pose;
  static_layer.occupancy.assign(static_cast<std::size_t>(config_.width) * config_.height, 0U);

  auto status = obstacle_layer_.Apply(filtered_frame, &static_layer);
  if (!status.ok()) {
    return status;
  }
  status = inflation_layer_.Apply(&static_layer);
  if (!status.ok()) {
    return status;
  }

  data::GridMap2D dynamic_layer;
  status = dynamic_layer_.Apply(pose, obstacles, static_layer, &dynamic_layer);
  if (!status.ok()) {
    return status;
  }

  latest_static_layer_.Publish(static_layer);
  latest_dynamic_layer_.Publish(dynamic_layer);
  *costmap = std::move(static_layer);
  return common::Status::Ok();
}

common::Status LocalCostmapBuilder::BuildAndPublish(const data::LidarFrame& filtered_frame,
                                                    const data::Pose3f& pose) {
  const std::vector<data::DynamicObstacle> no_obstacles;
  return BuildAndPublish(filtered_frame, pose, no_obstacles);
}

common::Status LocalCostmapBuilder::BuildAndPublish(
    const data::LidarFrame& filtered_frame, const data::Pose3f& pose,
    const std::vector<data::DynamicObstacle>& obstacles) {
  data::LocalCostmap costmap;
  const auto status = Build(filtered_frame, pose, obstacles, &costmap);
  if (!status.ok()) {
    return status;
  }
  latest_costmap_.Publish(std::move(costmap));
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
