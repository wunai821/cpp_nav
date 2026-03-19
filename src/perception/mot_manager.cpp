#include "rm_nav/perception/mot_manager.hpp"

#include "rm_nav/common/time.hpp"

namespace rm_nav::perception {

common::Status MotManager::Configure(const MotConfig& config) {
  if (config.cluster_tolerance_m <= 0.0F || config.min_cluster_points == 0U ||
      config.association_distance_m <= 0.0F || config.max_missed_frames == 0U) {
    return common::Status::InvalidArgument("invalid mot config");
  }
  config_ = config;
  auto status = clusterer_.Configure(config_);
  if (!status.ok()) {
    return status;
  }
  status = tracker_.Configure(config_);
  if (!status.ok()) {
    return status;
  }
  configured_ = true;
  return common::Status::Ok();
}

common::Status MotManager::Update(const data::LidarFrame& filtered_frame,
                                  const data::Pose3f& pose,
                                  std::vector<data::DynamicObstacle>* obstacles) {
  if (obstacles == nullptr) {
    return common::Status::InvalidArgument("obstacle output is null");
  }
  if (!configured_) {
    Configure({});
  }

  const auto begin_ns = common::NowNs();
  obstacles->clear();
  const auto clustering_begin_ns = common::NowNs();
  std::vector<Cluster> clusters;
  auto status = clusterer_.Build(filtered_frame, pose, reduce_roi_next_cycle_, &clusters);
  if (!status.ok()) {
    return status;
  }
  const auto clustering_latency_ns = common::NowNs() - clustering_begin_ns;
  status = tracker_.Update(clusters, filtered_frame.stamp, obstacles);
  if (!status.ok()) {
    return status;
  }

  MotPerfSnapshot perf;
  perf.clustering_latency_ns = clustering_latency_ns;
  perf.total_update_latency_ns = common::NowNs() - begin_ns;
  perf.active_roi_radius_m =
      reduce_roi_next_cycle_ ? config_.reduced_roi_radius_m : config_.roi_radius_m;
  perf.reduced_roi_active = reduce_roi_next_cycle_;
  latest_perf_.Publish(perf);
  reduce_roi_next_cycle_ =
      perf.total_update_latency_ns >
      static_cast<common::TimeNs>(config_.clustering_budget_ms) * 1000000LL;
  return common::Status::Ok();
}

common::Status MotManager::UpdateAndPublish(const data::LidarFrame& filtered_frame,
                                            const data::Pose3f& pose) {
  data::DynamicObstacleSet obstacle_set;
  obstacle_set.stamp = filtered_frame.stamp;
  const auto status = Update(filtered_frame, pose, &obstacle_set.obstacles);
  if (!status.ok()) {
    return status;
  }
  latest_obstacles_.Publish(std::move(obstacle_set));
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
