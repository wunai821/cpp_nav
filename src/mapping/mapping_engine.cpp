#include "rm_nav/mapping/mapping_engine.hpp"

#include <utility>

#include "rm_nav/common/time.hpp"

namespace rm_nav::mapping {

common::Status MappingEngine::Initialize(const config::MappingConfig& config) {
  config_ = config;
  latest_result_ = {};
  auto status = builder_.Configure(config);
  if (!status.ok()) {
    return status;
  }
  status = projector_.Configure(config);
  if (!status.ok()) {
    return status;
  }
  initialized_ = true;
  return common::Status::Ok();
}

common::Status MappingEngine::Update(const data::SyncedFrame& frame,
                                     const data::Pose3f& map_to_base) {
  if (!initialized_) {
    return common::Status::NotReady("mapping engine is not initialized");
  }
  const auto begin_ns = common::NowNs();
  auto status = builder_.Update(frame, map_to_base);
  if (!status.ok()) {
    return status;
  }
  latest_result_.map_to_base = map_to_base;
  latest_result_.accumulated_points = builder_.GlobalPoints().size();
  latest_result_.processed_frames = builder_.frame_count();
  latest_result_.processing_latency_ns = common::NowNs() - begin_ns;
  return common::Status::Ok();
}

common::Status MappingEngine::BuildGridMap2D(data::GridMap2D* grid_map) const {
  if (!initialized_) {
    return common::Status::NotReady("mapping engine is not initialized");
  }
  return projector_.Project(builder_.GlobalPoints(), grid_map);
}

common::Status MappingEngine::SaveMap(const std::string& output_dir,
                                      localization::StaticMap* exported_map) const {
  if (!initialized_) {
    return common::Status::NotReady("mapping engine is not initialized");
  }
  data::GridMap2D grid_map;
  auto status = BuildGridMap2D(&grid_map);
  if (!status.ok()) {
    return status;
  }

  const auto global_points = builder_.GlobalPoints();
  const std::string resolved_output_dir =
      output_dir.empty() ? config_.output_dir : output_dir;
  mapping::MapArtifactPaths artifacts;
  status = serializer_.Save(resolved_output_dir, global_points, grid_map, &artifacts);
  if (!status.ok()) {
    return status;
  }

  if (exported_map != nullptr) {
    exported_map->occupancy = grid_map;
    exported_map->global_points = global_points;
    exported_map->occupancy_path = artifacts.occupancy_bin_path;
    exported_map->global_map_path = artifacts.global_map_pcd_path;
    exported_map->occupancy_loaded = true;
    exported_map->global_map_loaded = true;
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
