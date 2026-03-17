#pragma once

#include <string>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/mapping/map_builder_3d.hpp"
#include "rm_nav/mapping/map_projector_2d.hpp"
#include "rm_nav/mapping/map_serializer.hpp"

namespace rm_nav::mapping {

struct MappingResult {
  data::Pose3f map_to_base{};
  std::size_t accumulated_points{0};
  std::size_t processed_frames{0};
  common::TimeNs processing_latency_ns{0};
};

class MappingEngine {
 public:
  common::Status Initialize(const config::MappingConfig& config);
  common::Status Update(const data::SyncedFrame& frame, const data::Pose3f& map_to_base);
  common::Status BuildGridMap2D(data::GridMap2D* grid_map) const;
  std::vector<data::PointXYZI> GlobalPointCloud() const { return builder_.GlobalPoints(); }
  MappingResult LatestResult() const { return latest_result_; }
  common::Status SaveMap(const std::string& output_dir,
                         localization::StaticMap* exported_map = nullptr) const;

 private:
  config::MappingConfig config_{};
  MapBuilder3D builder_{};
  MapProjector2D projector_{};
  MapSerializer serializer_{};
  MappingResult latest_result_{};
  bool initialized_{false};
};

}  // namespace rm_nav::mapping
