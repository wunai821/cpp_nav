#pragma once

#include <string>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/point_types.hpp"

namespace rm_nav::localization {

struct StaticMap {
  data::GridMap2D occupancy{};
  std::vector<data::PointXYZI> global_points{};
  std::string occupancy_path{};
  std::string global_map_path{};
  bool occupancy_loaded{false};
  bool global_map_loaded{false};
};

class MapLoader {
 public:
  common::Status Load(const std::string& config_dir,
                      const config::LocalizationConfig& config,
                      StaticMap* map) const;
};

}  // namespace rm_nav::localization
