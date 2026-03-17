#pragma once

#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/point_types.hpp"

namespace rm_nav::mapping {

class MapProjector2D {
 public:
  common::Status Configure(const config::MappingConfig& config);
  common::Status Project(const std::vector<data::PointXYZI>& global_points,
                         data::GridMap2D* occupancy) const;

 private:
  float z_min_m_{0.2F};
  float z_max_m_{1.2F};
  float resolution_m_{0.1F};
  float padding_m_{1.0F};
};

}  // namespace rm_nav::mapping
