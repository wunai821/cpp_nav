#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/point_types.hpp"

namespace rm_nav::mapping {

struct MapArtifactPaths {
  std::string global_map_pcd_path{};
  std::string occupancy_bin_path{};
  std::string occupancy_png_path{};
  std::string map_meta_path{};
};

class MapSerializer {
 public:
  common::Status Save(const std::filesystem::path& output_dir,
                      const std::vector<data::PointXYZI>& global_points,
                      const data::GridMap2D& occupancy,
                      MapArtifactPaths* artifact_paths = nullptr) const;
};

}  // namespace rm_nav::mapping
