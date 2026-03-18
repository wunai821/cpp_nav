#pragma once

#include <filesystem>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/point_types.hpp"
#include "rm_nav/mapping/map_serializer.hpp"
#include "rm_nav/mapping/mapping_trajectory.hpp"
#include "rm_nav/mapping/map_validator.hpp"

namespace rm_nav::mapping {

enum class MapSaveFailureKind {
  kNone = 0,
  kWriteFailed,
  kValidationFailed,
  kStorageSwitchFailed,
};

struct MapStorageLayout {
  std::filesystem::path active_dir{};
  std::filesystem::path staging_dir{};
  std::filesystem::path last_good_dir{};
  std::filesystem::path failed_dir{};
};

class MapSaveManager {
 public:
  common::Status SaveAndActivate(const MapStorageLayout& layout,
                                 const config::MappingConfig& config,
                                 const std::vector<data::PointXYZI>& global_points,
                                 const data::GridMap2D& occupancy,
                                 const std::vector<MappingTrajectorySample>& trajectory_samples,
                                 MapArtifactPaths* artifact_paths = nullptr,
                                 MapValidationReport* validation_report = nullptr,
                                 MapSaveFailureKind* failure_kind = nullptr) const;

 private:
  MapSerializer serializer_{};
  MapValidator validator_{};
};

}  // namespace rm_nav::mapping
