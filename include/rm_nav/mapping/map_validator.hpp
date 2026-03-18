#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/point_types.hpp"
#include "rm_nav/mapping/mapping_trajectory.hpp"

namespace rm_nav::mapping {

struct MapValidationReport {
  bool passed{false};
  bool trajectory_consistency_passed{true};
  std::size_t global_point_count{0};
  std::size_t occupied_cell_count{0};
  std::size_t trajectory_sample_count{0};
  std::size_t loop_consistency_sample_count{0};
  double occupied_ratio{0.0};
  double raw_loop_translation_error_mean_m{0.0};
  double optimized_loop_translation_error_mean_m{0.0};
  double raw_loop_yaw_error_mean_rad{0.0};
  double optimized_loop_yaw_error_mean_rad{0.0};
  double loop_translation_gain_m{0.0};
  double loop_yaw_gain_rad{0.0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  double resolution_m{0.0};
  std::string summary{"not_run"};
};

class MapValidator {
 public:
  common::Status Validate(const config::MappingConfig& config,
                          const std::vector<data::PointXYZI>& global_points,
                          const data::GridMap2D& occupancy,
                          const std::vector<MappingTrajectorySample>& trajectory_samples,
                          MapValidationReport* report) const;
  common::Status WriteReport(const std::filesystem::path& report_path,
                             const MapValidationReport& report) const;
};

}  // namespace rm_nav::mapping
