#include "rm_nav/mapping/map_validator.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

namespace rm_nav::mapping {
namespace {

std::string JoinReasons(const std::vector<std::string>& reasons) {
  std::ostringstream stream;
  for (std::size_t index = 0; index < reasons.size(); ++index) {
    if (index != 0U) {
      stream << "; ";
    }
    stream << reasons[index];
  }
  return stream.str();
}

}  // namespace

common::Status MapValidator::Validate(const config::MappingConfig& config,
                                      const std::vector<data::PointXYZI>& global_points,
                                      const data::GridMap2D& occupancy,
                                      const std::vector<MappingTrajectorySample>& trajectory_samples,
                                      MapValidationReport* report) const {
  if (report == nullptr) {
    return common::Status::InvalidArgument("validation report must not be null");
  }

  MapValidationReport local_report;
  local_report.global_point_count = global_points.size();
  local_report.trajectory_sample_count = trajectory_samples.size();
  local_report.width = occupancy.width;
  local_report.height = occupancy.height;
  local_report.resolution_m = occupancy.resolution_m;

  for (const auto value : occupancy.occupancy) {
    if (value > 0U) {
      ++local_report.occupied_cell_count;
    }
  }

  const std::size_t total_cells =
      static_cast<std::size_t>(occupancy.width) * static_cast<std::size_t>(occupancy.height);
  if (total_cells > 0U) {
    local_report.occupied_ratio =
        static_cast<double>(local_report.occupied_cell_count) /
        static_cast<double>(total_cells);
  }

  std::vector<std::string> reasons;
  if (global_points.size() <
      static_cast<std::size_t>(std::max(0, config.validation_min_global_points))) {
    reasons.push_back("global point count below threshold");
  }
  if (local_report.occupied_cell_count <
      static_cast<std::size_t>(std::max(0, config.validation_min_occupied_cells))) {
    reasons.push_back("occupied cell count below threshold");
  }
  if (occupancy.width < static_cast<std::uint32_t>(std::max(0, config.validation_min_width))) {
    reasons.push_back("occupancy width below threshold");
  }
  if (occupancy.height < static_cast<std::uint32_t>(std::max(0, config.validation_min_height))) {
    reasons.push_back("occupancy height below threshold");
  }
  if (occupancy.resolution_m <= 0.0F) {
    reasons.push_back("occupancy resolution is invalid");
  }
  if (local_report.occupied_ratio < config.validation_min_occupied_ratio) {
    reasons.push_back("occupied ratio below threshold");
  }

  double raw_translation_sum = 0.0;
  double optimized_translation_sum = 0.0;
  double raw_yaw_sum = 0.0;
  double optimized_yaw_sum = 0.0;
  for (const auto& sample : trajectory_samples) {
    if (!sample.loop_consistency_evaluated) {
      continue;
    }
    ++local_report.loop_consistency_sample_count;
    raw_translation_sum += sample.raw_loop_translation_error_m;
    optimized_translation_sum += sample.optimized_loop_translation_error_m;
    raw_yaw_sum += std::fabs(sample.raw_loop_yaw_error_rad);
    optimized_yaw_sum += std::fabs(sample.optimized_loop_yaw_error_rad);
  }

  if (local_report.loop_consistency_sample_count > 0U) {
    const double count =
        static_cast<double>(local_report.loop_consistency_sample_count);
    local_report.raw_loop_translation_error_mean_m = raw_translation_sum / count;
    local_report.optimized_loop_translation_error_mean_m =
        optimized_translation_sum / count;
    local_report.raw_loop_yaw_error_mean_rad = raw_yaw_sum / count;
    local_report.optimized_loop_yaw_error_mean_rad = optimized_yaw_sum / count;
    local_report.loop_translation_gain_m =
        local_report.raw_loop_translation_error_mean_m -
        local_report.optimized_loop_translation_error_mean_m;
    local_report.loop_yaw_gain_rad =
        local_report.raw_loop_yaw_error_mean_rad -
        local_report.optimized_loop_yaw_error_mean_rad;

    if (local_report.optimized_loop_translation_error_mean_m >
        local_report.raw_loop_translation_error_mean_m +
            config.validation_max_loop_translation_regression_m) {
      local_report.trajectory_consistency_passed = false;
      reasons.push_back("optimized loop translation consistency regressed");
    }
    if (local_report.optimized_loop_yaw_error_mean_rad >
        local_report.raw_loop_yaw_error_mean_rad +
            config.validation_max_loop_yaw_regression_rad) {
      local_report.trajectory_consistency_passed = false;
      reasons.push_back("optimized loop yaw consistency regressed");
    }
  }

  local_report.passed = reasons.empty();
  local_report.summary = local_report.passed ? "pass" : JoinReasons(reasons);
  *report = local_report;
  return local_report.passed ? common::Status::Ok()
                             : common::Status::InvalidArgument(local_report.summary);
}

common::Status MapValidator::WriteReport(const std::filesystem::path& report_path,
                                         const MapValidationReport& report) const {
  std::filesystem::create_directories(report_path.parent_path());
  std::ofstream output(report_path);
  if (!output.is_open()) {
    return common::Status::Unavailable("failed to open map_validation_report.json");
  }
  output << "{\n";
  output << "  \"passed\": " << (report.passed ? "true" : "false") << ",\n";
  output << "  \"trajectory_consistency_passed\": "
         << (report.trajectory_consistency_passed ? "true" : "false") << ",\n";
  output << "  \"global_point_count\": " << report.global_point_count << ",\n";
  output << "  \"occupied_cell_count\": " << report.occupied_cell_count << ",\n";
  output << "  \"trajectory_sample_count\": " << report.trajectory_sample_count << ",\n";
  output << "  \"loop_consistency_sample_count\": " << report.loop_consistency_sample_count
         << ",\n";
  output << "  \"occupied_ratio\": " << report.occupied_ratio << ",\n";
  output << "  \"raw_loop_translation_error_mean_m\": "
         << report.raw_loop_translation_error_mean_m << ",\n";
  output << "  \"optimized_loop_translation_error_mean_m\": "
         << report.optimized_loop_translation_error_mean_m << ",\n";
  output << "  \"raw_loop_yaw_error_mean_rad\": "
         << report.raw_loop_yaw_error_mean_rad << ",\n";
  output << "  \"optimized_loop_yaw_error_mean_rad\": "
         << report.optimized_loop_yaw_error_mean_rad << ",\n";
  output << "  \"loop_translation_gain_m\": " << report.loop_translation_gain_m << ",\n";
  output << "  \"loop_yaw_gain_rad\": " << report.loop_yaw_gain_rad << ",\n";
  output << "  \"width\": " << report.width << ",\n";
  output << "  \"height\": " << report.height << ",\n";
  output << "  \"resolution_m\": " << report.resolution_m << ",\n";
  output << "  \"summary\": \"" << report.summary << "\"\n";
  output << "}\n";
  return output.good() ? common::Status::Ok()
                       : common::Status::InternalError("failed to write map_validation_report.json");
}

}  // namespace rm_nav::mapping
