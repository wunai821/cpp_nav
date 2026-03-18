#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/mapping/map_validator.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace {

rm_nav::data::Pose3f MakePose(float x, float y, float yaw) {
  rm_nav::data::Pose3f pose;
  pose.stamp = rm_nav::common::Now();
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.position.x = x;
  pose.position.y = y;
  pose.rpy.z = yaw;
  pose.is_valid = true;
  return pose;
}

rm_nav::data::GridMap2D MakeGrid() {
  rm_nav::data::GridMap2D grid;
  grid.width = 10U;
  grid.height = 10U;
  grid.resolution_m = 0.1F;
  grid.occupancy.assign(grid.width * grid.height, 0U);
  grid.occupancy[11] = 100U;
  grid.occupancy[22] = 100U;
  grid.occupancy[33] = 100U;
  return grid;
}

std::vector<rm_nav::data::PointXYZI> MakePoints() {
  return {
      {0.0F, 0.0F, 0.5F, 1.0F, 0.0F}, {0.2F, 0.0F, 0.5F, 1.0F, 0.0F},
      {0.4F, 0.1F, 0.5F, 1.0F, 0.0F}, {0.6F, 0.2F, 0.5F, 1.0F, 0.0F},
      {0.8F, 0.3F, 0.5F, 1.0F, 0.0F}};
}

}  // namespace

int main() {
  rm_nav::config::MappingConfig config;
  config.validation_min_global_points = 5;
  config.validation_min_occupied_cells = 3;
  config.validation_min_width = 8;
  config.validation_min_height = 8;
  config.validation_min_occupied_ratio = 0.001;
  config.validation_max_loop_translation_regression_m = 0.02;
  config.validation_max_loop_yaw_regression_rad = 0.02;

  std::vector<rm_nav::mapping::MappingTrajectorySample> improved_samples;
  {
    rm_nav::mapping::MappingTrajectorySample sample;
    sample.frame_index = 10U;
    sample.external_pose = MakePose(1.0F, 0.0F, 0.10F);
    sample.predicted_pose = sample.external_pose;
    sample.optimized_pose = MakePose(0.2F, 0.0F, 0.02F);
    sample.loop_consistency_evaluated = true;
    sample.loop_candidate_found = true;
    sample.loop_match_converged = true;
    sample.loop_correction_accepted = true;
    sample.raw_loop_translation_error_m = 1.0F;
    sample.raw_loop_yaw_error_rad = 0.10F;
    sample.optimized_loop_translation_error_m = 0.2F;
    sample.optimized_loop_yaw_error_rad = 0.02F;
    improved_samples.push_back(sample);
  }

  rm_nav::mapping::MapValidator validator;
  rm_nav::mapping::MapValidationReport report;
  assert(validator.Validate(config, MakePoints(), MakeGrid(), improved_samples, &report).ok());
  assert(report.passed);
  assert(report.trajectory_consistency_passed);
  assert(report.loop_consistency_sample_count == 1U);
  assert(report.loop_translation_gain_m > 0.7);
  assert(report.loop_yaw_gain_rad > 0.05);

  std::vector<rm_nav::mapping::MappingTrajectorySample> regressed_samples = improved_samples;
  regressed_samples.front().optimized_loop_translation_error_m = 1.2F;
  regressed_samples.front().optimized_loop_yaw_error_rad = 0.15F;
  const auto regression_status =
      validator.Validate(config, MakePoints(), MakeGrid(), regressed_samples, &report);
  assert(!regression_status.ok());
  assert(!report.passed);
  assert(!report.trajectory_consistency_passed);
  assert(report.summary.find("optimized loop translation consistency regressed") !=
         std::string::npos);

  std::cout << "test_map_validator_trajectory passed\n";
  return 0;
}
