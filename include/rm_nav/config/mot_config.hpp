#pragma once

#include <cstddef>
#include <cstdint>

namespace rm_nav::config {

struct MotConfig {
  double cluster_tolerance_m{0.45};
  std::size_t min_cluster_points{3};
  std::size_t max_cluster_points{128};
  double roi_radius_m{6.0};
  double reduced_roi_radius_m{3.5};
  double association_distance_m{1.0};
  std::uint32_t max_missed_frames{4};
  double process_noise{0.08};
  double measurement_noise{0.15};
  double initial_confidence{0.45};
  double confirmation_confidence{0.6};
  int clustering_budget_ms{3};
};

}  // namespace rm_nav::config
