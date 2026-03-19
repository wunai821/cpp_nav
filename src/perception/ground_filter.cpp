#include "rm_nav/perception/ground_filter.hpp"

#include <cmath>

#include "rm_nav/perception/preprocess_pipeline.hpp"

namespace rm_nav::perception {

common::Status GroundFilter::Configure(const PreprocessConfig& config) {
  ground_z_max_m_ = config.ground_z_max_m;
  ground_margin_m_ = std::max(0.0F, config.ground_margin_m);
  configured_ = true;
  return common::Status::Ok();
}

common::Status GroundFilter::Apply(const std::vector<data::PointXYZI>& input,
                                   std::vector<data::PointXYZI>* output) const {
  if (output == nullptr) {
    return common::Status::InvalidArgument("ground filter output is null");
  }
  if (!configured_) {
    return common::Status::NotReady("ground filter is not configured");
  }

  output->clear();
  output->reserve(input.size());
  for (const auto& point : input) {
    const float range_xy = std::sqrt(point.x * point.x + point.y * point.y);
    const float adaptive_ground_z = ground_z_max_m_ + ground_margin_m_ * std::min(range_xy, 1.0F);
    if (point.z <= adaptive_ground_z) {
      continue;
    }
    output->push_back(point);
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
