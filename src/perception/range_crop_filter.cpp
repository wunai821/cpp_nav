#include "rm_nav/perception/range_crop_filter.hpp"

#include <cmath>

#include "rm_nav/perception/preprocess_pipeline.hpp"

namespace rm_nav::perception {
namespace {

bool PointInConvexQuad(const std::array<common::Vec2f, 4>& quad, float x, float y) {
  bool has_positive = false;
  bool has_negative = false;
  for (std::size_t index = 0; index < quad.size(); ++index) {
    const auto& a = quad[index];
    const auto& b = quad[(index + 1U) % quad.size()];
    const float cross = (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
    has_positive = has_positive || cross > 1.0e-6F;
    has_negative = has_negative || cross < -1.0e-6F;
    if (has_positive && has_negative) {
      return false;
    }
  }
  return true;
}

}  // namespace

common::Status RangeCropFilter::Configure(const PreprocessConfig& config) {
  min_range_m_ = config.min_range_m;
  max_range_m_ = config.max_range_m;
  blind_zone_radius_m_ = std::max(0.0F, config.blind_zone_radius_m);
  min_height_m_ = config.min_height_m;
  max_height_m_ = config.max_height_m;
  self_mask_enabled_ = config.self_mask_enabled;
  self_mask_polygon_ = config.self_mask_polygon;
  self_mask_polygon_valid_ = config.self_mask_polygon_valid;
  self_mask_x_min_m_ = config.self_mask_x_min_m;
  self_mask_x_max_m_ = config.self_mask_x_max_m;
  self_mask_y_min_m_ = config.self_mask_y_min_m;
  self_mask_y_max_m_ = config.self_mask_y_max_m;
  configured_ = true;
  return common::Status::Ok();
}

bool RangeCropFilter::PointInSelfMask(float x, float y) const {
  if (!self_mask_enabled_) {
    return false;
  }
  if (self_mask_polygon_valid_) {
    return PointInConvexQuad(self_mask_polygon_, x, y);
  }
  return x >= self_mask_x_min_m_ && x <= self_mask_x_max_m_ && y >= self_mask_y_min_m_ &&
         y <= self_mask_y_max_m_;
}

common::Status RangeCropFilter::Apply(const std::vector<data::PointXYZI>& input,
                                      std::vector<data::PointXYZI>* output) const {
  if (output == nullptr) {
    return common::Status::InvalidArgument("range crop output is null");
  }
  if (!configured_) {
    return common::Status::NotReady("range crop filter is not configured");
  }

  output->clear();
  output->reserve(input.size());
  for (const auto& point : input) {
    const float range_m = std::sqrt(point.x * point.x + point.y * point.y);
    if (range_m < min_range_m_ || range_m > max_range_m_ || range_m < blind_zone_radius_m_) {
      continue;
    }
    if (point.z < min_height_m_ || point.z > max_height_m_) {
      continue;
    }
    if (PointInSelfMask(point.x, point.y)) {
      continue;
    }
    if (point.x == 0.0F && point.y == 0.0F && point.z == 0.0F) {
      continue;
    }
    output->push_back(point);
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
