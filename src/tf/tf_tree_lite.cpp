#include "rm_nav/tf/tf_tree_lite.hpp"

#include "rm_nav/data/tf_types.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::tf {
namespace {

bool Matches(std::string_view parent, std::string_view child,
             std::string_view expected_parent, std::string_view expected_child) {
  return parent == expected_parent && child == expected_child;
}

}  // namespace

common::Status TfTreeLite::RegisterStaticTransform(const data::Pose3f& transform) {
  return static_registry_.Register(transform);
}

common::Status TfTreeLite::PushMapToOdom(const data::Pose3f& transform) {
  if (!Matches(transform.reference_frame, transform.child_frame, tf::kMapFrame,
               tf::kOdomFrame)) {
    return common::Status::InvalidArgument("expected map->odom transform");
  }
  map_to_odom_buffer_.Push(transform);
  return common::Status::Ok();
}

common::Status TfTreeLite::PushOdomToBase(const data::Pose3f& transform) {
  if (!Matches(transform.reference_frame, transform.child_frame, tf::kOdomFrame,
               tf::kBaseLinkFrame)) {
    return common::Status::InvalidArgument("expected odom->base_link transform");
  }
  odom_to_base_buffer_.Push(transform);
  return common::Status::Ok();
}

std::optional<data::Pose3f> TfTreeLite::Lookup(std::string_view parent,
                                               std::string_view child,
                                               common::TimePoint stamp) const {
  if (parent == child) {
    return MakeTransform(parent, child, 0.0F, 0.0F, 0.0F, stamp);
  }

  if (Matches(parent, child, tf::kMapFrame, tf::kOdomFrame)) {
    return map_to_odom_buffer_.LookupNearest(stamp);
  }
  if (Matches(parent, child, tf::kOdomFrame, tf::kBaseLinkFrame)) {
    return odom_to_base_buffer_.LookupNearest(stamp);
  }
  if (Matches(parent, child, tf::kOdomFrame, tf::kMapFrame)) {
    const auto direct = map_to_odom_buffer_.LookupNearest(stamp);
    return direct.has_value() ? std::optional<data::Pose3f>(Inverse(*direct))
                              : std::nullopt;
  }
  if (Matches(parent, child, tf::kBaseLinkFrame, tf::kOdomFrame)) {
    const auto direct = odom_to_base_buffer_.LookupNearest(stamp);
    return direct.has_value() ? std::optional<data::Pose3f>(Inverse(*direct))
                              : std::nullopt;
  }

  if (Matches(parent, child, tf::kMapFrame, tf::kBaseLinkFrame)) {
    const auto map_to_odom = map_to_odom_buffer_.LookupNearest(stamp);
    const auto odom_to_base = odom_to_base_buffer_.LookupNearest(stamp);
    if (map_to_odom.has_value() && odom_to_base.has_value()) {
      return Compose(*map_to_odom, *odom_to_base);
    }
    return std::nullopt;
  }
  if (Matches(parent, child, tf::kBaseLinkFrame, tf::kMapFrame)) {
    const auto forward = Lookup(tf::kMapFrame, tf::kBaseLinkFrame, stamp);
    return forward.has_value() ? std::optional<data::Pose3f>(Inverse(*forward))
                               : std::nullopt;
  }

  if (const auto direct = static_registry_.Lookup(parent, child); direct.has_value()) {
    auto result = *direct;
    result.stamp = stamp;
    return result;
  }

  if (child == tf::kLaserFrame || child == tf::kImuFrame) {
    const auto parent_to_base = Lookup(parent, tf::kBaseLinkFrame, stamp);
    const auto base_to_sensor = static_registry_.Lookup(tf::kBaseLinkFrame, child);
    if (parent_to_base.has_value() && base_to_sensor.has_value()) {
      auto result = Compose(*parent_to_base, *base_to_sensor);
      result.stamp = stamp;
      return result;
    }
  }

  return std::nullopt;
}

}  // namespace rm_nav::tf
