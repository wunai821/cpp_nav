#include "rm_nav/tf/static_tf_registry.hpp"

#include "rm_nav/tf/frame_ids.hpp"
#include "rm_nav/tf/transform_query.hpp"
#include "rm_nav/data/tf_types.hpp"

namespace rm_nav::tf {

common::Status StaticTfRegistry::Register(const data::Pose3f& transform) {
  for (auto& entry : entries_) {
    if (entry.occupied &&
        entry.transform.reference_frame == transform.reference_frame &&
        entry.transform.child_frame == transform.child_frame) {
      entry.transform = transform;
      entry.transform.is_valid = true;
      return common::Status::Ok();
    }
  }

  for (auto& entry : entries_) {
    if (!entry.occupied) {
      entry.transform = transform;
      entry.transform.is_valid = true;
      entry.occupied = true;
      return common::Status::Ok();
    }
  }

  return common::Status::Unavailable("static tf registry is full");
}

std::optional<data::Pose3f> StaticTfRegistry::Lookup(std::string_view parent,
                                                     std::string_view child) const {
  for (const auto& entry : entries_) {
    if (!entry.occupied) {
      continue;
    }
    if (entry.transform.reference_frame == parent &&
        entry.transform.child_frame == child) {
      return entry.transform;
    }
    if (entry.transform.reference_frame == child &&
        entry.transform.child_frame == parent) {
      return Inverse(entry.transform);
    }
  }
  return std::nullopt;
}

}  // namespace rm_nav::tf
