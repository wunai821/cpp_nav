#include "rm_nav/localization/map_odom_fuser.hpp"

#include "rm_nav/data/tf_types.hpp"

namespace rm_nav::localization {

common::Status MapOdomFuser::Fuse(const data::Pose3f& odom_to_base,
                                  const data::Pose3f& map_to_base,
                                  data::Pose3f* map_to_odom) const {
  if (map_to_odom == nullptr) {
    return common::Status::InvalidArgument("map_to_odom output is null");
  }
  if (!odom_to_base.is_valid || !map_to_base.is_valid) {
    return common::Status::NotReady("pose inputs are not valid");
  }
  *map_to_odom = tf::Compose(map_to_base, tf::Inverse(odom_to_base));
  map_to_odom->reference_frame = tf::kMapFrame;
  map_to_odom->child_frame = tf::kOdomFrame;
  map_to_odom->is_valid = true;
  return common::Status::Ok();
}

}  // namespace rm_nav::localization
