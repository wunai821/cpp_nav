#pragma once

#include <string_view>

#include "rm_nav/common/status.hpp"
#include "rm_nav/tf/tf_tree_lite.hpp"

namespace rm_nav::tf {

common::Status LookupTransform(const TfTreeLite& tree, std::string_view parent,
                               std::string_view child, common::TimePoint stamp,
                               data::Pose3f* transform);

}  // namespace rm_nav::tf
