#include "rm_nav/localization/ndt_matcher.hpp"

namespace rm_nav::localization {

common::Status NdtMatcher::Configure(const ScanMatchConfig&) {
  return common::Status::Ok();
}

common::Status NdtMatcher::Match(const StaticMap&, const data::LidarFrame&,
                                 const data::Pose3f&, ScanMatchResult*) const {
  return common::Status::Unimplemented("ndt matcher is reserved but not implemented");
}

}  // namespace rm_nav::localization
