#include "rm_nav/app/bootstrap.hpp"

#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::app {

RuntimeManifest BuildPhase0Manifest() {
  RuntimeManifest manifest;
  manifest.frames.assign(tf::kRequiredFrames.begin(), tf::kRequiredFrames.end());
  manifest.main_chain = {
      {"L1/IMU", ThreadDomain::kDriver},
      {"Sync", ThreadDomain::kSync},
      {"Localization/Mapping", ThreadDomain::kPoseCore},
      {"Preprocess", ThreadDomain::kPerception},
      {"Costmap/MOT", ThreadDomain::kPerception},
      {"Planner", ThreadDomain::kPlanner},
      {"Safety", ThreadDomain::kSafetyFsm},
      {"STM32", ThreadDomain::kDriver},
  };
  manifest.debug_chain = {
      {"MirrorTap", ThreadDomain::kDebug},
      {"DebugPublisher", ThreadDomain::kDebug},
      {"WebSocket", ThreadDomain::kDebug},
  };
  manifest.threads.assign(kThreadBindings.begin(), kThreadBindings.end());
  return manifest;
}

}  // namespace rm_nav::app
