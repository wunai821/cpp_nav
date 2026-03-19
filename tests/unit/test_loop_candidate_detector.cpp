#include <cassert>
#include <iostream>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/mapping/loop_candidate_detector.hpp"
#include "rm_nav/mapping/mapping_engine.hpp"
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

rm_nav::mapping::MappingKeyframe MakeKeyframe(std::uint32_t frame_index, rm_nav::common::TimePoint stamp,
                                              float x, float y, float yaw) {
  rm_nav::mapping::MappingKeyframe keyframe;
  keyframe.stamp = stamp;
  keyframe.frame_index = frame_index;
  keyframe.map_to_base = MakePose(x, y, yaw);
  keyframe.map_to_base.stamp = stamp;
  return keyframe;
}

}  // namespace

int main() {
  rm_nav::config::MappingConfig config;
  config.loop_candidate_distance_threshold_m = 1.0;
  config.loop_candidate_min_time_separation_s = 5.0;
  config.loop_candidate_max_yaw_delta_rad = 0.5;
  config.loop_candidate_min_revisit_score = 0.35;

  const auto now = rm_nav::common::Now();
  std::vector<rm_nav::mapping::MappingKeyframe> keyframes;
  keyframes.push_back(MakeKeyframe(
      10U, now - std::chrono::seconds(9), 0.2F, 0.1F, 0.05F));
  keyframes.push_back(MakeKeyframe(
      20U, now - std::chrono::seconds(2), 0.15F, 0.1F, 0.05F));
  keyframes.push_back(MakeKeyframe(
      30U, now - std::chrono::seconds(8), 0.3F, 0.2F, 1.2F));

  rm_nav::mapping::LoopCandidateDetector detector;
  rm_nav::mapping::LoopClosureCandidate candidate;
  auto status = detector.FindCandidate(config, MakePose(0.0F, 0.0F, 0.0F), now, 100U, keyframes,
                                       &candidate);
  assert(status.ok());
  assert(candidate.found);
  assert(candidate.frame_index == 10U);
  assert(candidate.keyframe_index == 0U);
  assert(candidate.revisit_score >= 0.5F);

  status = detector.FindCandidate(config, MakePose(3.0F, 3.0F, 0.0F), now, 101U, keyframes,
                                  &candidate);
  assert(status.ok());
  assert(!candidate.found);

  std::vector<rm_nav::mapping::MappingKeyframe> yaw_reject_keyframes;
  yaw_reject_keyframes.push_back(
      MakeKeyframe(40U, now - std::chrono::seconds(8), 0.2F, 0.1F, 0.05F));
  status = detector.FindCandidate(config, MakePose(0.2F, 0.1F, 1.3F), now, 102U,
                                  yaw_reject_keyframes, &candidate);
  assert(status.ok());
  assert(!candidate.found);

  std::cout << "test_loop_candidate_detector passed\n";
  return 0;
}
