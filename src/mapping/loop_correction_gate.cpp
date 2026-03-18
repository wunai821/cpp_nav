#include "rm_nav/mapping/loop_correction_gate.hpp"

#include <algorithm>

namespace rm_nav::mapping {
namespace {

const char* ReasonMessage(LoopCorrectionDecisionReason reason) {
  switch (reason) {
    case LoopCorrectionDecisionReason::kNoCandidate:
      return "no loop candidate";
    case LoopCorrectionDecisionReason::kMatcherFailed:
      return "loop matcher failed";
    case LoopCorrectionDecisionReason::kNotConverged:
      return "loop matcher did not converge";
    case LoopCorrectionDecisionReason::kScoreTooLow:
      return "loop match score below threshold";
    case LoopCorrectionDecisionReason::kTranslationTooLarge:
      return "loop correction translation too large";
    case LoopCorrectionDecisionReason::kYawTooLarge:
      return "loop correction yaw too large";
    case LoopCorrectionDecisionReason::kTooManyRecentFailures:
      return "too many recent loop correction failures";
    case LoopCorrectionDecisionReason::kAccepted:
      return "loop correction accepted";
    case LoopCorrectionDecisionReason::kNone:
    default:
      return "not evaluated";
  }
}

}  // namespace

common::Status LoopCorrectionGate::Evaluate(const config::MappingConfig& config,
                                            const LoopClosureCandidate& candidate,
                                            const LoopClosureMatchResult& match,
                                            int consecutive_failures_before,
                                            LoopCorrectionDecision* decision) const {
  if (decision == nullptr) {
    return common::Status::InvalidArgument("loop correction decision output must not be null");
  }

  *decision = {};
  decision->evaluated = true;
  decision->candidate_found = candidate.found;
  decision->keyframe_index = candidate.keyframe_index;
  decision->frame_index = candidate.frame_index;
  decision->score = match.score;
  decision->iterations = match.iterations;
  decision->translation_correction_m = match.translation_correction_m;
  decision->yaw_correction_rad = match.yaw_correction_rad;
  decision->consecutive_failures_before = consecutive_failures_before;

  if (!candidate.found) {
    decision->reason = LoopCorrectionDecisionReason::kNoCandidate;
    decision->reason_message = ReasonMessage(decision->reason);
    decision->consecutive_failures_after = consecutive_failures_before;
    return common::Status::Ok();
  }

  if (consecutive_failures_before >= config.loop_correction_max_consecutive_failures) {
    decision->reason = LoopCorrectionDecisionReason::kTooManyRecentFailures;
  } else if (match.status_code != common::StatusCode::kOk) {
    decision->reason = LoopCorrectionDecisionReason::kMatcherFailed;
  } else if (!match.converged) {
    decision->reason = LoopCorrectionDecisionReason::kNotConverged;
  } else if (match.score < static_cast<float>(config.loop_correction_min_score)) {
    decision->reason = LoopCorrectionDecisionReason::kScoreTooLow;
  } else if (match.translation_correction_m >
             static_cast<float>(config.loop_correction_max_translation_m)) {
    decision->reason = LoopCorrectionDecisionReason::kTranslationTooLarge;
  } else if (match.yaw_correction_rad >
             static_cast<float>(config.loop_correction_max_yaw_rad)) {
    decision->reason = LoopCorrectionDecisionReason::kYawTooLarge;
  } else {
    decision->accepted = true;
    decision->reason = LoopCorrectionDecisionReason::kAccepted;
    decision->accepted_pose_candidate_frame = match.matched_pose_candidate_frame;
  }

  if (decision->accepted) {
    decision->consecutive_failures_after = 0;
  } else if (candidate.found) {
    decision->consecutive_failures_after =
        std::min(consecutive_failures_before + 1,
                 std::max(1, config.loop_correction_max_consecutive_failures + 1));
  } else {
    decision->consecutive_failures_after = consecutive_failures_before;
  }
  decision->reason_message = ReasonMessage(decision->reason);
  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
