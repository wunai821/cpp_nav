#include <cassert>
#include <iostream>

#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/mapping/loop_correction_gate.hpp"

int main() {
  rm_nav::config::MappingConfig config;
  config.loop_correction_min_score = 0.6;
  config.loop_correction_max_translation_m = 0.25;
  config.loop_correction_max_yaw_rad = 0.15;
  config.loop_correction_max_consecutive_failures = 3;

  rm_nav::mapping::LoopClosureCandidate candidate;
  candidate.found = true;
  candidate.keyframe_index = 2U;
  candidate.frame_index = 42U;

  rm_nav::mapping::LoopClosureMatchResult match;
  match.attempted = true;
  match.candidate_found = true;
  match.converged = true;
  match.score = 0.92F;
  match.iterations = 4;
  match.translation_correction_m = 0.08F;
  match.yaw_correction_rad = 0.04F;
  match.status_code = rm_nav::common::StatusCode::kOk;
  match.matched_pose_candidate_frame.is_valid = true;

  rm_nav::mapping::LoopCorrectionGate gate;
  rm_nav::mapping::LoopCorrectionDecision decision;
  assert(gate.Evaluate(config, candidate, match, 0, &decision).ok());
  assert(decision.accepted);
  assert(decision.reason == rm_nav::mapping::LoopCorrectionDecisionReason::kAccepted);
  assert(decision.consecutive_failures_after == 0);

  match.translation_correction_m = 0.4F;
  assert(gate.Evaluate(config, candidate, match, 0, &decision).ok());
  assert(!decision.accepted);
  assert(decision.reason ==
         rm_nav::mapping::LoopCorrectionDecisionReason::kTranslationTooLarge);
  assert(decision.consecutive_failures_after == 1);

  match.translation_correction_m = 0.08F;
  match.score = 0.3F;
  assert(gate.Evaluate(config, candidate, match, 0, &decision).ok());
  assert(!decision.accepted);
  assert(decision.reason == rm_nav::mapping::LoopCorrectionDecisionReason::kScoreTooLow);

  match.score = 0.92F;
  assert(gate.Evaluate(config, candidate, match, 3, &decision).ok());
  assert(!decision.accepted);
  assert(decision.auto_disabled);
  assert(decision.reason ==
         rm_nav::mapping::LoopCorrectionDecisionReason::kTooManyRecentFailures);
  assert(decision.consecutive_failures_after == 4);

  candidate.found = false;
  assert(gate.Evaluate(config, candidate, match, 2, &decision).ok());
  assert(!decision.accepted);
  assert(decision.reason == rm_nav::mapping::LoopCorrectionDecisionReason::kNoCandidate);
  assert(decision.consecutive_failures_after == 2);

  std::cout << "test_loop_correction_gate passed\n";
  return 0;
}
