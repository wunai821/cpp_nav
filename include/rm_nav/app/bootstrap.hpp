#pragma once

#include <string_view>
#include <vector>

#include "rm_nav/app/thread_model.hpp"

namespace rm_nav::app {

struct PipelineStage {
  std::string_view name{};
  ThreadDomain thread{};
};

struct RuntimeManifest {
  std::vector<std::string_view> frames{};
  std::vector<PipelineStage> main_chain{};
  std::vector<PipelineStage> debug_chain{};
  std::vector<ThreadBinding> threads{};
};

RuntimeManifest BuildPhase0Manifest();

}  // namespace rm_nav::app
