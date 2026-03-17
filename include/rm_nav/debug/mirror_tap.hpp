#pragma once

#include <string_view>

#include "rm_nav/common/time.hpp"

namespace rm_nav::debug {

struct DebugEnvelope {
  common::TimePoint stamp{};
  std::string_view source_stage{};
  std::string_view schema{};
  std::string_view summary{};
};

class MirrorTap {
 public:
  virtual ~MirrorTap() = default;

  virtual void Mirror(const DebugEnvelope& envelope) = 0;
};

}  // namespace rm_nav::debug
