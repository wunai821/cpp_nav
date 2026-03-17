#pragma once

#include "rm_nav/debug/mirror_tap.hpp"

namespace rm_nav::debug {

class DebugPublisher {
 public:
  virtual ~DebugPublisher() = default;

  virtual void Publish(const DebugEnvelope& envelope) = 0;
};

}  // namespace rm_nav::debug
