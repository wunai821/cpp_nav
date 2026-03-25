#pragma once

#include <optional>
#include <string>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/odom_state.hpp"

namespace rm_nav::app {

class OdomFeedbackYawAdapter {
 public:
  common::Status Configure(const std::string& mode);
  void Reset();

  data::OdomState Adapt(const data::OdomState& odom, float seed_yaw_rad);

 private:
  enum class Mode {
    kBaseLink = 0,
    kIntegrateWz,
  };

  static float NormalizeAngle(float angle_rad);

  Mode mode_{Mode::kBaseLink};
  bool initialized_{false};
  common::TimePoint last_stamp_{};
  float integrated_yaw_rad_{0.0F};
};

}  // namespace rm_nav::app
