#include "rm_nav/app/odom_feedback_yaw_adapter.hpp"

#include <cmath>

namespace rm_nav::app {

common::Status OdomFeedbackYawAdapter::Configure(const std::string& mode) {
  Reset();
  if (mode == "base_link") {
    mode_ = Mode::kBaseLink;
    return common::Status::Ok();
  }
  if (mode == "integrate_wz") {
    mode_ = Mode::kIntegrateWz;
    return common::Status::Ok();
  }
  return common::Status::InvalidArgument("unsupported stm32.feedback_yaw_mode");
}

void OdomFeedbackYawAdapter::Reset() {
  initialized_ = false;
  last_stamp_ = {};
  integrated_yaw_rad_ = 0.0F;
}

data::OdomState OdomFeedbackYawAdapter::Adapt(const data::OdomState& odom,
                                              float seed_yaw_rad) {
  if (mode_ == Mode::kBaseLink) {
    return odom;
  }

  data::OdomState adapted = odom;
  if (!initialized_) {
    integrated_yaw_rad_ = NormalizeAngle(seed_yaw_rad);
    last_stamp_ = odom.stamp;
    initialized_ = true;
    adapted.yaw_rad = integrated_yaw_rad_;
    return adapted;
  }

  if (odom.stamp > last_stamp_) {
    const auto dt = std::chrono::duration_cast<std::chrono::duration<float>>(odom.stamp - last_stamp_)
                        .count();
    if (dt > 0.0F && dt < 0.2F) {
      integrated_yaw_rad_ =
          NormalizeAngle(integrated_yaw_rad_ + odom.wz_radps * dt);
    }
    last_stamp_ = odom.stamp;
  }

  adapted.yaw_rad = integrated_yaw_rad_;
  return adapted;
}

float OdomFeedbackYawAdapter::NormalizeAngle(float angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

}  // namespace rm_nav::app
