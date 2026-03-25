#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/app/odom_feedback_yaw_adapter.hpp"
#include "rm_nav/common/time.hpp"

namespace {

bool Near(float value, float expected, float tolerance = 1.0e-4F) {
  return std::fabs(value - expected) <= tolerance;
}

}  // namespace

int main() {
  rm_nav::app::OdomFeedbackYawAdapter adapter;
  auto status = adapter.Configure("base_link");
  assert(status.ok());

  rm_nav::data::OdomState odom;
  odom.stamp = rm_nav::common::FromNanoseconds(1'000'000'000LL);
  odom.yaw_rad = 0.7F;
  odom.wz_radps = 2.0F;
  auto adapted = adapter.Adapt(odom, 0.0F);
  assert(Near(adapted.yaw_rad, 0.7F));

  status = adapter.Configure("integrate_wz");
  assert(status.ok());

  odom.stamp = rm_nav::common::FromNanoseconds(2'000'000'000LL);
  odom.yaw_rad = 1.5F;
  odom.wz_radps = 0.5F;
  adapted = adapter.Adapt(odom, 0.2F);
  assert(Near(adapted.yaw_rad, 0.2F));

  odom.stamp = rm_nav::common::FromNanoseconds(2'100'000'000LL);
  odom.yaw_rad = 1.6F;
  odom.wz_radps = 0.5F;
  adapted = adapter.Adapt(odom, -0.4F);
  assert(Near(adapted.yaw_rad, 0.25F));

  odom.stamp = rm_nav::common::FromNanoseconds(2'400'000'000LL);
  odom.yaw_rad = -2.0F;
  odom.wz_radps = 0.5F;
  adapted = adapter.Adapt(odom, -1.0F);
  assert(Near(adapted.yaw_rad, 0.25F));

  status = adapter.Configure("invalid_mode");
  assert(!status.ok());

  std::cout << "test_odom_feedback_yaw_adapter passed\n";
  return 0;
}
