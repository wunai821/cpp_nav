#include <cassert>
#include <cmath>
#include <iostream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/tf_types.hpp"
#include "rm_nav/tf/frame_ids.hpp"
#include "rm_nav/tf/tf_tree_lite.hpp"
#include "rm_nav/tf/transform_query.hpp"

namespace {

bool AlmostEqual(float lhs, float rhs, float epsilon = 1e-4F) {
  return std::fabs(lhs - rhs) <= epsilon;
}

}  // namespace

int main() {
  rm_nav::tf::TfTreeLite tree;
  const auto stamp = rm_nav::common::FromNanoseconds(1000000);

  auto base_to_laser =
      rm_nav::tf::MakeTransform(rm_nav::tf::kBaseLinkFrame, rm_nav::tf::kLaserFrame,
                                0.20F, 0.01F, 0.12F, 0.02F, -0.03F, 0.05F, stamp);
  auto base_to_imu =
      rm_nav::tf::MakeTransform(rm_nav::tf::kBaseLinkFrame, rm_nav::tf::kImuFrame,
                                -0.10F, -0.02F, 0.03F, 0.00F, 0.01F, -0.02F, stamp);
  assert(tree.RegisterStaticTransform(base_to_laser).ok());
  assert(tree.RegisterStaticTransform(base_to_imu).ok());

  auto map_to_odom =
      rm_nav::tf::MakeTransform(rm_nav::tf::kMapFrame, rm_nav::tf::kOdomFrame,
                                1.0F, 2.0F, 0.05F, 0.01F, -0.02F, 0.3F, stamp);
  auto odom_to_base =
      rm_nav::tf::MakeTransform(rm_nav::tf::kOdomFrame, rm_nav::tf::kBaseLinkFrame,
                                0.5F, 0.0F, 0.02F, -0.01F, 0.02F, 0.2F, stamp);
  assert(tree.PushMapToOdom(map_to_odom).ok());
  assert(tree.PushOdomToBase(odom_to_base).ok());

  rm_nav::data::Pose3f queried;
  assert(rm_nav::tf::LookupTransform(tree, rm_nav::tf::kMapFrame,
                                     rm_nav::tf::kBaseLinkFrame, stamp, &queried)
             .ok());

  const auto expected =
      rm_nav::tf::Compose(map_to_odom, odom_to_base);
  assert(AlmostEqual(queried.position.x, expected.position.x));
  assert(AlmostEqual(queried.position.y, expected.position.y));
  assert(AlmostEqual(queried.position.z, expected.position.z));
  assert(AlmostEqual(queried.rpy.x, expected.rpy.x));
  assert(AlmostEqual(queried.rpy.y, expected.rpy.y));
  assert(AlmostEqual(queried.rpy.z, expected.rpy.z));

  assert(rm_nav::tf::LookupTransform(tree, rm_nav::tf::kMapFrame,
                                     rm_nav::tf::kLaserFrame, stamp, &queried)
             .ok());
  const auto expected_laser =
      rm_nav::tf::Compose(expected, base_to_laser);
  assert(AlmostEqual(queried.position.x, expected_laser.position.x));
  assert(AlmostEqual(queried.position.y, expected_laser.position.y));
  assert(AlmostEqual(queried.position.z, expected_laser.position.z));
  assert(AlmostEqual(queried.rpy.x, expected_laser.rpy.x));
  assert(AlmostEqual(queried.rpy.y, expected_laser.rpy.y));
  assert(AlmostEqual(queried.rpy.z, expected_laser.rpy.z));

  assert(rm_nav::tf::LookupTransform(tree, rm_nav::tf::kBaseLinkFrame,
                                     rm_nav::tf::kOdomFrame, stamp, &queried)
             .ok());
  const auto expected_inverse = rm_nav::tf::Inverse(odom_to_base);
  assert(AlmostEqual(queried.position.x, expected_inverse.position.x));
  assert(AlmostEqual(queried.position.y, expected_inverse.position.y));
  assert(AlmostEqual(queried.position.z, expected_inverse.position.z));
  assert(AlmostEqual(queried.rpy.x, expected_inverse.rpy.x));
  assert(AlmostEqual(queried.rpy.y, expected_inverse.rpy.y));
  assert(AlmostEqual(queried.rpy.z, expected_inverse.rpy.z));

  std::cout << "test_tf_tree_lite passed\n";
  return 0;
}
