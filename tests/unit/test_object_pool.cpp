#include <cassert>
#include <iostream>

#include "rm_nav/common/object_pool.hpp"
#include "rm_nav/data/debug_sample.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/point_block.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/data/trajectory.hpp"

namespace {

void TestAcquireRelease() {
  rm_nav::common::ObjectPool<rm_nav::data::LidarFrame, 2> pool;
  auto first = pool.Acquire();
  auto second = pool.Acquire();
  assert(first);
  assert(second);
  assert(pool.in_use() == 2U);
  assert(!pool.Acquire());
  first.reset();
  assert(pool.in_use() == 1U);
  auto third = pool.Acquire();
  assert(third);
  assert(pool.high_watermark() == 2U);
}

void TestTypeCoverage() {
  rm_nav::common::ObjectPool<rm_nav::data::PointBlock, 4> point_pool;
  rm_nav::common::ObjectPool<rm_nav::data::SyncedFrame, 4> sync_pool;
  rm_nav::common::ObjectPool<rm_nav::data::Trajectory, 4> traj_pool;
  rm_nav::common::ObjectPool<rm_nav::data::DebugSample, 4> debug_pool;

  auto point = point_pool.Acquire();
  auto synced = sync_pool.Acquire();
  auto traj = traj_pool.Acquire();
  auto debug = debug_pool.Acquire();
  assert(point && synced && traj && debug);
  assert(point_pool.occupancy_rate() > 0.0);
}

void TestPressure() {
  rm_nav::common::ObjectPool<int, 8> pool;
  for (int index = 0; index < 10000; ++index) {
    auto handle = pool.Acquire();
    assert(handle);
    *handle = index;
    assert(*handle == index);
  }
  assert(pool.in_use() == 0U);
}

}  // namespace

int main() {
  TestAcquireRelease();
  TestTypeCoverage();
  TestPressure();
  std::cout << "test_object_pool passed\n";
  return 0;
}
