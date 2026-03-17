#include <cassert>
#include <cstdint>
#include <iostream>
#include <thread>

#include "rm_nav/common/ring_queue.hpp"

namespace {

struct alignas(64) AlignedValue {
  std::uint64_t lanes[8]{};
  int value{0};
};

void TestEmptyAndFull() {
  rm_nav::common::SpscRingQueue<int, 3> queue;
  int value = 0;
  assert(!queue.try_pop(&value));
  assert(queue.try_push(1));
  assert(queue.try_push(2));
  assert(queue.try_push(3));
  assert(!queue.try_push(4));
  assert(queue.dropped_pushes() == 1U);
  assert(queue.try_pop(&value) && value == 1);
  assert(queue.try_pop(&value) && value == 2);
  assert(queue.try_pop(&value) && value == 3);
  assert(!queue.try_pop(&value));
}

void TestSpscPressure() {
  constexpr int kIterations = 100000;
  rm_nav::common::SpscRingQueue<int, 256> queue;
  std::thread producer([&]() {
    for (int expected = 0; expected < kIterations; ++expected) {
      while (!queue.try_push(expected)) {
      }
    }
  });

  std::thread consumer([&]() {
    int expected = 0;
    while (expected < kIterations) {
      int value = 0;
      if (queue.try_pop(&value)) {
        assert(value == expected);
        ++expected;
      }
    }
  });

  producer.join();
  consumer.join();
}

void TestAlignment() {
  static_assert(alignof(AlignedValue) >= 64, "alignment contract must hold");
  rm_nav::common::SpscRingQueue<AlignedValue, 2> queue;
  AlignedValue input;
  input.value = 42;
  assert(queue.try_push(input));
  AlignedValue output;
  assert(queue.try_pop(&output));
  assert(output.value == 42);
}

}  // namespace

int main() {
  TestEmptyAndFull();
  TestSpscPressure();
  TestAlignment();
  std::cout << "test_ring_queue passed\n";
  return 0;
}
