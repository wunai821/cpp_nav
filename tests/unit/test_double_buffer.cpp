#include <cassert>
#include <atomic>
#include <iostream>
#include <thread>

#include "rm_nav/common/double_buffer.hpp"

namespace {

struct Snapshot {
  int seq{0};
  int checksum{0};
};

void TestPublishAndRead() {
  rm_nav::common::DoubleBuffer<int> buffer(1);
  assert(buffer.ReadSnapshot() == 1);
  buffer.Publish(5);
  assert(buffer.ReadSnapshot() == 5);
}

void TestConcurrentReadWrite() {
  rm_nav::common::DoubleBuffer<Snapshot> buffer(Snapshot{0, 0});
  std::atomic<bool> stop{false};
  std::thread writer([&]() {
    for (int seq = 1; seq <= 5000; ++seq) {
      buffer.Publish(Snapshot{seq, seq * 7});
    }
    stop.store(true, std::memory_order_release);
  });

  while (!stop.load(std::memory_order_acquire)) {
    const Snapshot snapshot = buffer.ReadSnapshot();
    assert(snapshot.checksum == snapshot.seq * 7);
  }

  writer.join();
  const Snapshot final_snapshot = buffer.ReadSnapshot();
  assert(final_snapshot.checksum == final_snapshot.seq * 7);
  assert(buffer.generation() > 0U);
}

}  // namespace

int main() {
  TestPublishAndRead();
  TestConcurrentReadWrite();
  std::cout << "test_double_buffer passed\n";
  return 0;
}
