#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>

#include "rm_nav/common/aligned_allocator.hpp"

namespace rm_nav::common {

template <typename T>
class DoubleBuffer {
 public:
  DoubleBuffer() = default;
  explicit DoubleBuffer(const T& initial_value) {
    buffers_[0] = initial_value;
    buffers_[1] = initial_value;
  }

  void Publish(const T& value) {
    const int inactive = 1 - active_index_.load(std::memory_order_relaxed);
    buffers_[inactive] = value;
    active_index_.store(inactive, std::memory_order_release);
    generation_.fetch_add(1, std::memory_order_relaxed);
  }

  void Publish(T&& value) {
    const int inactive = 1 - active_index_.load(std::memory_order_relaxed);
    buffers_[inactive] = std::move(value);
    active_index_.store(inactive, std::memory_order_release);
    generation_.fetch_add(1, std::memory_order_relaxed);
  }

  template <typename F>
  void ModifyAndPublish(F&& updater) {
    const int active = active_index_.load(std::memory_order_relaxed);
    const int inactive = 1 - active;
    buffers_[inactive] = buffers_[active];
    updater(&buffers_[inactive]);
    active_index_.store(inactive, std::memory_order_release);
    generation_.fetch_add(1, std::memory_order_relaxed);
  }

  T ReadSnapshot() const {
    const int active = active_index_.load(std::memory_order_acquire);
    return buffers_[active];
  }

  std::uint64_t generation() const {
    return generation_.load(std::memory_order_relaxed);
  }

 private:
  alignas(kCacheLineSize) std::array<T, 2> buffers_{};
  alignas(kCacheLineSize) std::atomic<int> active_index_{0};
  alignas(kCacheLineSize) std::atomic<std::uint64_t> generation_{0};
};

}  // namespace rm_nav::common
