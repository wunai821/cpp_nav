#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <new>
#include <type_traits>
#include <utility>

#include "rm_nav/common/aligned_allocator.hpp"

namespace rm_nav::common {

template <typename T, std::size_t Capacity>
class SpscRingQueue {
  static_assert(Capacity >= 1, "SpscRingQueue capacity must be at least 1");

 public:
  SpscRingQueue() = default;
  ~SpscRingQueue() { Clear(); }

  SpscRingQueue(const SpscRingQueue&) = delete;
  SpscRingQueue& operator=(const SpscRingQueue&) = delete;

  static constexpr std::size_t capacity() { return Capacity; }

  template <typename... Args>
  bool try_emplace(Args&&... args) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t next = Increment(tail);
    if (next == head_.load(std::memory_order_acquire)) {
      dropped_pushes_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }

    new (&storage_[tail]) T(std::forward<Args>(args)...);
    tail_.store(next, std::memory_order_release);
    return true;
  }

  bool try_push(const T& value) { return try_emplace(value); }
  bool try_push(T&& value) { return try_emplace(std::move(value)); }

  bool try_pop(T* value) {
    if (value == nullptr) {
      return false;
    }

    const std::size_t head = head_.load(std::memory_order_relaxed);
    if (head == tail_.load(std::memory_order_acquire)) {
      return false;
    }

    T* slot = Slot(head);
    *value = std::move(*slot);
    slot->~T();
    head_.store(Increment(head), std::memory_order_release);
    return true;
  }

  bool empty() const {
    return head_.load(std::memory_order_acquire) ==
           tail_.load(std::memory_order_acquire);
  }

  bool full() const { return size_approx() == capacity(); }

  std::size_t size_approx() const {
    const std::size_t head = head_.load(std::memory_order_acquire);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    return tail >= head ? tail - head : kStorageSize - head + tail;
  }

  std::uint64_t dropped_pushes() const {
    return dropped_pushes_.load(std::memory_order_relaxed);
  }

  void Clear() {
    while (!empty()) {
      const std::size_t head = head_.load(std::memory_order_relaxed);
      Slot(head)->~T();
      head_.store(Increment(head), std::memory_order_relaxed);
    }
  }

 private:
  using Storage = typename std::aligned_storage<sizeof(T), alignof(T)>::type;
  static constexpr std::size_t kStorageSize = Capacity + 1U;

  static constexpr std::size_t Increment(std::size_t index) {
    return (index + 1U) % kStorageSize;
  }

  T* Slot(std::size_t index) {
    return std::launder(reinterpret_cast<T*>(&storage_[index]));
  }

  alignas(kCacheLineSize) std::array<Storage, kStorageSize> storage_{};
  alignas(kCacheLineSize) std::atomic<std::size_t> head_{0};
  alignas(kCacheLineSize) std::atomic<std::size_t> tail_{0};
  alignas(kCacheLineSize) std::atomic<std::uint64_t> dropped_pushes_{0};
};

}  // namespace rm_nav::common
