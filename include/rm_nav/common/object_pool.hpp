#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <utility>

#include "rm_nav/common/aligned_allocator.hpp"

namespace rm_nav::common {

template <typename T, std::size_t Capacity>
class ObjectPool {
  static_assert(Capacity > 0, "ObjectPool capacity must be positive");

 public:
  class Handle {
   public:
    Handle() = default;
    Handle(ObjectPool* pool, std::size_t index) : pool_(pool), index_(index) {}
    ~Handle() { reset(); }

    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;

    Handle(Handle&& other) noexcept { *this = std::move(other); }
    Handle& operator=(Handle&& other) noexcept {
      if (this != &other) {
        reset();
        pool_ = other.pool_;
        index_ = other.index_;
        other.pool_ = nullptr;
      }
      return *this;
    }

    T* get() { return pool_ == nullptr ? nullptr : &pool_->storage_[index_]; }
    const T* get() const {
      return pool_ == nullptr ? nullptr : &pool_->storage_[index_];
    }

    T& operator*() { return *get(); }
    T* operator->() { return get(); }
    const T& operator*() const { return *get(); }
    const T* operator->() const { return get(); }

    explicit operator bool() const { return pool_ != nullptr; }

    void reset() {
      if (pool_ != nullptr) {
        pool_->Release(index_);
        pool_ = nullptr;
      }
    }

   private:
    ObjectPool* pool_{nullptr};
    std::size_t index_{0};
  };

  ObjectPool() {
    for (std::size_t index = 0; index < Capacity; ++index) {
      next_[index].store(index + 1U < Capacity ? static_cast<int>(index + 1U) : -1,
                         std::memory_order_relaxed);
    }
  }

  Handle Acquire() {
    int head = free_head_.load(std::memory_order_acquire);
    while (head >= 0) {
      const int next = next_[static_cast<std::size_t>(head)].load(std::memory_order_relaxed);
      if (free_head_.compare_exchange_weak(head, next, std::memory_order_acq_rel,
                                           std::memory_order_acquire)) {
        const std::size_t in_use = in_use_.fetch_add(1, std::memory_order_relaxed) + 1U;
        std::size_t high = high_watermark_.load(std::memory_order_relaxed);
        while (in_use > high &&
               !high_watermark_.compare_exchange_weak(high, in_use,
                                                      std::memory_order_relaxed)) {
        }
        return Handle(this, static_cast<std::size_t>(head));
      }
    }
    return {};
  }

  constexpr std::size_t capacity() const { return Capacity; }
  std::size_t in_use() const { return in_use_.load(std::memory_order_relaxed); }
  std::size_t high_watermark() const {
    return high_watermark_.load(std::memory_order_relaxed);
  }
  double occupancy_rate() const {
    return static_cast<double>(in_use()) / static_cast<double>(Capacity);
  }

 private:
  void Release(std::size_t index) {
    int head = free_head_.load(std::memory_order_acquire);
    do {
      next_[index].store(head, std::memory_order_relaxed);
    } while (!free_head_.compare_exchange_weak(head, static_cast<int>(index),
                                               std::memory_order_acq_rel,
                                               std::memory_order_acquire));
    in_use_.fetch_sub(1, std::memory_order_relaxed);
  }

  alignas(kCacheLineSize) std::array<T, Capacity> storage_{};
  alignas(kCacheLineSize) std::array<std::atomic<int>, Capacity> next_{};
  alignas(kCacheLineSize) std::atomic<int> free_head_{0};
  alignas(kCacheLineSize) std::atomic<std::size_t> in_use_{0};
  alignas(kCacheLineSize) std::atomic<std::size_t> high_watermark_{0};
};

}  // namespace rm_nav::common
