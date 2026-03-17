#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>

#include "rm_nav/data/pose.hpp"

namespace rm_nav::tf {

class TimedPoseBuffer {
 public:
  static constexpr std::size_t kCapacity = 128;

  struct Slot {
    std::atomic<std::uint64_t> version{0};
    data::Pose3f pose{};
  };

  void Push(const data::Pose3f& pose);
  std::optional<data::Pose3f> LookupNearest(common::TimePoint stamp) const;
  std::optional<data::Pose3f> LookupLatest() const;

  std::array<Slot, kCapacity> slots_{};
  std::atomic<std::uint64_t> write_count_{0};
};

}  // namespace rm_nav::tf
