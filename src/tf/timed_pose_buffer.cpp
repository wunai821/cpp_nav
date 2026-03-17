#include "rm_nav/tf/timed_pose_buffer.hpp"

#include <limits>

#include "rm_nav/common/time.hpp"

namespace rm_nav::tf {
namespace {

bool ReadStable(const TimedPoseBuffer::Slot& slot, data::Pose3f* pose) {
  const std::uint64_t version_a = slot.version.load(std::memory_order_acquire);
  if ((version_a & 1U) != 0U) {
    return false;
  }
  const data::Pose3f copy = slot.pose;
  const std::uint64_t version_b = slot.version.load(std::memory_order_acquire);
  if (version_a != version_b || (version_b & 1U) != 0U) {
    return false;
  }
  *pose = copy;
  return true;
}

std::uint64_t DistanceNs(common::TimePoint a, common::TimePoint b) {
  const auto delta = common::ToNanoseconds(a) - common::ToNanoseconds(b);
  return static_cast<std::uint64_t>(delta >= 0 ? delta : -delta);
}

}  // namespace

void TimedPoseBuffer::Push(const data::Pose3f& pose) {
  const std::uint64_t ticket = write_count_.fetch_add(1, std::memory_order_acq_rel);
  Slot& slot = slots_[ticket % kCapacity];
  const std::uint64_t final_version = (ticket + 1U) * 2U;
  slot.version.store(final_version - 1U, std::memory_order_release);
  slot.pose = pose;
  slot.version.store(final_version, std::memory_order_release);
}

std::optional<data::Pose3f> TimedPoseBuffer::LookupNearest(common::TimePoint stamp) const {
  const std::uint64_t written = write_count_.load(std::memory_order_acquire);
  if (written == 0U) {
    return std::nullopt;
  }

  const std::uint64_t limit = written < kCapacity ? written : kCapacity;
  std::uint64_t best_distance = std::numeric_limits<std::uint64_t>::max();
  std::optional<data::Pose3f> best_pose;

  for (std::uint64_t offset = 0; offset < limit; ++offset) {
    const std::uint64_t ticket = written - 1U - offset;
    data::Pose3f candidate;
    if (!ReadStable(slots_[ticket % kCapacity], &candidate) || !candidate.is_valid) {
      continue;
    }
    const auto distance = DistanceNs(candidate.stamp, stamp);
    if (distance < best_distance) {
      best_distance = distance;
      best_pose = candidate;
    }
  }

  return best_pose;
}

std::optional<data::Pose3f> TimedPoseBuffer::LookupLatest() const {
  const std::uint64_t written = write_count_.load(std::memory_order_acquire);
  if (written == 0U) {
    return std::nullopt;
  }

  for (std::uint64_t offset = 0; offset < kCapacity && offset < written; ++offset) {
    data::Pose3f candidate;
    if (ReadStable(slots_[(written - 1U - offset) % kCapacity], &candidate) &&
        candidate.is_valid) {
      return candidate;
    }
  }
  return std::nullopt;
}

}  // namespace rm_nav::tf
