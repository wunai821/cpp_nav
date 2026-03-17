#pragma once

#include <array>
#include <optional>
#include <string_view>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::tf {

class StaticTfRegistry {
 public:
  common::Status Register(const data::Pose3f& transform);
  std::optional<data::Pose3f> Lookup(std::string_view parent,
                                     std::string_view child) const;

 private:
  struct Entry {
    data::Pose3f transform{};
    bool occupied{false};
  };

  std::array<Entry, 8> entries_{};
};

}  // namespace rm_nav::tf
