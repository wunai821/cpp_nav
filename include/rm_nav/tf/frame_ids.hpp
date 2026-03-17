#pragma once

#include <array>
#include <string_view>

namespace rm_nav::tf {

inline constexpr std::string_view kMapFrame = "map";
inline constexpr std::string_view kOdomFrame = "odom";
inline constexpr std::string_view kBaseLinkFrame = "base_link";
inline constexpr std::string_view kLaserFrame = "laser_link";
inline constexpr std::string_view kImuFrame = "imu_link";

inline constexpr std::array<std::string_view, 5> kRequiredFrames = {
    kMapFrame,
    kOdomFrame,
    kBaseLinkFrame,
    kLaserFrame,
    kImuFrame,
};

}  // namespace rm_nav::tf
