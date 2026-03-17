#include "rm_nav/perception/local_costmap_builder.hpp"

#include <algorithm>

namespace rm_nav::perception {

common::Status LocalCostmapBuilder::Build(const data::LidarFrame& filtered_frame,
                                          const data::Pose3f& pose,
                                          data::GridMap2D* costmap) {
  if (costmap == nullptr) {
    return common::Status::InvalidArgument("costmap output is null");
  }

  constexpr std::uint32_t kWidth = 80;
  constexpr std::uint32_t kHeight = 80;
  constexpr float kResolution = 0.10F;

  costmap->stamp = filtered_frame.stamp;
  costmap->resolution_m = kResolution;
  costmap->width = kWidth;
  costmap->height = kHeight;
  costmap->origin = pose;
  costmap->occupancy.assign(kWidth * kHeight, 0U);

  const int center_x = static_cast<int>(kWidth / 2U);
  const int center_y = static_cast<int>(kHeight / 2U);
  for (const auto& point : filtered_frame.points) {
    const int gx = center_x + static_cast<int>(point.x / kResolution);
    const int gy = center_y + static_cast<int>(point.y / kResolution);
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(kWidth) ||
        gy >= static_cast<int>(kHeight)) {
      continue;
    }
    costmap->occupancy[static_cast<std::size_t>(gy) * kWidth +
                       static_cast<std::size_t>(gx)] = 100U;
  }
  return common::Status::Ok();
}

common::Status LocalCostmapBuilder::BuildAndPublish(
    const data::LidarFrame& filtered_frame, const data::Pose3f& pose) {
  data::LocalCostmap costmap;
  const auto status = Build(filtered_frame, pose, &costmap);
  if (!status.ok()) {
    return status;
  }
  latest_costmap_.Publish(std::move(costmap));
  return common::Status::Ok();
}

}  // namespace rm_nav::perception
