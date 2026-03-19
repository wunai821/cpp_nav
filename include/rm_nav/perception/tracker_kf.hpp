#pragma once

#include <array>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/common/types.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"

namespace rm_nav::perception {

struct MotConfig;
struct Cluster;

class TrackerKf {
 public:
  common::Status Configure(const MotConfig& config);
  void Reset();
  common::Status Update(const std::vector<Cluster>& clusters, common::TimePoint stamp,
                        std::vector<data::DynamicObstacle>* obstacles);

 private:
  struct Track {
    common::ObjectId id{0};
    std::array<float, 4> state{{0.0F, 0.0F, 0.0F, 0.0F}};
    std::array<float, 16> covariance{{0.0F}};
    float radius_m{0.25F};
    float confidence{0.0F};
    std::uint32_t age{0};
    std::uint32_t missed_frames{0};
    common::TimePoint stamp{};
    bool confirmed{false};
  };

  void PredictTrack(common::TimePoint stamp, Track* track) const;
  void UpdateTrack(const Cluster& cluster, common::TimePoint stamp, Track* track) const;
  data::DynamicObstacle ToDynamicObstacle(const Track& track) const;

  float association_distance_m_{1.0F};
  std::uint32_t max_missed_frames_{4U};
  float process_noise_{0.08F};
  float measurement_noise_{0.15F};
  float initial_confidence_{0.45F};
  float confirmation_confidence_{0.6F};
  std::vector<Track> tracks_{};
  common::ObjectId next_track_id_{1};
  bool configured_{false};
};

}  // namespace rm_nav::perception
