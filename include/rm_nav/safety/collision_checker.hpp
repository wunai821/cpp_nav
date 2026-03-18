#pragma once

#include <array>
#include <utility>
#include <vector>

#include "rm_nav/config/safety_config.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::safety {

enum class CollisionType {
  kNone = 0,
  kStatic,
  kDynamic,
};

struct CollisionCheckResult {
  CollisionType type{CollisionType::kNone};
  bool obstacle_too_close{false};
};

class CollisionChecker {
 public:
  CollisionCheckResult Evaluate(const data::Pose3f& pose, const data::GridMap2D& costmap,
                                const std::vector<data::DynamicObstacle>& obstacles,
                                const data::ChassisCmd& cmd,
                                const config::SafetyConfig& config) const;

 private:
  using FootprintSamples = std::array<std::pair<float, float>, 5>;

  FootprintSamples BuildFootprintSamples(float local_x, float local_y, float local_yaw,
                                         const config::SafetyConfig& config) const;
  bool CostmapCollisionAtLocal(const data::GridMap2D& costmap, float local_x,
                               float local_y) const;
  bool DynamicCollisionAhead(const data::Pose3f& pose, const data::ChassisCmd& cmd,
                             const std::vector<data::DynamicObstacle>& obstacles,
                             const config::SafetyConfig& config) const;
  bool ObstacleTooClose(const data::Pose3f& pose,
                        const std::vector<data::DynamicObstacle>& obstacles,
                        float threshold_m,
                        const config::SafetyConfig& config) const;
};

}  // namespace rm_nav::safety
