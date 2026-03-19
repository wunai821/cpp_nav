#pragma once

#include <string>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/comm_config.hpp"
#include "rm_nav/config/costmap_config.hpp"
#include "rm_nav/config/debug_config.hpp"
#include "rm_nav/config/frames_config.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/config/mot_config.hpp"
#include "rm_nav/config/planner_config.hpp"
#include "rm_nav/config/safety_config.hpp"
#include "rm_nav/config/sensor_config.hpp"
#include "rm_nav/config/spawn_config.hpp"
#include "rm_nav/config/system_config.hpp"

namespace rm_nav::config {

struct LoadedConfig {
  std::string config_dir{};
  std::vector<std::string> loaded_files{};
  SystemConfig system{};
  FramesConfig frames{};
  SensorConfig sensors{};
  CommConfig comm{};
  DebugConfig debug{};
  LocalizationConfig localization{};
  MappingConfig mapping{};
  CostmapConfig costmap{};
  MotConfig mot{};
  PlannerConfig planner{};
  SafetyConfig safety{};
  SpawnConfig spawn{};
};

class ConfigLoader {
 public:
  common::Status LoadFromDirectory(const std::string& config_dir,
                                   LoadedConfig* loaded_config) const;
  std::string BuildSummary(const LoadedConfig& loaded_config) const;
};

}  // namespace rm_nav::config
