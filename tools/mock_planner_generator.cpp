#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rm_nav/common/ring_queue.hpp"
#include "rm_nav/common/time.hpp"
#include "rm_nav/config/config_loader.hpp"
#include "rm_nav/debug/foxglove_server.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/planning/planner_coordinator.hpp"
#include "rm_nav/tf/frame_ids.hpp"
#include "rm_nav/utils/logger.hpp"

namespace {

constexpr std::size_t kQueueCapacity = 64U;
constexpr std::size_t kMaxObstacles = 8U;

struct Options {
  std::string config_dir{"config"};
  std::string output_dir{"logs/mock_planner"};
  std::string scenario{"mixed"};
  std::string scenario_file{};
  int duration_s{8};
  double hz{20.0};
  double switch_goal_period_s{0.0};
  bool integrate_pose{true};
  bool force_fixed_pose{false};
  bool foxglove_enabled{true};
  std::string foxglove_host{"127.0.0.1"};
  int foxglove_port{-1};
};

struct WallSegment {
  int x0{0};
  int y0{0};
  int x1{0};
  int y1{0};
};

enum class MotionModel {
  kLinear = 0,
  kChase,
};

struct ScenarioObstacle {
  rm_nav::common::ObjectId id{0};
  MotionModel motion{MotionModel::kLinear};
  float x_m{0.0F};
  float y_m{0.0F};
  float vx_mps{0.0F};
  float vy_mps{0.0F};
  float radius_m{0.3F};
  float offset_x_m{0.0F};
  float offset_y_m{0.0F};
};

struct ScenarioConfig {
  float start_x_m{0.0F};
  float start_y_m{0.0F};
  float start_yaw_rad{0.0F};
  float primary_goal_x_m{0.0F};
  float primary_goal_y_m{0.0F};
  float primary_goal_yaw_rad{0.0F};
  float secondary_goal_x_m{0.0F};
  float secondary_goal_y_m{0.0F};
  float secondary_goal_yaw_rad{0.0F};
  float switch_goal_period_s{0.0F};
  bool integrate_pose{true};
  std::uint32_t map_width_cells{100U};
  std::uint32_t map_height_cells{70U};
  float map_resolution_m{0.1F};
  std::vector<WallSegment> walls{};
  std::vector<ScenarioObstacle> obstacles{};
};

struct MockPlannerFrame {
  std::uint32_t sequence{0};
  rm_nav::common::TimePoint stamp{};
  rm_nav::data::Pose3f current_pose{};
  rm_nav::data::Pose3f goal_pose{};
  std::array<rm_nav::data::DynamicObstacle, kMaxObstacles> obstacles{};
  std::size_t obstacle_count{0};
};

struct PlannerCmdMirror {
  std::atomic<float> vx_mps{0.0F};
  std::atomic<float> vy_mps{0.0F};
  std::atomic<float> wz_radps{0.0F};
  std::atomic<bool> brake{true};
};

struct ProducerStats {
  std::atomic<std::uint32_t> produced{0};
  std::atomic<std::uint32_t> dropped{0};
};

struct ConsumerStats {
  std::atomic<std::uint32_t> consumed{0};
  std::atomic<std::uint32_t> planned{0};
};

using MockQueue = rm_nav::common::SpscRingQueue<MockPlannerFrame, kQueueCapacity>;

constexpr std::uint32_t kPoseChannelId = 1U;
constexpr std::uint32_t kGoalChannelId = 2U;
constexpr std::uint32_t kCmdChannelId = 3U;
constexpr std::uint32_t kPlannerStatusChannelId = 4U;
constexpr std::uint32_t kPathChannelId = 5U;
constexpr std::uint32_t kDynamicObstaclesChannelId = 6U;
constexpr std::uint32_t kTrajectoryChannelId = 7U;
constexpr std::uint32_t kStaticMapSceneChannelId = 8U;
constexpr std::uint32_t kLocalCostmapSceneChannelId = 9U;
constexpr std::uint32_t kDynamicMaxRiskChannelId = 10U;
constexpr std::uint32_t kDynamicIntegratedRiskChannelId = 11U;
constexpr std::uint32_t kDynamicClearanceMinChannelId = 12U;
constexpr std::uint32_t kDynamicRisk05ChannelId = 13U;
constexpr std::uint32_t kDynamicRisk10ChannelId = 14U;

constexpr char kPoseSchema[] =
    R"({"type":"object","properties":{"x":{"type":"number"},"y":{"type":"number"},"yaw":{"type":"number"}},"required":["x","y","yaw"]})";
constexpr char kGoalSchema[] =
    R"({"type":"object","properties":{"x":{"type":"number"},"y":{"type":"number"},"yaw":{"type":"number"}},"required":["x","y","yaw"]})";
constexpr char kCmdSchema[] =
    R"({"type":"object","properties":{"vx":{"type":"number"},"vy":{"type":"number"},"wz":{"type":"number"},"brake":{"type":"boolean"}},"required":["vx","vy","wz","brake"]})";
constexpr char kPlannerStatusSchema[] =
    R"({"type":"object","properties":{"mode":{"type":"string"},"distance_to_goal_m":{"type":"number"},"distance_to_center_m":{"type":"number"},"yaw_error_rad":{"type":"number"},"reached":{"type":"boolean"},"cmd":{"type":"object"},"dwa_score":{"type":"object","properties":{"goal":{"type":"number"},"path":{"type":"number"},"smooth":{"type":"number"},"heading":{"type":"number"},"clearance":{"type":"number"},"velocity":{"type":"number"},"dynamic":{"type":"number"},"dynamic_max_risk":{"type":"number"},"dynamic_integrated_risk":{"type":"number"},"dynamic_clearance_min":{"type":"number"},"dynamic_risk_05":{"type":"number"},"dynamic_risk_10":{"type":"number"},"total":{"type":"number"}}}},"required":["mode","distance_to_goal_m","distance_to_center_m","yaw_error_rad","reached","cmd","dwa_score"]})";
constexpr char kPathSchema[] =
    R"({"type":"object","properties":{"points":{"type":"array","items":{"type":"object","properties":{"x":{"type":"number"},"y":{"type":"number"},"heading":{"type":"number"},"speed":{"type":"number"}},"required":["x","y","heading","speed"]}}},"required":["points"]})";
constexpr char kDynamicObstaclesSchema[] =
    R"({"type":"object","properties":{"obstacles":{"type":"array","items":{"type":"object","properties":{"id":{"type":"integer"},"x":{"type":"number"},"y":{"type":"number"},"vx":{"type":"number"},"vy":{"type":"number"},"radius_m":{"type":"number"}},"required":["id","x","y","vx","vy","radius_m"]}}},"required":["obstacles"]})";
constexpr char kTrajectorySchema[] =
    R"({"type":"object","properties":{"poses":{"type":"array","items":{"type":"object","properties":{"x":{"type":"number"},"y":{"type":"number"},"yaw":{"type":"number"}},"required":["x","y","yaw"]}}},"required":["poses"]})";
constexpr char kScalarSchema[] =
    R"({"type":"object","properties":{"value":{"type":"number"}},"required":["value"]})";

struct RgbaColor {
  float r{1.0F};
  float g{1.0F};
  float b{1.0F};
  float a{1.0F};
};

std::string Trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1U);
}

std::string StripQuotes(const std::string& value) {
  if (value.size() >= 2U &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    return value.substr(1, value.size() - 2U);
  }
  return value;
}

bool ParseBool(const std::string& value, bool* parsed) {
  if (parsed == nullptr) {
    return false;
  }
  if (value == "true" || value == "True" || value == "1") {
    *parsed = true;
    return true;
  }
  if (value == "false" || value == "False" || value == "0") {
    *parsed = false;
    return true;
  }
  return false;
}

template <typename NumberT>
bool ParseNumber(const std::string& value, NumberT* parsed) {
  if (parsed == nullptr) {
    return false;
  }
  std::istringstream stream(value);
  NumberT result{};
  stream >> result;
  if (stream.fail()) {
    return false;
  }
  *parsed = result;
  return true;
}

float NormalizeAngle(float angle) {
  constexpr float kPi = 3.14159265358979323846F;
  while (angle > kPi) {
    angle -= 2.0F * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0F * kPi;
  }
  return angle;
}

rm_nav::common::Status ParseArgs(int argc, char** argv, Options* options) {
  if (options == nullptr) {
    return rm_nav::common::Status::InvalidArgument("options output is null");
  }
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--config" && index + 1 < argc) {
      options->config_dir = argv[++index];
      continue;
    }
    if (arg == "--output-dir" && index + 1 < argc) {
      options->output_dir = argv[++index];
      continue;
    }
    if (arg == "--scenario" && index + 1 < argc) {
      options->scenario = argv[++index];
      continue;
    }
    if (arg == "--scenario-file" && index + 1 < argc) {
      options->scenario_file = argv[++index];
      continue;
    }
    if (arg == "--duration-s" && index + 1 < argc) {
      options->duration_s = std::atoi(argv[++index]);
      continue;
    }
    if (arg == "--hz" && index + 1 < argc) {
      options->hz = std::atof(argv[++index]);
      continue;
    }
    if (arg == "--switch-goal-period-s" && index + 1 < argc) {
      options->switch_goal_period_s = std::atof(argv[++index]);
      continue;
    }
    if (arg == "--fixed-pose") {
      options->integrate_pose = false;
      options->force_fixed_pose = true;
      continue;
    }
    if (arg == "--no-foxglove") {
      options->foxglove_enabled = false;
      continue;
    }
    if (arg == "--foxglove-host" && index + 1 < argc) {
      options->foxglove_host = argv[++index];
      continue;
    }
    if (arg == "--foxglove-port" && index + 1 < argc) {
      options->foxglove_port = std::atoi(argv[++index]);
      continue;
    }
    return rm_nav::common::Status::InvalidArgument(
        "usage: mock_planner_generator [--config dir] [--output-dir dir] "
        "[--scenario-file file] "
        "[--scenario cross|head_on|chase|line|mixed] [--duration-s N] [--hz N] "
        "[--switch-goal-period-s N] [--fixed-pose] [--no-foxglove] "
        "[--foxglove-host ip] [--foxglove-port N]");
  }
  if (options->duration_s <= 0) {
    return rm_nav::common::Status::InvalidArgument("duration must be positive");
  }
  if (options->hz <= 0.0) {
    return rm_nav::common::Status::InvalidArgument("hz must be positive");
  }
  return rm_nav::common::Status::Ok();
}

std::vector<rm_nav::debug::FoxgloveChannel> BuildMockFoxgloveChannels() {
  return {
      {kPoseChannelId, "/rm_nav/mock/pose", "json", "rm_nav.Pose", "jsonschema", kPoseSchema},
      {kGoalChannelId, "/rm_nav/mock/goal", "json", "rm_nav.Goal", "jsonschema", kGoalSchema},
      {kCmdChannelId, "/rm_nav/mock/cmd", "json", "rm_nav.Cmd", "jsonschema", kCmdSchema},
      {kPlannerStatusChannelId, "/rm_nav/mock/planner_status", "json", "rm_nav.PlannerStatus",
       "jsonschema", kPlannerStatusSchema},
      {kPathChannelId, "/rm_nav/mock/global_path", "json", "rm_nav.Path2D", "jsonschema",
       kPathSchema},
      {kDynamicObstaclesChannelId, "/rm_nav/mock/dynamic_obstacles", "json",
       "rm_nav.DynamicObstacles", "jsonschema", kDynamicObstaclesSchema},
      {kTrajectoryChannelId, "/rm_nav/mock/trajectory", "json", "rm_nav.Trajectory", "jsonschema",
       kTrajectorySchema},
      {kStaticMapSceneChannelId, "/rm_nav/mock/static_map_scene", "json", "foxglove.SceneUpdate",
       "", ""},
      {kLocalCostmapSceneChannelId, "/rm_nav/mock/local_costmap_scene", "json",
       "foxglove.SceneUpdate", "", ""},
      {kDynamicMaxRiskChannelId, "/rm_nav/mock/metrics/dynamic_max_risk", "json",
       "rm_nav.Scalar", "jsonschema", kScalarSchema},
      {kDynamicIntegratedRiskChannelId, "/rm_nav/mock/metrics/dynamic_integrated_risk", "json",
       "rm_nav.Scalar", "jsonschema", kScalarSchema},
      {kDynamicClearanceMinChannelId, "/rm_nav/mock/metrics/dynamic_clearance_min", "json",
       "rm_nav.Scalar", "jsonschema", kScalarSchema},
      {kDynamicRisk05ChannelId, "/rm_nav/mock/metrics/dynamic_risk_05", "json",
       "rm_nav.Scalar", "jsonschema", kScalarSchema},
      {kDynamicRisk10ChannelId, "/rm_nav/mock/metrics/dynamic_risk_10", "json",
       "rm_nav.Scalar", "jsonschema", kScalarSchema},
  };
}

ScenarioConfig BuildDefaultScenario(const rm_nav::config::PlannerConfig& planner_config,
                                    const Options& options) {
  ScenarioConfig scenario;
  scenario.start_x_m = static_cast<float>(planner_config.supply_start_x_m);
  scenario.start_y_m = static_cast<float>(planner_config.supply_start_y_m);
  scenario.start_yaw_rad = 0.0F;
  scenario.primary_goal_x_m = static_cast<float>(planner_config.center_goal_x_m);
  scenario.primary_goal_y_m = static_cast<float>(planner_config.center_goal_y_m);
  scenario.secondary_goal_x_m = static_cast<float>(planner_config.supply_start_x_m);
  scenario.secondary_goal_y_m = static_cast<float>(planner_config.supply_start_y_m);
  scenario.integrate_pose = options.integrate_pose;
  scenario.switch_goal_period_s = static_cast<float>(options.switch_goal_period_s);
  scenario.walls = {
      {0, 0, 99, 0},  {0, 69, 99, 69}, {0, 0, 0, 69},     {99, 0, 99, 69},
      {34, 8, 34, 24}, {34, 36, 34, 58}, {58, 18, 78, 18}, {58, 48, 78, 48},
  };
  if (options.scenario == "cross" || options.scenario == "mixed") {
    scenario.obstacles.push_back({1U, MotionModel::kLinear, 3.8F, 1.0F, 0.0F, 0.7F, 0.32F, 0.0F,
                                  0.0F});
  }
  if (options.scenario == "head_on" || options.scenario == "mixed") {
    scenario.obstacles.push_back({2U, MotionModel::kLinear, 5.8F, 3.0F, -0.75F, 0.0F, 0.35F, 0.0F,
                                  0.0F});
  }
  if (options.scenario == "chase" || options.scenario == "mixed") {
    scenario.obstacles.push_back({3U, MotionModel::kChase, 0.0F, 0.0F, 0.9F, 0.03F, 0.30F, -1.2F,
                                  -0.25F});
  }
  if (options.scenario == "line" || options.scenario == "mixed") {
    scenario.obstacles.push_back({4U, MotionModel::kLinear, 2.2F, 2.55F, 0.5F, 0.0F, 0.28F, 0.0F,
                                  0.0F});
  }
  return scenario;
}

rm_nav::common::Status ApplyScalarOverride(const std::string& section, const std::string& key,
                                           const std::string& value,
                                           ScenarioConfig* scenario) {
  if (scenario == nullptr) {
    return rm_nav::common::Status::InvalidArgument("scenario output is null");
  }
  if (section == "mock") {
    if (key == "start_x_m" && ParseNumber(value, &scenario->start_x_m)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "start_y_m" && ParseNumber(value, &scenario->start_y_m)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "start_yaw_rad" && ParseNumber(value, &scenario->start_yaw_rad)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "primary_goal_x_m" && ParseNumber(value, &scenario->primary_goal_x_m)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "primary_goal_y_m" && ParseNumber(value, &scenario->primary_goal_y_m)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "primary_goal_yaw_rad" && ParseNumber(value, &scenario->primary_goal_yaw_rad)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "secondary_goal_x_m" && ParseNumber(value, &scenario->secondary_goal_x_m)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "secondary_goal_y_m" && ParseNumber(value, &scenario->secondary_goal_y_m)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "secondary_goal_yaw_rad" && ParseNumber(value, &scenario->secondary_goal_yaw_rad)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "switch_goal_period_s" && ParseNumber(value, &scenario->switch_goal_period_s)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "map_width_cells" && ParseNumber(value, &scenario->map_width_cells)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "map_height_cells" && ParseNumber(value, &scenario->map_height_cells)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "map_resolution_m" && ParseNumber(value, &scenario->map_resolution_m)) {
      return rm_nav::common::Status::Ok();
    }
    if (key == "integrate_pose" && ParseBool(value, &scenario->integrate_pose)) {
      return rm_nav::common::Status::Ok();
    }
  }
  return rm_nav::common::Status::InvalidArgument("unsupported mock scenario scalar: " + section +
                                                 "." + key);
}

rm_nav::common::Status ParseScenarioFile(const std::filesystem::path& path,
                                         ScenarioConfig* scenario) {
  if (scenario == nullptr) {
    return rm_nav::common::Status::InvalidArgument("scenario output is null");
  }

  std::ifstream input(path);
  if (!input.is_open()) {
    return rm_nav::common::Status::Unavailable("failed to open mock scenario file");
  }

  enum class ListMode {
    kNone = 0,
    kWalls,
    kObstacles,
  };

  scenario->walls.clear();
  scenario->obstacles.clear();
  std::string section;
  ListMode list_mode = ListMode::kNone;
  std::optional<WallSegment> pending_wall;
  std::optional<ScenarioObstacle> pending_obstacle;

  auto flush_pending = [&]() -> rm_nav::common::Status {
    if (pending_wall.has_value()) {
      scenario->walls.push_back(*pending_wall);
      pending_wall.reset();
    }
    if (pending_obstacle.has_value()) {
      scenario->obstacles.push_back(*pending_obstacle);
      pending_obstacle.reset();
    }
    return rm_nav::common::Status::Ok();
  };

  std::string line;
  while (std::getline(input, line)) {
    const auto hash_pos = line.find('#');
    if (hash_pos != std::string::npos) {
      line = line.substr(0, hash_pos);
    }
    const auto trimmed = Trim(line);
    if (trimmed.empty()) {
      continue;
    }

    const auto indent = line.find_first_not_of(' ');
    if (indent == std::string::npos || indent % 2U != 0U) {
      return rm_nav::common::Status::InvalidArgument("mock scenario yaml indentation is invalid");
    }

    if (indent == 0U) {
      if (!trimmed.empty() && trimmed.back() == ':') {
        const auto top_key = Trim(trimmed.substr(0, trimmed.size() - 1U));
        if (top_key == "mock" || top_key == "walls" || top_key == "obstacles") {
          auto status = flush_pending();
          if (!status.ok()) {
            return status;
          }
          section = top_key;
          list_mode = top_key == "walls"   ? ListMode::kWalls
                      : top_key == "obstacles" ? ListMode::kObstacles
                                               : ListMode::kNone;
          continue;
        }
      }
      return rm_nav::common::Status::InvalidArgument("unsupported mock scenario top-level key");
    }

    if (section == "mock") {
      const auto colon_pos = trimmed.find(':');
      if (colon_pos == std::string::npos) {
        return rm_nav::common::Status::InvalidArgument("mock scalar line missing ':'");
      }
      const auto key = Trim(trimmed.substr(0, colon_pos));
      const auto value = StripQuotes(Trim(trimmed.substr(colon_pos + 1U)));
      auto status = ApplyScalarOverride("mock", key, value, scenario);
      if (!status.ok()) {
        return status;
      }
      continue;
    }

    if (list_mode == ListMode::kWalls || list_mode == ListMode::kObstacles) {
      if (trimmed.rfind("- ", 0U) == 0U) {
        auto status = flush_pending();
        if (!status.ok()) {
          return status;
        }
        if (list_mode == ListMode::kWalls) {
          pending_wall = WallSegment{};
        } else {
          pending_obstacle = ScenarioObstacle{};
        }

        const auto inline_item = Trim(trimmed.substr(2U));
        if (inline_item.empty()) {
          continue;
        }
        const auto colon_pos = inline_item.find(':');
        if (colon_pos == std::string::npos) {
          return rm_nav::common::Status::InvalidArgument("mock list item missing ':'");
        }
        const auto key = Trim(inline_item.substr(0, colon_pos));
        const auto value = StripQuotes(Trim(inline_item.substr(colon_pos + 1U)));
        if (list_mode == ListMode::kWalls) {
          if (key == "x0" && ParseNumber(value, &pending_wall->x0)) {
            continue;
          }
          if (key == "y0" && ParseNumber(value, &pending_wall->y0)) {
            continue;
          }
          if (key == "x1" && ParseNumber(value, &pending_wall->x1)) {
            continue;
          }
          if (key == "y1" && ParseNumber(value, &pending_wall->y1)) {
            continue;
          }
        } else {
          if (key == "id" && ParseNumber(value, &pending_obstacle->id)) {
            continue;
          }
          if (key == "motion") {
            pending_obstacle->motion =
                value == "chase" ? MotionModel::kChase : MotionModel::kLinear;
            continue;
          }
          if (key == "x_m" && ParseNumber(value, &pending_obstacle->x_m)) {
            continue;
          }
          if (key == "y_m" && ParseNumber(value, &pending_obstacle->y_m)) {
            continue;
          }
          if (key == "vx_mps" && ParseNumber(value, &pending_obstacle->vx_mps)) {
            continue;
          }
          if (key == "vy_mps" && ParseNumber(value, &pending_obstacle->vy_mps)) {
            continue;
          }
          if (key == "radius_m" && ParseNumber(value, &pending_obstacle->radius_m)) {
            continue;
          }
          if (key == "offset_x_m" && ParseNumber(value, &pending_obstacle->offset_x_m)) {
            continue;
          }
          if (key == "offset_y_m" && ParseNumber(value, &pending_obstacle->offset_y_m)) {
            continue;
          }
        }
        return rm_nav::common::Status::InvalidArgument("unsupported mock list key");
      }

      const auto colon_pos = trimmed.find(':');
      if (colon_pos == std::string::npos) {
        return rm_nav::common::Status::InvalidArgument("mock nested line missing ':'");
      }
      const auto key = Trim(trimmed.substr(0, colon_pos));
      const auto value = StripQuotes(Trim(trimmed.substr(colon_pos + 1U)));
      if (list_mode == ListMode::kWalls && pending_wall.has_value()) {
        if (key == "x0" && ParseNumber(value, &pending_wall->x0)) {
          continue;
        }
        if (key == "y0" && ParseNumber(value, &pending_wall->y0)) {
          continue;
        }
        if (key == "x1" && ParseNumber(value, &pending_wall->x1)) {
          continue;
        }
        if (key == "y1" && ParseNumber(value, &pending_wall->y1)) {
          continue;
        }
      }
      if (list_mode == ListMode::kObstacles && pending_obstacle.has_value()) {
        if (key == "id" && ParseNumber(value, &pending_obstacle->id)) {
          continue;
        }
        if (key == "motion") {
          pending_obstacle->motion =
              value == "chase" ? MotionModel::kChase : MotionModel::kLinear;
          continue;
        }
        if (key == "x_m" && ParseNumber(value, &pending_obstacle->x_m)) {
          continue;
        }
        if (key == "y_m" && ParseNumber(value, &pending_obstacle->y_m)) {
          continue;
        }
        if (key == "vx_mps" && ParseNumber(value, &pending_obstacle->vx_mps)) {
          continue;
        }
        if (key == "vy_mps" && ParseNumber(value, &pending_obstacle->vy_mps)) {
          continue;
        }
        if (key == "radius_m" && ParseNumber(value, &pending_obstacle->radius_m)) {
          continue;
        }
        if (key == "offset_x_m" && ParseNumber(value, &pending_obstacle->offset_x_m)) {
          continue;
        }
        if (key == "offset_y_m" && ParseNumber(value, &pending_obstacle->offset_y_m)) {
          continue;
        }
      }
      return rm_nav::common::Status::InvalidArgument("unsupported mock nested key");
    }
  }

  auto status = flush_pending();
  if (!status.ok()) {
    return status;
  }
  return rm_nav::common::Status::Ok();
}

rm_nav::data::Pose3f MakePose(rm_nav::common::TimePoint stamp, float x, float y, float yaw) {
  rm_nav::data::Pose3f pose;
  pose.stamp = stamp;
  pose.reference_frame = rm_nav::tf::kMapFrame;
  pose.child_frame = rm_nav::tf::kBaseLinkFrame;
  pose.position.x = x;
  pose.position.y = y;
  pose.rpy.z = yaw;
  pose.is_valid = true;
  return pose;
}

rm_nav::data::DynamicObstacle MakeObstacle(rm_nav::common::TimePoint stamp,
                                           rm_nav::common::ObjectId id, float x, float y,
                                           float vx, float vy, float radius_m) {
  rm_nav::data::DynamicObstacle obstacle;
  obstacle.id = id;
  obstacle.pose = MakePose(stamp, x, y, 0.0F);
  obstacle.velocity.x = vx;
  obstacle.velocity.y = vy;
  obstacle.predicted_pose_05s = MakePose(stamp, x + 0.5F * vx, y + 0.5F * vy, 0.0F);
  obstacle.predicted_pose_10s = MakePose(stamp, x + 1.0F * vx, y + 1.0F * vy, 0.0F);
  obstacle.radius_m = radius_m;
  obstacle.confidence = 1.0F;
  obstacle.age = 1U;
  obstacle.is_confirmed = true;
  return obstacle;
}

void SetOccupied(rm_nav::data::GridMap2D* map, int x, int y) {
  if (map == nullptr || x < 0 || y < 0 || x >= static_cast<int>(map->width) ||
      y >= static_cast<int>(map->height)) {
    return;
  }
  map->occupancy[static_cast<std::size_t>(y) * map->width + static_cast<std::size_t>(x)] = 100U;
}

void AddRectWall(rm_nav::data::GridMap2D* map, int x0, int y0, int x1, int y1) {
  if (map == nullptr) {
    return;
  }
  const int begin_x = std::min(x0, x1);
  const int end_x = std::max(x0, x1);
  const int begin_y = std::min(y0, y1);
  const int end_y = std::max(y0, y1);
  for (int y = begin_y; y <= end_y; ++y) {
    for (int x = begin_x; x <= end_x; ++x) {
      SetOccupied(map, x, y);
    }
  }
}

rm_nav::localization::StaticMap BuildSyntheticStaticMap(const ScenarioConfig& scenario) {
  rm_nav::localization::StaticMap map;
  map.occupancy.stamp = rm_nav::common::Now();
  map.occupancy.resolution_m = scenario.map_resolution_m;
  map.occupancy.width = scenario.map_width_cells;
  map.occupancy.height = scenario.map_height_cells;
  map.occupancy.origin = MakePose(map.occupancy.stamp, 0.0F, 0.0F, 0.0F);
  map.occupancy.origin.child_frame = rm_nav::tf::kMapFrame;
  map.occupancy.occupancy.assign(map.occupancy.width * map.occupancy.height, 0U);

  for (const auto& wall : scenario.walls) {
    AddRectWall(&map.occupancy, wall.x0, wall.y0, wall.x1, wall.y1);
  }

  for (std::uint32_t y = 0; y < map.occupancy.height; ++y) {
    for (std::uint32_t x = 0; x < map.occupancy.width; ++x) {
      const auto index = static_cast<std::size_t>(y) * map.occupancy.width + x;
      if (map.occupancy.occupancy[index] < 50U) {
        continue;
      }
      rm_nav::data::PointXYZI point;
      point.x = map.occupancy.origin.position.x +
                (static_cast<float>(x) + 0.5F) * map.occupancy.resolution_m;
      point.y = map.occupancy.origin.position.y +
                (static_cast<float>(y) + 0.5F) * map.occupancy.resolution_m;
      point.z = 0.0F;
      point.intensity = 100.0F;
      map.global_points.push_back(point);
    }
  }

  map.occupancy_loaded = true;
  map.global_map_loaded = true;
  map.occupancy_path = "synthetic";
  map.global_map_path = "synthetic";
  return map;
}

std::uint8_t SampleStaticOccupancy(const rm_nav::data::GridMap2D& map, float world_x,
                                   float world_y) {
  if (map.width == 0U || map.height == 0U || map.occupancy.empty()) {
    return 0U;
  }
  const float local_x = world_x - map.origin.position.x;
  const float local_y = world_y - map.origin.position.y;
  const int gx = static_cast<int>(std::floor(local_x / map.resolution_m));
  const int gy = static_cast<int>(std::floor(local_y / map.resolution_m));
  if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.width) ||
      gy >= static_cast<int>(map.height)) {
    return 100U;
  }
  return map.occupancy[static_cast<std::size_t>(gy) * map.width + static_cast<std::size_t>(gx)];
}

rm_nav::data::GridMap2D BuildLocalCostmap(const rm_nav::data::GridMap2D& static_map,
                                          const rm_nav::data::Pose3f& pose) {
  rm_nav::data::GridMap2D local_costmap;
  local_costmap.stamp = pose.stamp;
  local_costmap.resolution_m = 0.1F;
  local_costmap.width = 80U;
  local_costmap.height = 80U;
  local_costmap.origin = pose;
  local_costmap.occupancy.assign(local_costmap.width * local_costmap.height, 0U);

  const int center_x = static_cast<int>(local_costmap.width / 2U);
  const int center_y = static_cast<int>(local_costmap.height / 2U);
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  for (std::uint32_t gy = 0; gy < local_costmap.height; ++gy) {
    for (std::uint32_t gx = 0; gx < local_costmap.width; ++gx) {
      const float local_x =
          static_cast<float>(static_cast<int>(gx) - center_x) * local_costmap.resolution_m;
      const float local_y =
          static_cast<float>(static_cast<int>(gy) - center_y) * local_costmap.resolution_m;
      const float world_x = pose.position.x + cos_yaw * local_x - sin_yaw * local_y;
      const float world_y = pose.position.y + sin_yaw * local_x + cos_yaw * local_y;
      local_costmap.occupancy[static_cast<std::size_t>(gy) * local_costmap.width + gx] =
          SampleStaticOccupancy(static_map, world_x, world_y);
    }
  }
  return local_costmap;
}

std::vector<rm_nav::data::PointXYZI> CollectLocalCostmapPoints(
    const rm_nav::data::GridMap2D& costmap, const rm_nav::data::Pose3f& pose) {
  std::vector<rm_nav::data::PointXYZI> points;
  const int center_x = static_cast<int>(costmap.width / 2U);
  const int center_y = static_cast<int>(costmap.height / 2U);
  const float cos_yaw = std::cos(pose.rpy.z);
  const float sin_yaw = std::sin(pose.rpy.z);
  for (std::uint32_t gy = 0; gy < costmap.height; ++gy) {
    for (std::uint32_t gx = 0; gx < costmap.width; ++gx) {
      if (costmap.occupancy[static_cast<std::size_t>(gy) * costmap.width + gx] < 50U) {
        continue;
      }
      const float local_x =
          static_cast<float>(static_cast<int>(gx) - center_x) * costmap.resolution_m;
      const float local_y =
          static_cast<float>(static_cast<int>(gy) - center_y) * costmap.resolution_m;
      rm_nav::data::PointXYZI point;
      point.x = pose.position.x + cos_yaw * local_x - sin_yaw * local_y;
      point.y = pose.position.y + sin_yaw * local_x + cos_yaw * local_y;
      point.z = 0.0F;
      point.intensity = 80.0F;
      points.push_back(point);
    }
  }
  return points;
}

rm_nav::data::Pose3f SelectGoalPose(const ScenarioConfig& scenario,
                                    rm_nav::common::TimePoint stamp, double sim_time_s) {
  const bool use_start_goal =
      scenario.switch_goal_period_s > 0.0F &&
      (static_cast<int>(sim_time_s / scenario.switch_goal_period_s) % 2 != 0);
  const float goal_x =
      use_start_goal ? scenario.secondary_goal_x_m : scenario.primary_goal_x_m;
  const float goal_y =
      use_start_goal ? scenario.secondary_goal_y_m : scenario.primary_goal_y_m;
  const float goal_yaw =
      use_start_goal ? scenario.secondary_goal_yaw_rad : scenario.primary_goal_yaw_rad;
  return MakePose(stamp, goal_x, goal_y, goal_yaw);
}

void AppendObstacle(MockPlannerFrame* frame, const rm_nav::data::DynamicObstacle& obstacle) {
  if (frame == nullptr || frame->obstacle_count >= frame->obstacles.size()) {
    return;
  }
  frame->obstacles[frame->obstacle_count++] = obstacle;
}

void PopulateObstacles(const ScenarioConfig& scenario, double sim_time_s,
                       const rm_nav::data::Pose3f& current_pose, MockPlannerFrame* frame) {
  if (frame == nullptr) {
    return;
  }

  const auto stamp = frame->stamp;
  for (const auto& config_obstacle : scenario.obstacles) {
    float obstacle_x = config_obstacle.x_m;
    float obstacle_y = config_obstacle.y_m;
    if (config_obstacle.motion == MotionModel::kLinear) {
      obstacle_x += config_obstacle.vx_mps * static_cast<float>(sim_time_s);
      obstacle_y += config_obstacle.vy_mps * static_cast<float>(sim_time_s);
    } else if (config_obstacle.motion == MotionModel::kChase) {
      obstacle_x = current_pose.position.x + config_obstacle.offset_x_m +
                   config_obstacle.vx_mps * static_cast<float>(sim_time_s);
      obstacle_y = current_pose.position.y + config_obstacle.offset_y_m +
                   config_obstacle.vy_mps * static_cast<float>(sim_time_s);
    }
    AppendObstacle(frame, MakeObstacle(stamp, config_obstacle.id, obstacle_x, obstacle_y,
                                       config_obstacle.vx_mps, config_obstacle.vy_mps,
                                       config_obstacle.radius_m));
  }
}

void WritePcd(const std::filesystem::path& path,
              const std::vector<rm_nav::data::PointXYZI>& points) {
  std::ofstream output(path);
  output << "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n";
  output << "COUNT 1 1 1 1\nWIDTH " << points.size() << "\nHEIGHT 1\n";
  output << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size() << "\nDATA ascii\n";
  for (const auto& point : points) {
    output << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << "\n";
  }
}

void WriteTextFile(const std::filesystem::path& path, const std::string& text) {
  std::ofstream output(path);
  output << text;
}

std::string BuildPathJson(const rm_nav::data::Path2D& global_path) {
  std::ostringstream output;
  output << "{\n  \"points\": [\n";
  for (std::size_t index = 0; index < global_path.points.size(); ++index) {
    const auto& point = global_path.points[index];
    output << "    {\"x\": " << point.position.x << ", \"y\": " << point.position.y
           << ", \"heading\": " << point.heading_rad << ", \"speed\": "
           << point.target_speed_mps << "}";
    if (index + 1U != global_path.points.size()) {
      output << ',';
    }
    output << "\n";
  }
  output << "  ]\n}\n";
  return output.str();
}

std::string BuildObstaclesJson(const std::vector<rm_nav::data::DynamicObstacle>& obstacles) {
  std::ostringstream output;
  output << "{\n  \"obstacles\": [\n";
  for (std::size_t index = 0; index < obstacles.size(); ++index) {
    const auto& obstacle = obstacles[index];
    output << "    {\"id\": " << obstacle.id << ", \"x\": " << obstacle.pose.position.x
           << ", \"y\": " << obstacle.pose.position.y << ", \"vx\": " << obstacle.velocity.x
           << ", \"vy\": " << obstacle.velocity.y << ", \"radius_m\": " << obstacle.radius_m
           << "}";
    if (index + 1U != obstacles.size()) {
      output << ',';
    }
    output << "\n";
  }
  output << "  ]\n}\n";
  return output.str();
}

std::string BuildPoseJson(const rm_nav::data::Pose3f& pose) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"x\": " << pose.position.x << ",\n";
  output << "  \"y\": " << pose.position.y << ",\n";
  output << "  \"yaw\": " << pose.rpy.z << "\n";
  output << "}\n";
  return output.str();
}

std::string BuildGoalJson(const rm_nav::data::Pose3f& goal_pose) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"x\": " << goal_pose.position.x << ",\n";
  output << "  \"y\": " << goal_pose.position.y << ",\n";
  output << "  \"yaw\": " << goal_pose.rpy.z << "\n";
  output << "}\n";
  return output.str();
}

std::string BuildCmdJson(const rm_nav::data::ChassisCmd& cmd) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"vx\": " << cmd.vx_mps << ",\n";
  output << "  \"vy\": " << cmd.vy_mps << ",\n";
  output << "  \"wz\": " << cmd.wz_radps << ",\n";
  output << "  \"brake\": " << (cmd.brake ? "true" : "false") << "\n";
  output << "}\n";
  return output.str();
}

std::string BuildScalarJson(float value) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"value\": " << value << "\n";
  output << "}\n";
  return output.str();
}

std::string BuildPlannerStatusJson(const rm_nav::planning::PlannerStatus& status,
                                   const rm_nav::data::ChassisCmd& cmd) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"mode\": \""
         << (status.mode == rm_nav::planning::GoalMode::kCenterHold ? "center_hold"
                                                                    : "approach_center")
         << "\",\n";
  output << "  \"distance_to_goal_m\": " << status.distance_to_goal_m << ",\n";
  output << "  \"distance_to_center_m\": " << status.distance_to_center_m << ",\n";
  output << "  \"yaw_error_rad\": " << status.yaw_error_rad << ",\n";
  output << "  \"reached\": " << (status.reached ? "true" : "false") << ",\n";
  output << "  \"cmd\": {\"vx\": " << cmd.vx_mps << ", \"vy\": " << cmd.vy_mps
         << ", \"wz\": " << cmd.wz_radps << "},\n";
  output << "  \"dwa_score\": {\n";
  output << "    \"goal\": " << status.dwa_score.goal_score << ",\n";
  output << "    \"path\": " << status.dwa_score.path_score << ",\n";
  output << "    \"smooth\": " << status.dwa_score.smooth_score << ",\n";
  output << "    \"heading\": " << status.dwa_score.heading_score << ",\n";
  output << "    \"clearance\": " << status.dwa_score.clearance_score << ",\n";
  output << "    \"velocity\": " << status.dwa_score.velocity_score << ",\n";
  output << "    \"dynamic\": " << status.dwa_score.dynamic_risk_score << ",\n";
  output << "    \"dynamic_max_risk\": " << status.dwa_score.dynamic_max_risk << ",\n";
  output << "    \"dynamic_integrated_risk\": " << status.dwa_score.dynamic_integrated_risk
         << ",\n";
  output << "    \"dynamic_clearance_min\": " << status.dwa_score.dynamic_clearance_min
         << ",\n";
  output << "    \"dynamic_risk_05\": " << status.dwa_score.dynamic_risk_05 << ",\n";
  output << "    \"dynamic_risk_10\": " << status.dwa_score.dynamic_risk_10 << ",\n";
  output << "    \"total\": " << status.dwa_score.total_score << "\n";
  output << "  }\n";
  output << "}\n";
  return output.str();
}

std::string BuildTrajectoryJson(const std::vector<rm_nav::data::Pose3f>& poses) {
  std::ostringstream output;
  output << "{\n  \"poses\": [\n";
  for (std::size_t index = 0; index < poses.size(); ++index) {
    const auto& pose = poses[index];
    output << "    {\"x\": " << pose.position.x << ", \"y\": " << pose.position.y
           << ", \"yaw\": " << pose.rpy.z << "}";
    if (index + 1U != poses.size()) {
      output << ',';
    }
    output << "\n";
  }
  output << "  ]\n}\n";
  return output.str();
}

std::string BuildSceneUpdateJson(const std::string& entity_id, std::string_view frame_id,
                                 const std::vector<rm_nav::data::PointXYZI>& points,
                                 float cube_size_xy_m, float cube_size_z_m,
                                 RgbaColor color) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"deletions\": [],\n";
  output << "  \"entities\": [\n";
  output << "    {\n";
  output << "      \"id\": \"" << entity_id << "\",\n";
  output << "      \"frame_id\": \"" << frame_id << "\",\n";
  output << "      \"frameId\": \"" << frame_id << "\",\n";
  output << "      \"frame_locked\": false,\n";
  output << "      \"frameLocked\": false,\n";
  output << "      \"cubes\": [\n";
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto& point = points[index];
    output << "        {\"pose\": {\"position\": {\"x\": " << point.x << ", \"y\": " << point.y
           << ", \"z\": " << point.z
           << "}, \"orientation\": {\"x\": 0, \"y\": 0, \"z\": 0, \"w\": 1}}, "
           << "\"size\": {\"x\": " << cube_size_xy_m << ", \"y\": " << cube_size_xy_m
           << ", \"z\": " << cube_size_z_m << "}, "
           << "\"color\": {\"r\": " << color.r << ", \"g\": " << color.g << ", \"b\": "
           << color.b << ", \"a\": " << color.a << "}}";
    if (index + 1U != points.size()) {
      output << ",";
    }
    output << "\n";
  }
  output << "      ]\n";
  output << "    }\n";
  output << "  ]\n";
  output << "}\n";
  return output.str();
}

void WriteSummary(const std::filesystem::path& path, const Options& options,
                  const ProducerStats& producer_stats, const ConsumerStats& consumer_stats,
                  const MockQueue& queue, const rm_nav::data::ChassisCmd& last_cmd) {
  std::ofstream output(path);
  output << "scenario=" << options.scenario << "\n";
  output << "scenario_file=" << (options.scenario_file.empty() ? "(builtin)" : options.scenario_file)
         << "\n";
  output << "duration_s=" << options.duration_s << "\n";
  output << "hz=" << options.hz << "\n";
  output << "integrate_pose=" << (options.integrate_pose ? "true" : "false") << "\n";
  output << "produced=" << producer_stats.produced.load() << "\n";
  output << "consumed=" << consumer_stats.consumed.load() << "\n";
  output << "planned=" << consumer_stats.planned.load() << "\n";
  output << "producer_dropped=" << producer_stats.dropped.load() << "\n";
  output << "queue_dropped=" << queue.dropped_pushes() << "\n";
  output << "last_cmd=" << last_cmd.vx_mps << "," << last_cmd.vy_mps << "," << last_cmd.wz_radps
         << ", brake=" << (last_cmd.brake ? "true" : "false") << "\n";
  if (options.foxglove_enabled && options.foxglove_port > 0) {
    output << "foxglove_ws=ws://" << options.foxglove_host << ":" << options.foxglove_port
           << "\n";
  }
}

void PublishFoxgloveJson(rm_nav::debug::FoxgloveServer* server, std::uint32_t channel_id,
                         const std::string& payload, rm_nav::common::TimePoint stamp) {
  if (server == nullptr || !server->is_running()) {
    return;
  }
  server->PublishJson(channel_id, payload, stamp);
}

void ProducerLoop(const Options& options, const ScenarioConfig& scenario,
                  PlannerCmdMirror* latest_cmd, MockQueue* queue, ProducerStats* stats) {
  if (latest_cmd == nullptr || queue == nullptr || stats == nullptr) {
    return;
  }

  const float dt_s = static_cast<float>(1.0 / options.hz);
  auto next_tick = rm_nav::common::Now();
  float x = scenario.start_x_m;
  float y = scenario.start_y_m;
  float yaw = scenario.start_yaw_rad;
  const int total_ticks = static_cast<int>(std::round(options.duration_s * options.hz));
  for (int tick = 0; tick < total_ticks; ++tick) {
    const auto stamp = rm_nav::common::Now();
    if (options.integrate_pose) {
      const float vx = latest_cmd->brake.load(std::memory_order_relaxed)
                           ? 0.0F
                           : latest_cmd->vx_mps.load(std::memory_order_relaxed);
      const float vy = latest_cmd->brake.load(std::memory_order_relaxed)
                           ? 0.0F
                           : latest_cmd->vy_mps.load(std::memory_order_relaxed);
      const float wz = latest_cmd->brake.load(std::memory_order_relaxed)
                           ? 0.0F
                           : latest_cmd->wz_radps.load(std::memory_order_relaxed);
      const float cos_yaw = std::cos(yaw);
      const float sin_yaw = std::sin(yaw);
      x += (vx * cos_yaw - vy * sin_yaw) * dt_s;
      y += (vx * sin_yaw + vy * cos_yaw) * dt_s;
      yaw = NormalizeAngle(yaw + wz * dt_s);
    }

    MockPlannerFrame frame{};
    frame.sequence = static_cast<std::uint32_t>(tick);
    frame.stamp = stamp;
    frame.current_pose = MakePose(stamp, x, y, yaw);
    frame.goal_pose = SelectGoalPose(scenario, stamp, tick * dt_s);
    PopulateObstacles(scenario, tick * dt_s, frame.current_pose, &frame);

    if (!queue->try_push(std::move(frame))) {
      stats->dropped.fetch_add(1U, std::memory_order_relaxed);
    } else {
      stats->produced.fetch_add(1U, std::memory_order_relaxed);
    }
    next_tick += std::chrono::duration_cast<rm_nav::common::Duration>(
        std::chrono::duration<double>(1.0 / options.hz));
    rm_nav::common::SleepUntil(next_tick);
  }
}

rm_nav::common::Status ConsumerLoop(
    const Options& options, const rm_nav::localization::StaticMap& static_map,
    const rm_nav::config::PlannerConfig& planner_config, PlannerCmdMirror* latest_cmd,
    std::atomic<bool>* producer_finished, rm_nav::debug::FoxgloveServer* foxglove_server,
    MockQueue* queue, ConsumerStats* stats) {
  if (latest_cmd == nullptr || queue == nullptr || stats == nullptr) {
    return rm_nav::common::Status::InvalidArgument("mock planner loop inputs are null");
  }
  if (producer_finished == nullptr) {
    return rm_nav::common::Status::InvalidArgument("producer_finished flag is null");
  }

  rm_nav::planning::PlannerCoordinator planner;
  auto status = planner.Initialize(planner_config, static_map);
  if (!status.ok()) {
    return status;
  }

  std::filesystem::create_directories(options.output_dir);
  WritePcd(std::filesystem::path(options.output_dir) / "static_map.pcd", static_map.global_points);
  const auto static_scene_json =
      BuildSceneUpdateJson("static_map", rm_nav::tf::kMapFrame, static_map.global_points,
                           static_map.occupancy.resolution_m, 0.02F, {0.35F, 0.35F, 0.35F, 0.85F});

  std::vector<rm_nav::data::Pose3f> pose_history;
  pose_history.reserve(static_cast<std::size_t>(options.duration_s * options.hz) + 4U);

  while (!producer_finished->load(std::memory_order_acquire) || !queue->empty()) {
    MockPlannerFrame frame{};
    if (!queue->try_pop(&frame)) {
      rm_nav::common::SleepFor(std::chrono::milliseconds(1));
      continue;
    }

    stats->consumed.fetch_add(1U, std::memory_order_relaxed);
    pose_history.push_back(frame.current_pose);

    std::vector<rm_nav::data::DynamicObstacle> obstacles;
    obstacles.reserve(frame.obstacle_count);
    for (std::size_t index = 0; index < frame.obstacle_count; ++index) {
      obstacles.push_back(frame.obstacles[index]);
    }

    const auto local_costmap = BuildLocalCostmap(static_map.occupancy, frame.current_pose);
    status = planner.PlanAndPublishToGoal(frame.current_pose, frame.goal_pose, local_costmap,
                                          obstacles);
    if (!status.ok()) {
      return status;
    }
    stats->planned.fetch_add(1U, std::memory_order_relaxed);

    const auto last_cmd = planner.LatestCmd();
    latest_cmd->vx_mps.store(last_cmd.vx_mps, std::memory_order_relaxed);
    latest_cmd->vy_mps.store(last_cmd.vy_mps, std::memory_order_relaxed);
    latest_cmd->wz_radps.store(last_cmd.wz_radps, std::memory_order_relaxed);
    latest_cmd->brake.store(last_cmd.brake, std::memory_order_relaxed);

    const auto pose_json = BuildPoseJson(frame.current_pose);
    const auto goal_json = BuildGoalJson(frame.goal_pose);
    const auto cmd_json = BuildCmdJson(last_cmd);
    const auto latest_status = planner.LatestStatus();
    const auto planner_status_json = BuildPlannerStatusJson(latest_status, last_cmd);
    const auto dynamic_max_risk_json =
        BuildScalarJson(latest_status.dwa_score.dynamic_max_risk);
    const auto dynamic_integrated_risk_json =
        BuildScalarJson(latest_status.dwa_score.dynamic_integrated_risk);
    const auto dynamic_clearance_min_json =
        BuildScalarJson(latest_status.dwa_score.dynamic_clearance_min);
    const auto dynamic_risk_05_json =
        BuildScalarJson(latest_status.dwa_score.dynamic_risk_05);
    const auto dynamic_risk_10_json =
        BuildScalarJson(latest_status.dwa_score.dynamic_risk_10);
    const auto path_json = BuildPathJson(planner.LatestPath());
    const auto obstacles_json = BuildObstaclesJson(obstacles);
    const auto trajectory_json = BuildTrajectoryJson(pose_history);
    const auto local_costmap_scene_json =
        BuildSceneUpdateJson("local_costmap", rm_nav::tf::kMapFrame,
                             CollectLocalCostmapPoints(local_costmap, frame.current_pose),
                             local_costmap.resolution_m, 0.03F, {0.95F, 0.25F, 0.25F, 0.75F});

    WriteTextFile(std::filesystem::path(options.output_dir) / "pose.json", pose_json);
    WriteTextFile(std::filesystem::path(options.output_dir) / "goal.json", goal_json);
    WriteTextFile(std::filesystem::path(options.output_dir) / "cmd.json", cmd_json);
    WriteTextFile(std::filesystem::path(options.output_dir) / "planner_status.json",
                  planner_status_json);
    WriteTextFile(std::filesystem::path(options.output_dir) / "global_path.json", path_json);
    WriteTextFile(std::filesystem::path(options.output_dir) / "dynamic_obstacles.json",
                  obstacles_json);
    WriteTextFile(std::filesystem::path(options.output_dir) / "trajectory.json", trajectory_json);
    WritePcd(std::filesystem::path(options.output_dir) / "local_costmap.pcd",
             CollectLocalCostmapPoints(local_costmap, frame.current_pose));

    PublishFoxgloveJson(foxglove_server, kPoseChannelId, pose_json, frame.stamp);
    PublishFoxgloveJson(foxglove_server, kGoalChannelId, goal_json, frame.stamp);
    PublishFoxgloveJson(foxglove_server, kCmdChannelId, cmd_json, frame.stamp);
    PublishFoxgloveJson(foxglove_server, kPlannerStatusChannelId, planner_status_json,
                        frame.stamp);
    PublishFoxgloveJson(foxglove_server, kPathChannelId, path_json, frame.stamp);
    PublishFoxgloveJson(foxglove_server, kDynamicObstaclesChannelId, obstacles_json,
                        frame.stamp);
    PublishFoxgloveJson(foxglove_server, kTrajectoryChannelId, trajectory_json, frame.stamp);
    PublishFoxgloveJson(foxglove_server, kDynamicMaxRiskChannelId, dynamic_max_risk_json,
                        frame.stamp);
    PublishFoxgloveJson(foxglove_server, kDynamicIntegratedRiskChannelId,
                        dynamic_integrated_risk_json, frame.stamp);
    PublishFoxgloveJson(foxglove_server, kDynamicClearanceMinChannelId,
                        dynamic_clearance_min_json, frame.stamp);
    PublishFoxgloveJson(foxglove_server, kDynamicRisk05ChannelId, dynamic_risk_05_json,
                        frame.stamp);
    PublishFoxgloveJson(foxglove_server, kDynamicRisk10ChannelId, dynamic_risk_10_json,
                        frame.stamp);
    PublishFoxgloveJson(foxglove_server, kStaticMapSceneChannelId, static_scene_json,
                        frame.stamp);
    PublishFoxgloveJson(foxglove_server, kLocalCostmapSceneChannelId, local_costmap_scene_json,
                        frame.stamp);
  }

  return rm_nav::common::Status::Ok();
}

}  // namespace

int main(int argc, char** argv) {
  rm_nav::utils::Logger::Instance().Initialize({rm_nav::utils::LogLevel::kInfo});

  Options options;
  auto status = ParseArgs(argc, argv, &options);
  if (!status.ok()) {
    rm_nav::utils::LogError("mock_planner", status.message);
    return 1;
  }

  rm_nav::config::ConfigLoader loader;
  rm_nav::config::LoadedConfig config;
  status = loader.LoadFromDirectory(options.config_dir, &config);
  if (!status.ok()) {
    rm_nav::utils::LogError("mock_planner", status.message);
    return 1;
  }

  if (options.scenario_file.empty()) {
    const auto default_scenario_path =
        std::filesystem::path(options.config_dir) / "mock_scenario.yaml";
    if (std::filesystem::exists(default_scenario_path)) {
      options.scenario_file = default_scenario_path.lexically_normal().string();
    }
  }

  ScenarioConfig scenario = BuildDefaultScenario(config.planner, options);
  if (!options.scenario_file.empty()) {
    status = ParseScenarioFile(options.scenario_file, &scenario);
    if (!status.ok()) {
      rm_nav::utils::LogError("mock_planner", status.message);
      return 1;
    }
    if (!options.force_fixed_pose) {
      options.integrate_pose = scenario.integrate_pose;
    }
    if (options.switch_goal_period_s <= 0.0 && scenario.switch_goal_period_s > 0.0F) {
      options.switch_goal_period_s = scenario.switch_goal_period_s;
    }
  }
  if (scenario.obstacles.size() > kMaxObstacles) {
    rm_nav::utils::LogError("mock_planner", "mock scenario obstacle count exceeds queue frame capacity");
    return 1;
  }

  if (options.foxglove_port <= 0) {
    options.foxglove_port = config.debug.websocket_port;
  }

  rm_nav::debug::FoxgloveServer foxglove_server;
  if (options.foxglove_enabled) {
    rm_nav::debug::FoxgloveServerOptions foxglove_options;
    foxglove_options.name = "rm_nav_mock";
    foxglove_options.host = options.foxglove_host;
    foxglove_options.port = static_cast<std::uint16_t>(options.foxglove_port);
    status = foxglove_server.Start(foxglove_options);
    if (!status.ok()) {
      rm_nav::utils::LogError("mock_planner", status.message);
      return 1;
    }
    status = foxglove_server.Advertise(BuildMockFoxgloveChannels());
    if (!status.ok()) {
      rm_nav::utils::LogError("mock_planner", status.message);
      foxglove_server.Stop();
      return 1;
    }
  }

  MockQueue queue;
  PlannerCmdMirror latest_cmd;
  ProducerStats producer_stats;
  ConsumerStats consumer_stats;
  std::atomic<bool> producer_finished{false};
  const auto static_map = BuildSyntheticStaticMap(scenario);

  std::thread producer([&]() {
    ProducerLoop(options, scenario, &latest_cmd, &queue, &producer_stats);
    producer_finished.store(true, std::memory_order_release);
  });
  status = ConsumerLoop(options, static_map, config.planner, &latest_cmd, &producer_finished,
                        &foxglove_server, &queue, &consumer_stats);
  producer.join();
  foxglove_server.Stop();

  if (!status.ok()) {
    rm_nav::utils::LogError("mock_planner", status.message);
    return 1;
  }

  rm_nav::data::ChassisCmd last_cmd;
  last_cmd.vx_mps = latest_cmd.vx_mps.load(std::memory_order_relaxed);
  last_cmd.vy_mps = latest_cmd.vy_mps.load(std::memory_order_relaxed);
  last_cmd.wz_radps = latest_cmd.wz_radps.load(std::memory_order_relaxed);
  last_cmd.brake = latest_cmd.brake.load(std::memory_order_relaxed);
  WriteSummary(std::filesystem::path(options.output_dir) / "summary.txt", options, producer_stats,
               consumer_stats, queue, last_cmd);

  rm_nav::utils::LogInfo(
      "mock_planner",
      "summary=" + std::string(" produced=") +
          std::to_string(producer_stats.produced.load()) + " consumed=" +
          std::to_string(consumer_stats.consumed.load()) + " planned=" +
          std::to_string(consumer_stats.planned.load()) + " dropped=" +
          std::to_string(queue.dropped_pushes()) + " foxglove_ws=" +
          (options.foxglove_enabled
               ? ("ws://" + options.foxglove_host + ":" + std::to_string(options.foxglove_port))
               : std::string("disabled")));
  return 0;
}
