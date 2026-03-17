#include "rm_nav/mapping/waypoint_manager.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::mapping {
namespace {

std::string Trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1U);
}

data::Pose3f MakePose(float x, float y, float yaw) {
  data::Pose3f pose;
  pose.reference_frame = tf::kMapFrame;
  pose.child_frame = tf::kBaseLinkFrame;
  pose.position.x = x;
  pose.position.y = y;
  pose.rpy.z = yaw;
  pose.is_valid = true;
  return pose;
}

}  // namespace

common::Status WaypointManager::Load(const std::string& path) {
  std::ifstream input(path);
  if (!input.is_open()) {
    return common::Status::Unavailable("failed to open waypoint file");
  }

  waypoints_.clear();
  current_index_ = 0;
  std::string line;
  while (std::getline(input, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    line = Trim(line);
    if (line.empty()) {
      continue;
    }
    for (char& ch : line) {
      if (ch == ',') {
        ch = ' ';
      }
    }
    std::istringstream stream(line);
    float x = 0.0F;
    float y = 0.0F;
    float yaw = 0.0F;
    if (!(stream >> x >> y)) {
      return common::Status::InvalidArgument("invalid waypoint line");
    }
    stream >> yaw;
    waypoints_.push_back(MakePose(x, y, yaw));
  }

  return waypoints_.empty() ? common::Status::InvalidArgument("waypoint file is empty")
                            : common::Status::Ok();
}

void WaypointManager::Reset() { current_index_ = 0; }

data::Pose3f WaypointManager::CurrentGoal() const {
  if (waypoints_.empty()) {
    return {};
  }
  return waypoints_[current_index_ % waypoints_.size()];
}

bool WaypointManager::AdvanceIfReached(const data::Pose3f& current_pose,
                                       float reach_tolerance_m) {
  if (waypoints_.empty() || !current_pose.is_valid) {
    return false;
  }
  const auto goal = CurrentGoal();
  const float dx = goal.position.x - current_pose.position.x;
  const float dy = goal.position.y - current_pose.position.y;
  if (std::sqrt(dx * dx + dy * dy) > reach_tolerance_m) {
    return false;
  }
  current_index_ = (current_index_ + 1U) % waypoints_.size();
  return true;
}

}  // namespace rm_nav::mapping
