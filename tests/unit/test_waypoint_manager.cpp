#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "rm_nav/mapping/waypoint_manager.hpp"

int main() {
  const std::filesystem::path path("logs/test_waypoint_manager_waypoints.txt");
  std::filesystem::create_directories(path.parent_path());
  {
    std::ofstream output(path);
    output << "1.0, 2.0, 0.0\n";
    output << "2.5, 3.0, 1.57\n";
  }

  rm_nav::mapping::WaypointManager manager;
  assert(manager.Load(path.string()).ok());
  assert(manager.waypoint_count() == 2U);
  auto goal = manager.CurrentGoal();
  assert(goal.is_valid);
  assert(goal.position.x == 1.0F);
  assert(goal.position.y == 2.0F);
  assert(manager.AdvanceIfReached(goal, 0.1F));
  goal = manager.CurrentGoal();
  assert(goal.position.x == 2.5F);
  assert(goal.position.y == 3.0F);

  std::cout << "test_waypoint_manager passed\n";
  return 0;
}
