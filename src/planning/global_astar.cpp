#include "rm_nav/planning/global_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace rm_nav::planning {
namespace {

struct Cell {
  int x{0};
  int y{0};
};

bool WorldToGrid(const data::GridMap2D& map, float x_m, float y_m, Cell* cell) {
  if (cell == nullptr || map.width == 0U || map.height == 0U || map.resolution_m <= 0.0F) {
    return false;
  }
  const float local_x = x_m - map.origin.position.x;
  const float local_y = y_m - map.origin.position.y;
  cell->x = static_cast<int>(std::floor(local_x / map.resolution_m));
  cell->y = static_cast<int>(std::floor(local_y / map.resolution_m));
  return cell->x >= 0 && cell->y >= 0 && cell->x < static_cast<int>(map.width) &&
         cell->y < static_cast<int>(map.height);
}

data::PathPoint2f GridToPathPoint(const data::GridMap2D& map, int x, int y) {
  data::PathPoint2f point;
  point.position.x = map.origin.position.x + (static_cast<float>(x) + 0.5F) * map.resolution_m;
  point.position.y = map.origin.position.y + (static_cast<float>(y) + 0.5F) * map.resolution_m;
  return point;
}

std::size_t Index(const data::GridMap2D& map, int x, int y) {
  return static_cast<std::size_t>(y) * map.width + static_cast<std::size_t>(x);
}

bool IsFree(const data::GridMap2D& map, int x, int y) {
  if (x < 0 || y < 0 || x >= static_cast<int>(map.width) || y >= static_cast<int>(map.height)) {
    return false;
  }
  return map.occupancy.empty() || map.occupancy[Index(map, x, y)] < 50U;
}

Cell FindNearestFree(const data::GridMap2D& map, Cell start) {
  if (IsFree(map, start.x, start.y)) {
    return start;
  }
  for (int radius = 1; radius < 12; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        Cell candidate{start.x + dx, start.y + dy};
        if (IsFree(map, candidate.x, candidate.y)) {
          return candidate;
        }
      }
    }
  }
  return start;
}

float Heuristic(const Cell& a, const Cell& b) {
  const float dx = static_cast<float>(a.x - b.x);
  const float dy = static_cast<float>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

common::Status GlobalAStar::Plan(const data::GridMap2D& static_map,
                                 const data::Pose3f& start_pose,
                                 const data::Pose3f& goal_pose,
                                 data::Path2D* path) const {
  if (path == nullptr) {
    return common::Status::InvalidArgument("global path output is null");
  }

  path->stamp = start_pose.stamp;
  path->points.clear();

  if (static_map.width == 0U || static_map.height == 0U || static_map.resolution_m <= 0.0F) {
    data::PathPoint2f start;
    start.position.x = start_pose.position.x;
    start.position.y = start_pose.position.y;
    data::PathPoint2f goal;
    goal.position.x = goal_pose.position.x;
    goal.position.y = goal_pose.position.y;
    path->points.push_back(start);
    path->points.push_back(goal);
    return common::Status::Ok();
  }

  Cell start_cell;
  Cell goal_cell;
  if (!WorldToGrid(static_map, start_pose.position.x, start_pose.position.y, &start_cell) ||
      !WorldToGrid(static_map, goal_pose.position.x, goal_pose.position.y, &goal_cell)) {
    return common::Status::InvalidArgument("planner start or goal falls outside static map");
  }
  start_cell = FindNearestFree(static_map, start_cell);
  goal_cell = FindNearestFree(static_map, goal_cell);

  struct QueueNode {
    float f_cost{0.0F};
    int x{0};
    int y{0};
    bool operator<(const QueueNode& other) const { return f_cost > other.f_cost; }
  };

  const std::size_t cell_count =
      static_cast<std::size_t>(static_map.width) * static_map.height;
  std::vector<float> g_cost(cell_count, std::numeric_limits<float>::max());
  std::vector<std::int32_t> parent(cell_count, -1);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<QueueNode> open;

  g_cost[Index(static_map, start_cell.x, start_cell.y)] = 0.0F;
  open.push({Heuristic(start_cell, goal_cell), start_cell.x, start_cell.y});

  constexpr std::array<Cell, 8> kNeighbors = {{
      {1, 0}, {-1, 0}, {0, 1},  {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
  }};

  bool found = false;
  while (!open.empty()) {
    const QueueNode node = open.top();
    open.pop();
    const std::size_t current_index = Index(static_map, node.x, node.y);
    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = true;
    if (node.x == goal_cell.x && node.y == goal_cell.y) {
      found = true;
      break;
    }

    for (const auto& offset : kNeighbors) {
      const int nx = node.x + offset.x;
      const int ny = node.y + offset.y;
      if (!IsFree(static_map, nx, ny)) {
        continue;
      }
      const std::size_t neighbor_index = Index(static_map, nx, ny);
      if (closed[neighbor_index]) {
        continue;
      }
      const float step_cost = (offset.x == 0 || offset.y == 0) ? 1.0F : 1.4142F;
      const float tentative = g_cost[current_index] + step_cost;
      if (tentative >= g_cost[neighbor_index]) {
        continue;
      }
      g_cost[neighbor_index] = tentative;
      parent[neighbor_index] = static_cast<std::int32_t>(current_index);
      open.push({tentative + Heuristic({nx, ny}, goal_cell), nx, ny});
    }
  }

  if (!found) {
    data::PathPoint2f start;
    start.position.x = start_pose.position.x;
    start.position.y = start_pose.position.y;
    data::PathPoint2f goal;
    goal.position.x = goal_pose.position.x;
    goal.position.y = goal_pose.position.y;
    path->points.push_back(start);
    path->points.push_back(goal);
    return common::Status::Ok();
  }

  std::vector<Cell> reversed_cells;
  std::size_t current = Index(static_map, goal_cell.x, goal_cell.y);
  const std::size_t start_index = Index(static_map, start_cell.x, start_cell.y);
  while (true) {
    const int x = static_cast<int>(current % static_map.width);
    const int y = static_cast<int>(current / static_map.width);
    reversed_cells.push_back({x, y});
    if (current == start_index) {
      break;
    }
    const std::int32_t prev = parent[current];
    if (prev < 0) {
      break;
    }
    current = static_cast<std::size_t>(prev);
  }

  std::reverse(reversed_cells.begin(), reversed_cells.end());
  path->points.reserve(reversed_cells.size() + 1U);
  for (const auto& cell : reversed_cells) {
    path->points.push_back(GridToPathPoint(static_map, cell.x, cell.y));
  }
  if (!path->points.empty()) {
    for (std::size_t index = 1; index < path->points.size(); ++index) {
      const float dx = path->points[index].position.x - path->points[index - 1U].position.x;
      const float dy = path->points[index].position.y - path->points[index - 1U].position.y;
      path->points[index - 1U].heading_rad = std::atan2(dy, dx);
      path->points[index - 1U].target_speed_mps = 0.8F;
    }
    path->points.back().heading_rad = goal_pose.rpy.z;
    path->points.back().target_speed_mps = 0.0F;
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::planning
