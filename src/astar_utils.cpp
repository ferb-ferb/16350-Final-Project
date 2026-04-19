#include "astar_utils.h"
#include "planner.h"
#include <algorithm>

bool AStar::findPath(Location start_loc, Location goal_loc, int start_time,
                     const std::vector<std::vector<int>> &grid,
                     const std::vector<Constraint> &agent_constraints,
                     Path &out_path) {
  /* Make the A* open list */
  std::priority_queue<std::shared_ptr<AStarNode>,
                      std::vector<std::shared_ptr<AStarNode>>,
                      AStar::CompareNode>
      open_list;
  /* The closed set */
  std::unordered_set<AStarNode> closed;

  auto start_node = std::make_shared<AStarNode>(
      start_loc, start_time, 0, AStar::manhattan(start_loc, goal_loc));
  open_list.push(start_node);

  // N, S, W, E, and WAIT
  std::vector<std::pair<int, int>> directions = {
      {0, 1}, {0, -1}, {-1, 0}, {1, 0}, {0, 0}};
  int max_time = grid.size() * grid[0].size(); // prevent infinite loops

  while (!open_list.empty()) {
    auto current = open_list.top();
    open_list.pop();

    if (closed.find(*current) != closed.end())
      continue;
    closed.insert(*current);

    if (current->time > max_time)
      continue;

    // goal reached
    if (current->loc == goal_loc) {
      out_path.clear();
      auto curr_ptr = current;
      while (curr_ptr != nullptr) {
        out_path.push_back(curr_ptr->loc);
        curr_ptr = curr_ptr->parent;
      }
      std::reverse(out_path.begin(), out_path.end());
      return true;
    }

    // expand neighbors
    int next_time = current->time + 1;

    for (const auto &dir : directions) {
      Location next_loc = {current->loc.x + dir.first,
                           current->loc.y + dir.second};

      // bounds check
      if (next_loc.x < 0 || next_loc.x >= static_cast<int>(grid[0].size()) ||
          next_loc.y < 0 || next_loc.y >= static_cast<int>(grid.size()))
        continue;

      // check static obstacles
      if (grid[next_loc.y][next_loc.x] == 1)
        continue;

      // check constraints
      bool is_constrained = false;
      for (const auto &c : agent_constraints) {
        if (c.time != next_time)
          continue;

        if (!c.loc2.has_value()) {
          // Vertex
          if (c.loc1 == next_loc)
            is_constrained = true;
        } else {
          // Edge
          if (c.loc1 == current->loc && c.loc2.value() == next_loc)
            is_constrained = true;
        }
      }
      if (is_constrained)
        continue;

      // generate and add neighbor
      int next_g = current->g + 1;
      auto neighbor = std::make_shared<AStarNode>(
          next_loc, next_time, next_g, AStar::manhattan(next_loc, goal_loc),
          current);

      if (closed.find(*neighbor) == closed.end()) {
        open_list.push(neighbor);
      }
    }
  }
  return false;
}
