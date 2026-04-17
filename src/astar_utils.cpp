#include "astar_utils.h"
#include "planner.h"

bool findPath(Agent curr_agent, const std::vector<std::vector<int>> &grid,
              const std::vector<Constraint> &agent_constraints,
              Path &out_path) {
  std::unordered_set<AStarNode> closed;
  Location &start = curr_agent.start;
  Location &goal = curr_agent.goal;
  return false;
}
