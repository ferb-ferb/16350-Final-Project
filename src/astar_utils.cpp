#include "astar_utils.h"
#include "planner.h"

bool findPath(Agent curr_agent, const std::vector<std::vector<int>> &grid,
              const std::vector<Constraint> &agent_constraints,
              Path &out_path) {
  /* Make the A* open list */
  std::priority_queue<std::shared_ptr<AStarNode>,
                      std::vector<std::shared_ptr<AStarNode>>,
                      AStar::CompareNode>
      open_list;
  /* The closed set */
  std::unordered_set<AStarNode> closed;

  Location &start = curr_agent.start;
  Location &goal = curr_agent.goal;
  return false;
}
