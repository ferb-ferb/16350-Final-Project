#include "planner.h"
// #include <algorithm>
#include "astar_utils.h"
#include <cmath>
#include <unordered_map>

std::unordered_map<int, Path> CBSPlanner::plan() {
  std::unordered_map<int, Path> plans;

  return plans;
}

std::vector<Constraint> CBSPlanner::getConstraintsForAgent(
    int agent_id, const ::std::vector<Constraint> &all_constraints) {
  std::vector<Constraint> filtered_constraints;

  // Loop through the provided constraint list
  for (const auto &constraint : all_constraints) {
    // save constraints for this agent
    if (constraint.agent_id == agent_id) {
      filtered_constraints.push_back(constraint);
    }
  }

  return filtered_constraints;
}

bool CBSPlanner::runSpaceTimeAStar(int agent_id,
                                   const std::vector<Constraint> &constraints,
                                   Path &out_path) {
  auto my_constraints = getConstraintsForAgent(agent_id, constraints);
  return AStar::findPath(this->agents[agent_id], grid_map, my_constraints,
                         out_path);
}
