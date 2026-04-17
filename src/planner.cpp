#include "planner.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>

std::unordered_map<int, Path> CBSPlanner::plan() {
  std::unordered_map<int, Path> plans;

  return plans;
}

bool CBSPlanner::runSpaceTimeAStar(int agent_id,
                                   const std::vector<Constraint> &constraints,
                                   Path &out_path) {
  Location start = this->agents[agent_id].start;
  Location goal = this->agents[agent_id].goal;
  return true;
}
