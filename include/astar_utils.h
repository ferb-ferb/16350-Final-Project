#ifndef ASTAR_UTILS_H
#define ASTAR_UTILS_H

#include "planner.h"
#include <cmath>

struct AStarNode {
  Location loc;
  int time;
  int g, h, f;
  std::shared_ptr<AStarNode> parent;

  AStarNode(Location l, int t, int g_val, int h_val,
            std::shared_ptr<AStarNode> p = nullptr)
      : loc(l), time(t), g(g_val), h(h_val), f(g_val + h_val), parent(p) {}

  // For the priority queue (Min-Heap)
  bool operator>(const AStarNode &other) const { return f > other.f; }

  // For the unordered_set (Closed List)
  bool operator==(const AStarNode &other) const {
    return loc == other.loc && time == other.time;
  }
};

namespace std {
template <> struct hash<AStarNode> {
  size_t operator()(const AStarNode &n) const {
    return hash<int>()(n.loc.x) ^ (hash<int>()(n.loc.y) << 1) ^
           (hash<int>()(n.time) << 2);
  }
};
} // namespace std

namespace AStar {

bool findPath(Agent curr_agent, const std::vector<std::vector<int>> &grid,
              const std::vector<Constraint> &agent_constraints, Path &out_path);

inline int manhattan(const Location &a, const Location &b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

struct CompareNode {
  bool operator()(const std::shared_ptr<AStarNode> a,
                  const std::shared_ptr<AStarNode> b) {
    return *a > *b;
  }
};
} // namespace AStar

#endif
