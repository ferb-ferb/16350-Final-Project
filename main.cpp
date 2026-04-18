#include "astar_utils.h"
#include "planner.h"
#include <iostream>
#include <vector>

// // Helper function to print the path nicely
// void printPath(const Path &path) {
//   if (path.empty()) {
//     std::cout << "  [Result] No path found!\n";
//     return;
//   }
//
//   std::cout << "  [Result] Path found (Length: " << path.size() - 1
//             << " steps):\n";
//   int time = 0;
//   for (const auto &loc : path) {
//     std::cout << "    Time " << time++ << ": (" << loc.x << ", " << loc.y
//               << ")\n";
//   }
// }
//
// int main() {
//   std::vector<std::vector<int>> grid = {
//       {0, 0, 0, 0}, {0, 1, 1, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}};
//
//   Agent A1 = {0, (0), (0), (2), (2)};
//   std::vector<Constraint> constraints;
//   Path my_path;
//
//   /* No Constraints basic A* test */
//   std::cout << "--- Test 1: Basic A* (No Constraints) ---\n";
//   bool success = AStar::findPath(A1, grid, constraints, my_path);
//   printPath(my_path);
//
//   /* A* with a vertex constraint */
//   std::cout << "\n--- Test 2: Space-Time A* (With Vertex Constraint) ---\n";
//
//   /* pretend another agent is in 1,0 at time 1 */
//   Constraint c1;
//   c1.agent_id = 0;
//   c1.loc1 = {1, 0};
//   c1.time = 1;
//   c1.loc2 = std::nullopt; // nullopt means this is a Vertex Constraint
//
//   constraints.push_back(c1);
//
//   Path constrained_path;
//   success = AStar::findPath(A1, grid, constraints, constrained_path);
//   printPath(constrained_path);
//
//   return 0;
// }

// Helper function to print the path nicely
void printPath(int agent_id, const Path &path) {
  if (path.empty()) {
    std::cout << "  [Agent " << agent_id << "] No path found!\n";
    return;
  }

  std::cout << "  [Agent " << agent_id
            << "] Path found (Length: " << path.size() - 1 << " steps):\n";
  int time = 0;
  for (const auto &loc : path) {
    std::cout << "    Time " << time++ << ": (" << loc.x << ", " << loc.y
              << ")\n";
  }
}

int main() {
  // 1. Create a 5x5 open grid (no walls needed to force this collision!)
  std::vector<std::vector<int>> grid = {{0, 0, 0, 0, 0},
                                        {0, 0, 0, 0, 0},
                                        {0, 0, 0, 0, 0},
                                        {0, 0, 0, 0, 0},
                                        {0, 0, 0, 0, 0}};

  // 2. Setup the Agents
  // Agent 0 goes Left to Right (y=2)
  Agent A0 = {0, {0, 2}, {4, 2}};
  // Agent 1 goes Top to Bottom (x=2)
  Agent A1 = {1, {2, 0}, {2, 4}};

  std::vector<Agent> agents = {A0, A1};

  // 3. Initialize the Planner
  // (Assuming you have a constructor like: CBSPlanner(grid, agents))
  CBSPlanner planner(grid, agents);

  // 4. Run the High-Level Search
  std::cout << "--- Running CBS Multi-Agent Planner ---\n";
  auto final_paths = planner.plan();

  // 5. Print the Results
  if (final_paths.empty()) {
    std::cout << "Planner failed to find a valid solution!\n";
  } else {
    printPath(0, final_paths[0]);
    std::cout << "\n";
    printPath(1, final_paths[1]);
  }

  return 0;
}
