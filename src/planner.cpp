#include "planner.h"
// #include <algorithm>
#include "astar_utils.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <unordered_map>

void CBSPlanner::createConstraintsFromConflict(const Conflict &conflict,
                                               Constraint &c1, Constraint &c2) {
  // Both constraints share the same time
  c1.time = conflict.time;
  c2.time = conflict.time;

  c1.agent_id = conflict.agent1;
  c2.agent_id = conflict.agent2;

  /* Vertex Cosntrain Handling */
  if (!conflict.loc2.has_value()) {
    c1.loc1 = conflict.loc1;
    c1.loc2 = std::nullopt;

    c2.loc1 = conflict.loc1;
    c2.loc2 = std::nullopt;
  } else {
    /* Edge Conflict handing: agent 1 constraint */
    c1.loc1 = conflict.loc1;
    c1.loc2 = conflict.loc2.value();

    /* Agent 2 constraint */
    c2.loc1 = conflict.loc2.value();
    c2.loc2 = conflict.loc1;
  }
}

bool CBSPlanner::getFirstConflict(const std::unordered_map<int, Path> &paths,
                                  Conflict &out_conflict) {
  // Extract just the agent IDs to make pairing them up easier
  std::vector<int> agent_ids;
  for (const auto &pair : paths) {
    agent_ids.push_back(pair.first);
  }

  // Compare every agent against every other agent O(N^2)
  for (size_t i = 0; i < agent_ids.size(); ++i) {
    for (size_t j = i + 1; j < agent_ids.size(); ++j) {
      int a1 = agent_ids[i];
      int a2 = agent_ids[j];
      const Path &p1 = paths.at(a1);
      const Path &p2 = paths.at(a2);

      // We must check up to the end of the longest path
      int max_time = std::max(p1.size(), p2.size());

      for (int t = 0; t < max_time; ++t) {
        /* IMPORTANT: Goal Persistence
         *  agents rest at their goal point when complete
         */
        Location loc1 = (t < static_cast<int>(p1.size())) ? p1[t] : p1.back();
        Location loc2 = (t < static_cast<int>(p2.size())) ? p2[t] : p2.back();

        /* Vertex Conflict */
        if (loc1 == loc2) {
          out_conflict.agent1 = a1;
          out_conflict.agent2 = a2;
          out_conflict.time = t;
          out_conflict.loc1 = loc1;
          out_conflict.loc2 = std::nullopt;
          return true;
        }

        /* Edge Conflict */
        if (t > 0) { // Edges only exist for t > 0
          Location prev1 =
              (t - 1 < static_cast<int>(p1.size())) ? p1[t - 1] : p1.back();
          Location prev2 =
              (t - 1 < static_cast<int>(p2.size())) ? p2[t - 1] : p2.back();

          if (prev1 == loc2 && loc1 == prev2) {
            out_conflict.agent1 = a1;
            out_conflict.agent2 = a2;
            out_conflict.time = t;
            // For edge conflicts, we record the direction Agent 1 was moving
            out_conflict.loc1 = prev1;
            out_conflict.loc2 = loc1;
            return true;
          }
        }
      }
    }
  }

  /* Safe */
  return false;
}

int CBSPlanner::calculateCost(const std::unordered_map<int, Path> &paths) {
  int total_cost = 0;
  for (const auto &[agent_id, path] : paths) {
    if (!path.empty()) {
      // Cost is the number of transitions (length - 1)
      total_cost += (path.size() - 1);
    }
  }
  return total_cost;
}

std::unordered_map<int, Path> CBSPlanner::plan() {
  int nodes_generated = 1;
  int nodes_expanded = 0;
  int max_depth = 0;

  int next_node_id = 0;
  std::ofstream dot_file("cbs_tree.dot");
  dot_file << "digraph CBSTree {\n";
  dot_file
      << "  node [shape=box, fontname=\"Arial\"];\n"; // Make nodes look nice

  /* Initialize CT */
  std::priority_queue<CTNode, std::vector<CTNode>, std::greater<CTNode>>
      open_list;

  CTNode root;

  /* Find initail uncoordinated paths */
  for (const auto &agent : agents) {
    Path initial_path;
    bool success = runSpaceTimeAStar(agent.id, root.constraints, initial_path);
    if (!success) {
      /* unsolvable */
      return {};
    }
    root.paths[agent.id] = initial_path;
  }

  root.cost = calculateCost(root.paths);
  root.depth = 0;
  root.id = next_node_id++;
  root.latest_constraint = "Root Node";
  dot_file << "  node" << root.id << " [label=\"ID: " << root.id
           << "\\nCost: " << root.cost << "\\n"
           << root.latest_constraint << "\"];\n";
  open_list.push(root);

  /* main loop */
  while (!open_list.empty()) {
    // Pop the cheapest node
    CTNode current = open_list.top();
    open_list.pop();
    nodes_expanded++;
    if (nodes_expanded % 100 == 0) {
      std::cout << "Nodes Expanded: " << nodes_expanded
                << " | Current Cost: " << current.cost << std::endl;
    }
    if (current.depth > max_depth) {
      max_depth = current.depth;
    }
    // Check for collisions
    Conflict conflict;
    bool has_conflict = getFirstConflict(current.paths, conflict);

    /* this is the optimal solution */
    if (!has_conflict) {
      // Color the winning node green in the tree!
      dot_file << "  node" << current.id
               << " [style=filled, fillcolor=palegreen];\n";
      dot_file << "}\n"; // Close the graph
      dot_file.close();  // Save the file
      // SUCCESS! WE FOUND A VALID PATH!
      std::cout << "--- CBS Tree Statistics ---\n";
      std::cout << "Nodes Generated : " << nodes_generated << "\n";
      std::cout << "Nodes Expanded  : " << nodes_expanded << "\n";
      std::cout << "Max Tree Depth  : " << max_depth << "\n";
      return current.paths;
    }

    /* branch the tree */
    Constraint c1, c2;
    createConstraintsFromConflict(conflict, c1, c2);

    /* Left child has agent 1 constraint */
    CTNode left_child = current;          // Copy parent's state
    left_child.constraints.push_back(c1); // Add the new rule
    left_child.depth = current.depth + 1;
    left_child.id = next_node_id++;
    // Create a label like "A1 avoids (5,5) @ t=4"
    left_child.latest_constraint = "A" + std::to_string(c1.agent_id) +
                                   " avoids (" + std::to_string(c1.loc1.x) +
                                   "," + std::to_string(c1.loc1.y) +
                                   ") @ t=" + std::to_string(c1.time);

    Path new_path_1;
    // Re-plan ONLY the agent that got the new constraint
    if (runSpaceTimeAStar(conflict.agent1, left_child.constraints,
                          new_path_1)) {
      left_child.paths[conflict.agent1] = new_path_1;
      left_child.cost = calculateCost(left_child.paths);
      open_list.push(left_child);
      nodes_generated++;
      dot_file << "  node" << left_child.id << " [label=\"ID: " << left_child.id
               << "\\nCost: " << left_child.cost << "\\n"
               << left_child.latest_constraint << "\"];\n";
      dot_file << "  node" << current.id << " -> node" << left_child.id
               << ";\n";
    }

    /* right child agent 2 gets cosntrained */
    CTNode right_child = current;          // Copy parent's state
    right_child.constraints.push_back(c2); // Add the new rule
    right_child.depth = current.depth + 1;
    right_child.id = next_node_id++;
    right_child.latest_constraint = "A" + std::to_string(c2.agent_id) +
                                    " avoids (" + std::to_string(c2.loc1.x) +
                                    "," + std::to_string(c2.loc1.y) +
                                    ") @ t=" + std::to_string(c2.time);

    Path new_path_2;
    // Re-plan ONLY the agent that got the new constraint
    if (runSpaceTimeAStar(conflict.agent2, right_child.constraints,
                          new_path_2)) {
      right_child.paths[conflict.agent2] = new_path_2;
      right_child.cost = calculateCost(right_child.paths);
      open_list.push(right_child);
      nodes_generated++;
      dot_file << "  node" << right_child.id
               << " [label=\"ID: " << right_child.id
               << "\\nCost: " << right_child.cost << "\\n"
               << right_child.latest_constraint << "\"];\n";
      dot_file << "  node" << current.id << " -> node" << right_child.id
               << ";\n";
    }
  }

  std::cout << "Falied. Open list empty, no solution found. \n";
  std::cout << "Nodes Expanded before failure: " << nodes_expanded << "\n";
  // If the open list runs out, no valid solution exists.
  dot_file << "}\n";
  dot_file.close();
  return {};
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
  Agent my_agent;
  my_agent = this->agents[agent_id];

  // --- PHASE 1: Start to Goal 1 ---
  Path path1;
  // Pass Start, Goal 1, and Time = 0
  if (!AStar::findPath(my_agent.start, my_agent.goal1, 0, grid_map,
                       my_constraints, path1)) {
    return false;
  }

  // --- PHASE 2: Goal 1 to Goal 2 ---
  Path path2;
  // Calculate when we arrived at Goal 1
  int arrival_time =
      path1.size() -
      1; // Assuming your locations have a time attached, or path1.size() - 1

  // Pass Goal 1, Goal 2, and Time = arrival_time
  if (!AStar::findPath(my_agent.goal1, my_agent.goal2, arrival_time, grid_map,
                       my_constraints, path2)) {
    return false;
  }

  // Combine them
  out_path = path1;
  out_path.insert(out_path.end(), path2.begin() + 1, path2.end());

  return true;
}
