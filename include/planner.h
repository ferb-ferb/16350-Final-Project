#ifndef PLANNER_H
#define PLANNER_H

#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
/* ****************************** Basic ****************************** */
/* A 2d coordinate */
struct Location {
  int x;
  int y;

  bool operator==(const Location &other) const {
    return x == other.x && y == other.y;
  }
};

/* Robots stored as an id, start and goal */
struct Agent {
  int id;
  Location start;
  Location goal1;
  Location goal2;
};

/* Paths are a vector of locations, index inherently represents time */
using Path = std::vector<Location>;

/* ****************************************************************** */
/* *********************** Conflict Tree Stuff ************************* */

/* Constraint struct defines a rule that robot id cannot occupy a location
 * (vertex constr.) OR traverse an edge at a given time (edge constr.)
 */
struct Constraint {
  int agent_id;
  Location loc1;
  std::optional<Location>
      loc2; // used for edge constraints. use .has_value() to check
  // edge vs vertex constraint
  int time;

  bool operator==(const Constraint &other) const {
    return agent_id == other.agent_id && loc1 == other.loc1 &&
           loc2 == other.loc2 && time == other.time;
  }
};

/* Defines a conflict between two agents, also vertex OR edge, like above */
struct Conflict {
  int agent1;
  int agent2;
  Location loc1;
  std::optional<Location> loc2; // Used for edge conflicts
  int time;
};

/* Defines a Node of the Conflict Tree
 * An uncoordinated solution to the set of constraints defined within
 */
struct CTNode {
  std::vector<Constraint> constraints; // Accumulated set of constraints
  std::unordered_map<int, Path> paths; // plans mapped to agents
  int cost;                            // Cost
  int depth = 0;

  int id = 0; 
  std::string latest_constraint = "Root";
  
  /* Overloaded comparator for min heap sorting of conflict tree */
  bool operator>(const CTNode &other) const { return cost > other.cost; }
};
/* ****************************************************************** */
/* **************************** CBS Planner Class ************************ */

class CBSPlanner {
public:
  /* Constructor */
  CBSPlanner(const std::vector<std::vector<int>> &map,
             const std::vector<Agent> &agents)
      : grid_map(map), agents(agents) {}
  /* planning function */
  std::unordered_map<int, Path> plan();

private:
  const std::vector<std::vector<int>> &grid_map;
  const std::vector<Agent> &agents;
  /* ************************** HIGH LEVEL FUNCTIONS* ********************* */
  /* Scans a CBS Node for a conflict and returns true on first conflict found
   * Returns false if this is a valid node
   */
  bool getFirstConflict(const std::unordered_map<int, Path> &paths,
                        Conflict &out_conflict);

  /* Given a conflict, splits into robot A and robot B constraints */
  void createConstraintsFromConflict(const Conflict &conflict, Constraint &c1,
                                     Constraint &c2);

  /* Calculates the cost of the current node's path */
  int calculateCost(const std::unordered_map<int, Path> &paths);
  /* ********************************************************************** */
  /* ************************** LOW LEVEL FUNCTIONS *********************** */

  /* Whatever low level planner we choose to use. probably space time A* */
  /* Finds optimal path for one agent given constraints */
  bool runSpaceTimeAStar(int agent_id,
                         const std::vector<Constraint> &constraints,
                         Path &out_path);

  /* Function to find my constraints w/in constraint list */
  std::vector<Constraint>
  getConstraintsForAgent(int agent_id,
                         const std::vector<Constraint> &all_constraints);
  /* ********************************************************************** */
};
#endif
