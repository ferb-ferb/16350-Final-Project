/*=================================================================
 *
 * runtest.cpp
 *
 *=================================================================*/

/*
Form of text file
W
200,200
C
100
N
3
S
25,100 #,# #,#
G
#,# #,# #,#
M
map
*/
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "planner.h"

int main(int argc, char *argv[]) {
  // READ PROBLEM
  if (argc != 2) {
    std::cout << "runtest takes exactly one command line argument: the map file"
              << std::endl;
    return -1;
  }

  std::cout << "Reading problem definition from: " << argv[1] << std::endl;

  std::ifstream myfile;
  myfile.open(argv[1]);
  if (!myfile.is_open()) {
    std::cout << "Failed to open the file." << std::endl;
    return -1;
  }

  // read map size
  char letter;
  std::string line;
  int x_size, y_size;

  myfile >> letter;
  if (letter != 'W') {
    std::cout << "error parsing file" << std::endl;
    return -1;
  }

  myfile >> x_size >> letter >> y_size;
  std::cout << "map size: " << x_size << letter << y_size << std::endl;

  // read collision threshold
  int collision_thresh;
  myfile >> letter;
  if (letter != 'C') {
    std::cout << "error parsing file" << std::endl;
    return -1;
  }

  myfile >> collision_thresh;
  std::cout << "collision threshold: " << collision_thresh << std::endl;

  // get number of robots
  int num_robots;
  myfile >> letter;
  if (letter != 'N') {
    std::cout << "error parsing file" << std::endl;
    return -1;
  }

  myfile >> num_robots;
  std::cout << "Number of robots: " << num_robots << std::endl;

  // read start positions
  int helperX, helperY;
  std::vector<Location> starts;
  myfile >> letter;
  if (letter != 'S') {
    std::cout << "error parsing file" << std::endl;
    return -1;
  }
  for (int i = 0; i < num_robots; i++) {
    myfile >> helperX >> letter >> helperY;
    starts.push_back(Location({helperX, helperY}));
  }

  // read goal positions
  std::vector<Location> goals;
  myfile >> letter;
  if (letter != 'G') {
    std::cout << "Missing G (Goal 1) in map file!" << std::endl;
    return -1;
  }
  std::vector<Location> goals1;
  for (int i = 0; i < num_robots; i++) {
    std::getline(myfile, line, ',');
    int helperX = std::stoi(line);
    myfile >> helperY;
    goals1.push_back({helperX, helperY});
  }

  // --- READ GOAL 2 (Dropoff) ---
  myfile >> letter;
  if (letter != 'D') {
    std::cout << "Missing D (Goal 2 / Dropoff) in map file!" << std::endl;
    return -1;
  }
  std::vector<Location> goals2;
  for (int i = 0; i < num_robots; i++) {
    std::getline(myfile, line, ',');
    int helperX = std::stoi(line);
    myfile >> helperY;
    goals2.push_back({helperX, helperY});
  }

  // --- MAKE AGENTS ---
  std::vector<Agent> agents;
  for (int i = 0; i < num_robots; i++) {
    // Now passing both goals!
    agents.push_back({i, starts[i], goals1[i], goals2[i]});
  }

  // read map
  // std::vector<std::vector<int>> map;
  // for (size_t i=0; i<x_size; i++)
  // {
  //     std::getline(myfile, line);
  //     std::stringstream ss(line);
  //     for (size_t j=0; j<y_size; j++)
  //     {
  //         double value;
  //         ss >> value;
  //
  //         map[j*x_size+i] = (int) value;
  //         if (j != y_size-1) ss.ignore();
  //     }
  // }
  std::string lineo;
  while (std::getline(myfile, lineo)) {
    if (!lineo.empty() && lineo.back() == '\r')
      lineo.pop_back(); // Handle Windows formatting
    if (lineo == "M")
      break; // Stop exactly when we hit the 'M'
  }
  // --- CORRECTED MAP READING ---
  // Initialize a 2D vector filled with 0s
  std::vector<std::vector<int>> grid_map(y_size, std::vector<int>(x_size, 0));

  // Read the map row by row (y) and column by column (x)
  for (int y = 0; y < y_size; y++) {
    std::getline(myfile, line);
    // Skip empty lines in case of weird Windows/Linux line endings
    while (line.empty() && myfile.good())
      std::getline(myfile, line);

    std::stringstream ss(line);
    for (int x = 0; x < x_size; x++) {
      double value;
      ss >> value;
      // Notice we use 2D indexing [y][x] here!
      grid_map[y][x] = (int)value;
      if (x != x_size - 1)
        ss.ignore(); // ignore the comma
    }
  }

  myfile.close();
  std::cout << "\nRunning planner" << std::endl;

  // CONTROL LOOP
  std::ofstream output_file("robot_trajectory.txt");
  if (!output_file.is_open()) {
    std::cerr << "Failed to open the file." << std::endl;
    return 1;
  }
  CBSPlanner planner(grid_map, agents);
  std::unordered_map<int, Path> plan = planner.plan();

  int robotposeX, robotposeY;
  Path curr_path;
  for (int i = 0; i < num_robots; i++) {

    curr_path = plan[i];
    int curr_step = 0;
    for (Location curr_loc : curr_path) {
      robotposeX = curr_loc.x;
      robotposeY = curr_loc.y;
      output_file << i << "," << curr_step << "," << robotposeX << ","
                  << robotposeY << std::endl;
      curr_step += 1;
    }
    printf("\n");
  }

  output_file.close();
  return 0;
}
