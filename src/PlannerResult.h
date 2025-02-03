#ifndef PLANNER_RESULT_H
#define PLANNER_RESULT_H

#include <string>
#include <vector>

#include "RobotConfig.h"
struct MetaData {
  std::string AlgorithmName{"RRT"};
  float runtime{-1};  // -1 implies not set
  // TODO make this a template parameter
  std::vector<RobotConfig> all_nodes;
};

struct PlannerResult {
  MetaData metadata;
  std::vector<RobotConfig> path;
};

#endif  // PLANNER_RESULT_H