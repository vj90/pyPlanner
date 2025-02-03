#ifndef PLANNER_RESULT_H
#define PLANNER_RESULT_H

#include <string>
#include <vector>

#include "RobotConfig.h"
template <typename T>
struct MetaData {
  std::string AlgorithmName{"RRT"};
  float runtime{-1};  // -1 implies not set
  // TODO make this a template parameter
  T data;
};

template <typename T>
struct PlannerResult {
  MetaData<T> metadata;
  std::vector<RobotConfig> path;
};

#endif  // PLANNER_RESULT_H