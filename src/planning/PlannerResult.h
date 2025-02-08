#ifndef PLANNER_RESULT_H
#define PLANNER_RESULT_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "RobotConfig.h"

struct AlgorithmData {
  std::string AlgorithmName{"RRT"};
  float runtime{-1};  // -1 implies not set
  std::string toStream() const {
    std::ostringstream stream;
    stream << "AlgorithmName = " << AlgorithmName << ", runtime = " << runtime;
    return stream.str();
  }
};

template <typename T>
struct MetaData {
  AlgorithmData algorithm_data;
  T data;
  std::string toStream() const {
    std::ostringstream stream;
    stream << algorithm_data.toStream();
    if (data.size() > 0) {
      stream << ", Data: available";
    } else {
      stream << ", Data: not available";
    }
    return stream.str();
  }
};

template <typename T>
struct PlannerResult {
  MetaData<T> metadata;
  std::vector<RobotConfig> path;
};

#endif  // PLANNER_RESULT_H