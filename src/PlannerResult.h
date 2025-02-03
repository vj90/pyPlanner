#include <string>
#include <vector>

#include "RobotConfig.h"
struct MetaData {
  std::string AlgorithmName;
  float runtime;
};

struct PlannerResult {
  // TODO add metadata
  std::vector<RobotConfig> path;
};