#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <memory>
#include <vector>

#include "Node.h"
#include "Obstacle.h"
#include "PlannerResult.h"
#include "RobotConfig.h"

namespace types {
using pyobstacle = std::vector<std::tuple<float, float, float>>;
}

class RRT {
  using Nptr = std::unique_ptr<Node>;

 public:
  using MNode = std::pair<RobotConfig, int>;
  using RRTMetaData = std::vector<MNode>;
  Nptr root{nullptr};
  Nptr goal{nullptr};

  std::vector<Nptr> nodes;

  RRT(float start_x, float start_y, float end_x, float end_y, float grid_x_max,
      float grid_y_max);

  RRT(RobotConfig& start, RobotConfig& end, float grid_x_max, float grid_y_max);

  void runRRT();

  void print_nodes() const;

  std::vector<RobotConfig> returnPath() const;

  void printPath() const;

  RRTMetaData getMetaData() const;

  void add_obstacle(std::unique_ptr<Obstacle> obstacle);

 private:
  float grid_x_max{-1};
  float grid_y_max{-1};
  // TODO make static constexpr and fix pybindings
  const float step_size{40};
  const float max_itr{1000};
  const float max_valid_itr{100};
  const float goal_threshold_dist{1};
  const int num_path_samples{10};
  std::vector<std::unique_ptr<Obstacle>> obstacle_list;

  Nptr sample() const;
  std::pair<std::size_t, float> nearest_node(const Nptr& sample) const;
  float distanceToGoal(const Nptr& sample) const;
  void gotoNode(const Nptr& nearest_node, Nptr& sample, float dist) const;
  void add_edge(const int parent_idx, Nptr& child) const;
  bool isValid(const Nptr& node) const;
  bool inCollision(const Nptr& node_start, const Nptr& node_end) const;
  std::vector<RobotConfig> sampleEdge(const Nptr& node_start,
                                      const Nptr& node_end) const;
};

PlannerResult<RRT::RRTMetaData> planPath(RobotConfig start, RobotConfig end,
                                         float grid_x_max, float grid_y_max,
                                         types::pyobstacle& circular_obstacles);

#endif