#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <memory>
#include <vector>

#include "PlannerResult.h"
#include "RobotConfig.h"

class Node {
 public:
  int x, y;
  int parent_idx;
  Node(int x, int y) {
    this->x = x;
    this->y = y;
    this->parent_idx = -1;
  }
  friend std::ostream& operator<<(std::ostream& os, const Node& node) {
    os << node.x << " " << node.y;
    return os;
  }
};

class RRT {
  typedef std::unique_ptr<Node> Nptr;

 public:
  typedef std::pair<RobotConfig, int> MNode;
  typedef std::vector<MNode> RRTMetaData;
  Nptr root{nullptr};
  Nptr goal{nullptr};

  std::vector<Nptr> nodes;

  RRT(float start_x, float start_y, float end_x, float end_y, float grid_x_max,
      float grid_y_max);

  RRT(RobotConfig start, RobotConfig end, float grid_x_max, float grid_y_max);

  void runRRT();

  void print_nodes() const;

  void print_nodes2() const;

  std::vector<RobotConfig> returnPath() const;

  void printPath() const;

  RRTMetaData getMetaData() const;

 private:
  float grid_x_max{-1};
  float grid_y_max{-1};
  // TODO make static constexpr and fix pybindings
  const float step_size{80};
  const float max_itr{100};
  const float goal_threshold_dist{1};
  Nptr sample() const;
  std::pair<std::size_t, float> nearest_node(const Nptr& sample) const;
  float distanceToGoal(const Nptr& sample) const;
  void gotoNode(const Nptr& nearest_node, Nptr& sample, float dist) const;
  void add_edge(const int parent_idx, Nptr& child) const;
  bool isValid(const Nptr& node) const;
};

PlannerResult<RRT::RRTMetaData> planPath(float x_start, float y_start,
                                         float x_end, float y_end,
                                         float grid_x_max, float grid_y_max);

#endif