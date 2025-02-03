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
  std::shared_ptr<Node> parent;
  Node(int x, int y) {
    this->x = x;
    this->y = y;
    this->parent = nullptr;
  }
  friend std::ostream& operator<<(std::ostream& os, const Node& node) {
    os << node.x << " " << node.y;
    return os;
  }
};

class RRT {
  typedef std::shared_ptr<Node> Nptr;

 public:
  typedef std::pair<RobotConfig, int> MNode;
  typedef std::vector<MNode> RRTMetaData;
  // TODO make const (also functions)
  Nptr root{nullptr};
  Nptr goal{nullptr};
  float grid_x_max = -1;
  float grid_y_max = -1;
  float step_size = 80;
  float max_itr = 100;
  float goal_threshold_dist = 1;
  std::vector<Nptr> nodes;

  RRT(float start_x, float start_y, float end_x, float end_y, float grid_x_max,
      float grid_y_max);

  RRT(RobotConfig start, RobotConfig end, float grid_x_max, float grid_y_max);

  ~RRT();

  void runRRT();

  Nptr sample();

  bool isValid(const Nptr node);

  std::pair<std::size_t, float> nearest_node(const Nptr sample);

  float distanceToGoal(const Nptr sample);

  void gotoNode(const Nptr nearest_node, Nptr sample, float dist);

  void add_edge(Nptr parent, Nptr child);

  void print_nodes();

  void print_nodes2();

  std::vector<RobotConfig> returnPath();

  void printPath();

  RRTMetaData getMetaData();
};

PlannerResult<RRT::RRTMetaData> planPath(float x_start, float y_start,
                                         float x_end, float y_end,
                                         float grid_x_max, float grid_y_max);

#endif