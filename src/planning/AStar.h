#ifndef A_STAR_H
#define A_STAR_H

#include <memory>
#include <ostream>
#include <sstream>
#include <vector>

#include "Node.h"
#include "Obstacle.h"
#include "PlannerResult.h"
#include "RobotConfig.h"

class NodeWithCost : public Node {
 public:
  float cost_to_come{INFINITY};
  float cost_to_go{INFINITY};
  float cost() const { return cost_to_come + cost_to_go; }
  friend std::ostream& operator<<(std::ostream& os, const NodeWithCost& node) {
    os << "x: " << node.x << " y: " << node.y
       << " cost_to_come: " << node.cost_to_come
       << " cost_to_go: " << node.cost_to_go << " cost: " << node.cost()
       << "parent_idx: " << node.parent_idx << std::endl;
    return os;
  }
};

class AStar {
 public:
  AStar(const RobotConfig& start, const RobotConfig& end,
        const float grid_x_max, const float grid_y_max);
  void runAStar();
  NodeWithCost goal_node;
  NodeWithCost root_node;
  // void print_nodes() const;
  std::vector<RobotConfig> returnPath() const;
  // void printPath() const;
  // TODO extract out to base class
  void add_obstacle(std::unique_ptr<Obstacle> obstacle);

  using AStarMetaData = int;

 private:
  // std::vector<std::unique_ptr<Node>> nodes;
  // std::unique_ptr<Node> root;
  // std::unique_ptr<Node> goal;
  const float grid_x_max{NAN};
  float grid_descretization_step_x{NAN};
  const float grid_y_max{NAN};
  float grid_descretization_step_y{NAN};
  static constexpr float goal_threshold_dist{1};
  static constexpr float grid_resolution_x{20};
  static constexpr float grid_resolution_y{20};

  std::vector<NodeWithCost> closed_list;
  NodeWithCost findClosestGraphNode(const Node& node) const;
  std::vector<NodeWithCost> getNeighbors(const NodeWithCost& node) const;
  void updateCostToCome(NodeWithCost& child, const NodeWithCost& parent) const;
  void updateCostToGo(NodeWithCost& node) const;
  void updateCost(NodeWithCost& node, const NodeWithCost& parent) const;

  std::vector<std::unique_ptr<Obstacle>> obstacle_list;
  std::pair<bool, int> nodeInList(const NodeWithCost& node,
                                  const std::vector<NodeWithCost>& list) const;
  // Function to snap a point to the nearest grid point
  std::pair<float, float> snapToGrid(float x, float y) const;
  void sortList(std::vector<NodeWithCost>& list) const;
};

std::vector<RobotConfig> planPathAStar(RobotConfig start, RobotConfig end,
                                       float grid_x_max, float grid_y_max,
                                       types::pyobstacle& circular_obstacles);
#endif  // A_STAR_H