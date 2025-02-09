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

class GraphNode : public Node {
 public:
  float cost_to_come{INFINITY};
  float cost_to_go{INFINITY};
  float cost() const { return cost_to_come + cost_to_go; }
  friend std::ostream& operator<<(std::ostream& os, const GraphNode& node) {
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
  GraphNode goal_node;
  GraphNode root_node;
  // void printNodes() const;
  std::vector<RobotConfig> returnPath() const;
  // void printPath() const;
  // TODO extract out to base class
  void addObstacle(std::unique_ptr<Obstacle> obstacle);

  using AStarMetaData = int;

 private:
  // Private properties
  const float grid_x_max{NAN};
  float grid_descretization_step_x{NAN};
  const float grid_y_max{NAN};
  float grid_descretization_step_y{NAN};
  static constexpr float goal_threshold_dist{1};
  static constexpr float grid_resolution_x{20};
  static constexpr float grid_resolution_y{20};
  static constexpr float m_relative_collision_cost_{1000};
  std::vector<GraphNode> closed_list_;

  // Private methods
  ///@brief Find the closest node snapped to the graph to the given node
  /// @param node input node whose closest graph node is to be found
  GraphNode findClosestGraphNode(const Node& node) const;

  /// @brief Find the neighbors of a given graph node in the graph
  /// @param node input graph node whose neighbors are to be found
  /// @return
  std::vector<GraphNode> getNeighbors(const GraphNode& node) const;
  void updateCostToCome(GraphNode& child, const GraphNode& parent) const;
  void updateCostToGo(GraphNode& node) const;
  void updateCost(GraphNode& node, const GraphNode& parent) const;

  std::vector<std::unique_ptr<Obstacle>> obstacle_list;
  std::pair<bool, int> nodeInList(const GraphNode& node,
                                  const std::vector<GraphNode>& list) const;
  // Function to snap a point to the nearest grid point
  std::pair<float, float> snapToGrid(float x, float y) const;
  void sortList(std::vector<GraphNode>& list) const;
};

std::vector<RobotConfig> planPathAStar(RobotConfig start, RobotConfig end,
                                       float grid_x_max, float grid_y_max,
                                       types::pyobstacle& circular_obstacles);
#endif  // A_STAR_H