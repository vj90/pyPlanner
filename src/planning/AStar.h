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
  // Public methods
  ///@brief Constructor
  ///@param start Starting configuration of the robot
  ///@param end Ending configuration of the robot
  ///@param grid_x_max Maximum x coordinate of the grid
  ///@param grid_y_max Maximum y coordinate of the grid
  ///@note Minimum x and y coordinates are 0
  AStar(const RobotConfig& start, const RobotConfig& end,
        const float grid_x_max, const float grid_y_max);

  ///@brief Runs the A* algorithm
  void runAStar();

  ///@brief Returns the path from the start to the end
  ///@return Vector of RobotConfig objects representing the path
  std::vector<RobotConfig> returnPath() const;

  // TODO extract out to base class
  ///@brief Adds an obstacle to the environment
  void addObstacle(std::unique_ptr<Obstacle> obstacle);

  /// TODO dummy metadata type
  using AStarMetaData = int;

 private:
  // Private properties
  GraphNode m_goal_node_;
  GraphNode m_root_node_;
  const float m_grid_x_max_{NAN};
  float m_grid_descretization_step_x_{NAN};
  const float m_grid_y_max_{NAN};
  float m_grid_descretization_step_y_{NAN};
  static constexpr float m_goal_threshold_dist_{1};
  static constexpr float m_grid_resolution_x_{20};
  static constexpr float m_grid_resolution_y_{20};
  static constexpr float m_relative_collision_cost_{1000};
  std::vector<GraphNode> m_closed_list_;
  std::vector<std::unique_ptr<Obstacle>> m_obstacle_list_;

  // Private methods
  ///@brief Find the closest node snapped to the graph to the given node
  /// @param node input node whose closest graph node is to be found
  GraphNode findClosestGraphNode(const Node& node) const;

  /// @brief Find the neighbors of a given graph node in the graph
  /// @param node input graph node whose neighbors are to be found
  /// @return vector of neighbors of the input graph node
  std::vector<GraphNode> getNeighbors(const GraphNode& node) const;

  /// @brief Update the cost to come of a child graph node, given the parent
  /// graph node
  /// @param child child graph node to be updated
  /// @param parent parent graph node from which the child node is reached
  void updateCostToCome(GraphNode& child, const GraphNode& parent) const;

  /// @brief Update the cost to go of a graph node to the goal graph node
  /// @param node input graph node whose cost to go is to be updated
  void updateCostToGo(GraphNode& node) const;

  /// @brief Update the cost of a graph node given the graph parent node
  /// @param node input graph node to be updated
  /// @param parent parent graph node from which the input node is reached
  void updateCost(GraphNode& node, const GraphNode& parent) const;

  /// @brief Check if a graph node is in a list of graph nodes
  /// @param node input graph node to be checked
  /// @param list list of graph nodes to be checked
  std::pair<bool, int> nodeInList(const GraphNode& node,
                                  const std::vector<GraphNode>& list) const;

  /// @brief Snap a point to the grid
  /// @param x x coordinate of the point to be snapped
  /// @param y y coordinate of the point to be snapped
  std::pair<float, float> snapToGrid(float x, float y) const;

  /// @brief Sort a list of graph nodes according to their total cost
  /// @param list list of graph nodes to be sorted
  void sortList(std::vector<GraphNode>& list) const;
};

std::vector<RobotConfig> planPathAStar(RobotConfig start, RobotConfig end,
                                       float grid_x_max, float grid_y_max,
                                       types::pyobstacle& circular_obstacles);
#endif  // A_STAR_H