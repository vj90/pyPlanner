#include "AStar.h"

#include <algorithm>

AStar::AStar(const RobotConfig& start, const RobotConfig& end,
             const float grid_x_max, const float grid_y_max)
    : grid_x_max(grid_x_max), grid_y_max(grid_y_max) {
  grid_descretization_step_x = grid_x_max / grid_resolution_x;
  grid_descretization_step_y = grid_y_max / grid_resolution_y;
  root_node = findClosestGraphNode({start.x, start.y});
  goal_node = findClosestGraphNode({end.x, end.y});

  // TODO Fix this (use std::optional?)
  root_node.cost_to_come = 0;
  goal_node.cost_to_go = 0;
  updateCostToGo(root_node);
  updateCostToCome(goal_node, root_node);
}

void AStar::runAStar() {
  // TODO does this need to be a member function?
  closed_list.clear();
  std::vector<NodeWithCost> open_list;
  open_list.push_back(root_node);
  while (!open_list.empty()) {
    sortList(open_list);
    NodeWithCost current = open_list.back();
    open_list.pop_back();
    closed_list.push_back(current);
    if (current == goal_node) {
      break;
    }
    auto neighbors = getNeighbors(current);
    for (auto& neighbor : neighbors) {
      //  Note that if the heuristic is consistent, we never need to visit an
      //  already visited node again
      if (nodeInList(neighbor, closed_list).first) {
        continue;
      }
      // Doing this here instead of while generating neighbors to avoid extra
      // calculations for already visited nodes
      updateCost(neighbor, current);
      const auto& [is_neighbor_in_list, candidate_idx] =
          nodeInList(neighbor, open_list);
      if (is_neighbor_in_list) {
        auto& candidate = open_list[candidate_idx];
        if (neighbor.cost() < candidate.cost()) {
          candidate = neighbor;
        }
      } else {
        open_list.push_back(neighbor);
      }
      if (neighbor == goal_node) {
        closed_list.push_back(neighbor);
        break;
      }
    }
  }
}

std::vector<RobotConfig> AStar::returnPath() const {
  std::vector<RobotConfig> path;
  // start with the node with the lowest cost_to_go
  const auto it =
      std::min_element(closed_list.begin(), closed_list.end(),
                       [](const NodeWithCost& a, const NodeWithCost& b) {
                         return a.cost_to_go < b.cost_to_go;
                       });
  path.emplace_back(it->x, it->y);
  int parent_idx = it->parent_idx;
  while (parent_idx != -1) {
    const auto& parent_node = closed_list[parent_idx];
    path.emplace_back(parent_node.x, parent_node.y);
    parent_idx = parent_node.parent_idx;
  }

  return path;
}

void AStar::add_obstacle(std::unique_ptr<Obstacle> obstacle) {}

NodeWithCost AStar::findClosestGraphNode(const Node& node) const {
  // "first" node is 0,0
  // "last" node is grid_x_max, grid_y_max
  // points are snapped to the nearest grid point
  const auto& [nearest_x, nearest_y] = snapToGrid(node.x, node.y);
  NodeWithCost nearest_node;

  nearest_node.x = nearest_x;
  nearest_node.y = nearest_y;
  return nearest_node;
}

// Function to snap a point to the nearest grid point
std::pair<float, float> AStar::snapToGrid(float x, float y) const {
  // Snap x and y to the nearest grid point
  float snapped_x =
      std::round(x / grid_descretization_step_x) * grid_descretization_step_x;
  float snapped_y =
      std::round(y / grid_descretization_step_y) * grid_descretization_step_y;

  // Ensure the snapped points don't exceed max boundaries
  snapped_x = std::min(snapped_x, std::floor(grid_x_max));
  snapped_y = std::min(snapped_y, std::floor(grid_y_max));

  return {snapped_x, snapped_y};
}

void AStar::sortList(std::vector<NodeWithCost>& list) const {
  std::sort(list.begin(), list.end(),
            [](const NodeWithCost& a, const NodeWithCost& b) {
              return a.cost() > b.cost();
            });
}

std::vector<NodeWithCost> AStar::getNeighbors(const NodeWithCost& node) const {
  // Every node has max 9 neighboring nodes
  std::vector<NodeWithCost> neighbors;
  const auto parent_idx = closed_list.size() - 1;
  constexpr int num_neighbours = 9;
  const std::array<float, 3> del_x{-grid_descretization_step_x, 0.0,
                                   grid_descretization_step_x};
  const std::array<float, 3> del_y{-grid_descretization_step_y, 0.0,
                                   grid_descretization_step_y};

  const auto x = node.x;
  const auto y = node.y;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == 1 && j == 1) {
        continue;
      }
      const float xc = x + del_x[i];
      const float yc = y + del_x[j];
      if (xc <= grid_x_max && xc >= 0 && yc <= grid_y_max && yc >= 0) {
        NodeWithCost node;
        node.x = xc;
        node.y = yc;
        node.parent_idx = parent_idx;
        neighbors.push_back(node);
      }
    }
  }

  return neighbors;
}

void AStar::updateCostToCome(NodeWithCost& child,
                             const NodeWithCost& parent) const {
  const float collision_cost = 0.0;  // TODO implement this
  const float ctc = std::hypot(child.x - parent.x, child.y - parent.y) +
                    parent.cost_to_come + collision_cost;
  child.cost_to_come = ctc;
}

void AStar::updateCostToGo(NodeWithCost& node) const {
  node.cost_to_go = std::hypot(node.x - goal_node.x, node.y - goal_node.y);
}

void AStar::updateCost(NodeWithCost& node, const NodeWithCost& parent) const {
  updateCostToCome(node, parent);
  updateCostToGo(node);
}

std::pair<bool, int> AStar::nodeInList(
    const NodeWithCost& node, const std::vector<NodeWithCost>& list) const {
  for (int i = 0; i < list.size(); i++) {
    if (node == list[i]) {
      return {true, i};
    }
  }
  return {false, -1};
}

std::vector<RobotConfig> planPathAStar(RobotConfig start, RobotConfig end,
                                       float grid_x_max, float grid_y_max,
                                       types::pyobstacle& circular_obstacles) {
  AStar astar_planner(start, end, grid_x_max, grid_y_max);
  /* for (const auto& obstacle : circular_obstacles) {
     const auto& [x, y, center] = obstacle;
     astar_planner.add_obstacle(
         std::make_unique<CircularObstacle>(x, y, center));
   }*/

  astar_planner.runAStar();
  std::vector<RobotConfig> result;
  result = astar_planner.returnPath();
  // TODO add meta data
  return result;
}