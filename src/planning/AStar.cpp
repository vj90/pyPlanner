#include "AStar.h"

#include <algorithm>

AStar::AStar(const RobotConfig& start, const RobotConfig& end,
             const float grid_x_max, const float grid_y_max)
    : m_grid_x_max_(grid_x_max), m_grid_y_max_(grid_y_max) {
  m_grid_descretization_step_x_ = grid_x_max / m_grid_resolution_x_;
  m_grid_descretization_step_y_ = grid_y_max / m_grid_resolution_y_;
  // TODO change to closest node finder
  // for hybrid A*, this becomes a func that does nothing
  m_root_node_ = findClosestGraphNode({start.x, start.y});
  m_goal_node_ = findClosestGraphNode({end.x, end.y});

  m_root_node_.cost_to_come = 0;
  m_goal_node_.cost_to_go = 0;

  // TODO replace with cost updater
  // cost to go remains the same
  // cost to go is length of the actual path
  updateCostToGo(m_root_node_);
  updateCostToCome(m_goal_node_, m_root_node_);
}

void AStar::runAStar() {
  // TODO does this need to be a member?
  m_closed_list_.clear();
  std::vector<GraphNode> open_list;
  open_list.push_back(m_root_node_);
  while (!open_list.empty()) {
    sortList(open_list);
    GraphNode current = open_list.back();
    open_list.pop_back();
    m_closed_list_.push_back(current);
    if (current == m_goal_node_) {
      break;
    }
    // TODO get neighbours is the key function here, for hybrid A* this is a
    // curve
    auto neighbors = getNeighbors(current);
    for (auto& neighbor : neighbors) {
      //  Note that if the heuristic is consistent, we never need to visit an
      //  already visited node again
      // TODO replace this, comparison of nodes is the most important here
      if (nodeInList(neighbor, m_closed_list_).first) {
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
      if (neighbor == m_goal_node_) {
        m_closed_list_.push_back(neighbor);
        break;
      }
    }
  }
}

std::vector<RobotConfig> AStar::returnPath() const {
  // TODO This needs a path interpolator somewhere
  std::vector<RobotConfig> path;
  // start with the node with the lowest cost_to_go
  const auto it = std::min_element(m_closed_list_.begin(), m_closed_list_.end(),
                                   [](const GraphNode& a, const GraphNode& b) {
                                     return a.cost_to_go < b.cost_to_go;
                                   });
  path.emplace_back(it->x, it->y);
  int parent_idx = it->parent_idx;
  while (parent_idx != -1) {
    const auto& parent_node = m_closed_list_[parent_idx];
    path.emplace_back(parent_node.x, parent_node.y);
    parent_idx = parent_node.parent_idx;
  }

  return path;
}

void AStar::addObstacle(std::unique_ptr<Obstacle> obstacle) {
  m_obstacle_list_.push_back(std::move(obstacle));
}

GraphNode AStar::findClosestGraphNode(const Node& node) const {
  // "first" node is 0,0
  // "last" node is grid_x_max, grid_y_max
  // points are snapped to the nearest grid point
  const auto& [nearest_x, nearest_y] = snapToGrid(node.x, node.y);
  GraphNode nearest_node;

  nearest_node.x = nearest_x;
  nearest_node.y = nearest_y;
  return nearest_node;
}

// Function to snap a point to the nearest grid point
std::pair<float, float> AStar::snapToGrid(float x, float y) const {
  // Snap x and y to the nearest grid point
  float snapped_x = std::round(x / m_grid_descretization_step_x_) *
                    m_grid_descretization_step_x_;
  float snapped_y = std::round(y / m_grid_descretization_step_y_) *
                    m_grid_descretization_step_y_;

  // Ensure the snapped points don't exceed max boundaries
  snapped_x = std::min(snapped_x, std::floor(m_grid_x_max_));
  snapped_y = std::min(snapped_y, std::floor(m_grid_y_max_));

  return {snapped_x, snapped_y};
}

void AStar::sortList(std::vector<GraphNode>& list) const {
  std::sort(list.begin(), list.end(),
            [](const GraphNode& a, const GraphNode& b) {
              return a.cost() > b.cost();
            });
}

std::vector<GraphNode> AStar::getNeighbors(const GraphNode& node) const {
  // Every node has max 9 neighboring nodes
  std::vector<GraphNode> neighbors;
  const auto parent_idx = m_closed_list_.size() - 1;
  constexpr int num_neighbours = 9;
  const std::array<float, 3> del_x{-m_grid_descretization_step_x_, 0.0,
                                   m_grid_descretization_step_x_};
  const std::array<float, 3> del_y{-m_grid_descretization_step_y_, 0.0,
                                   m_grid_descretization_step_y_};

  const auto x = node.x;
  const auto y = node.y;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == 1 && j == 1) {
        continue;
      }
      const float xc = x + del_x[i];
      const float yc = y + del_x[j];
      if (xc <= m_grid_x_max_ && xc >= 0 && yc <= m_grid_y_max_ && yc >= 0) {
        GraphNode node;
        node.x = xc;
        node.y = yc;
        node.parent_idx = parent_idx;
        neighbors.push_back(node);
      }
    }
  }

  return neighbors;
}

void AStar::updateCostToCome(GraphNode& child, const GraphNode& parent) const {
  const float collision_cost =
      isEdgeInCollision(child, parent) ? m_relative_collision_cost_ : 0.0;
  const float ctc = std::hypot(child.x - parent.x, child.y - parent.y) +
                    parent.cost_to_come + collision_cost;
  child.cost_to_come = ctc;
}

void AStar::updateCostToGo(GraphNode& node) const {
  node.cost_to_go =
      std::hypot(node.x - m_goal_node_.x, node.y - m_goal_node_.y);
}

void AStar::updateCost(GraphNode& node, const GraphNode& parent) const {
  updateCostToCome(node, parent);
  updateCostToGo(node);
}

std::pair<bool, int> AStar::nodeInList(
    const GraphNode& node, const std::vector<GraphNode>& list) const {
  for (int i = 0; i < list.size(); i++) {
    if (node == list[i]) {
      return {true, i};
    }
  }
  return {false, -1};
}

std::vector<RobotConfig> AStar::sampleEdge(const GraphNode& node_start,
                                           const GraphNode& node_end) const {
  std::vector<RobotConfig> samples;
  samples.reserve(m_num_path_samples_);
  const float increment = 1.0f / (m_num_path_samples_ - 1);
  for (int i = 0; i < m_num_path_samples_; i++) {
    const auto x = i * increment * (node_end.x - node_start.x) + node_start.x;
    const auto y = i * increment * (node_end.y - node_start.y) + node_start.y;
    samples.emplace_back(x, y);
  }
  return samples;
}

bool AStar::isEdgeInCollision(const GraphNode& node_start,
                              const GraphNode& node_end) const {
  const auto samples = sampleEdge(node_start, node_end);

  for (const auto& obstacle : m_obstacle_list_) {
    if (obstacle->collision(samples)) {
      return true;
    }
  }
  return false;
}

std::vector<RobotConfig> planPathAStar(RobotConfig start, RobotConfig end,
                                       float grid_x_max, float grid_y_max,
                                       types::pyobstacle& circular_obstacles) {
  AStar astar_planner(start, end, grid_x_max, grid_y_max);
  for (const auto& obstacle : circular_obstacles) {
    const auto& [x, y, center] = obstacle;
    astar_planner.addObstacle(std::make_unique<CircularObstacle>(x, y, center));
  }

  astar_planner.runAStar();
  std::vector<RobotConfig> result;
  result = astar_planner.returnPath();
  // TODO add meta data
  return result;
}