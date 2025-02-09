#include "RRT.h"

#include <time.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

RRT::RRT(float start_x, float start_y, float end_x, float end_y,
         float grid_x_max, float grid_y_max) {
  root = std::make_unique<Node>(start_x, start_x);
  goal = std::make_unique<Node>(end_x, end_y);

  Nptr root_sample = std::make_unique<Node>(start_x, start_y);
  nodes.push_back(std::move(root_sample));
  this->grid_x_max = grid_x_max;
  this->grid_y_max = grid_y_max;
  srand(time(NULL));
}

RRT::RRT(RobotConfig& start, RobotConfig& end, float grid_x_max,
         float grid_y_max) {
  root = std::make_unique<Node>(start.x, start.y);
  goal = std::make_unique<Node>(end.x, end.y);
  Nptr root_sample = std::make_unique<Node>(start.x, start.y);
  nodes.push_back(std::move(root));
  this->grid_x_max = grid_x_max;
  this->grid_y_max = grid_y_max;
  srand(time(NULL));
}

void RRT::runRRT() {
  int ctr = 0;
  int valid_ctr = 0;
  while (ctr < max_itr && valid_ctr < max_valid_itr) {
    Nptr sample = this->sample();
    // check it is valid. TODO is this required?
    if (isValid(sample)) {
      const auto [parent_node_idx, dist] = nearest_node(sample);
      gotoNode(nodes[parent_node_idx], sample, dist);
      if (inCollision(nodes[parent_node_idx], sample)) {
        continue;
      }
      add_edge(parent_node_idx, sample);
      nodes.push_back(std::move(sample));
      valid_ctr++;
      // check distance to goal
      if (distanceToGoal(nodes.back()) <= goal_threshold_dist) {
        Nptr goal_sample = std::make_unique<Node>(goal->x, goal->y);
        if (inCollision(nodes.back(), goal_sample)) {
          continue;
        }
        add_edge(nodes.size() - 1, goal_sample);
        nodes.push_back(std::move(goal_sample));
        break;
      }
    }
    ctr++;
  }
}

Nptr RRT::sample() const {
  const int x = rand() % (int)grid_x_max;
  const int y = rand() % (int)grid_y_max;
  return std::make_unique<Node>(x, y);
}

bool RRT::isValid(const Nptr& node) const {
  if (node->x < 0 || node->x > grid_x_max || node->y < 0 ||
      node->y > grid_y_max) {
    return false;
  }
  return true;
}

bool RRT::inCollision(const Nptr& node_start, const Nptr& node_end) const {
  const auto edge_samples = sampleEdge(node_start, node_end);
  for (const auto& sample : edge_samples) {
    for (const auto& obstacle : obstacle_list) {
      if (obstacle->collision({sample})) {
        return true;
      }
    }
  }
  return false;
}

std::vector<RobotConfig> RRT::sampleEdge(const Nptr& node_start,
                                         const Nptr& node_end) const {
  std::vector<RobotConfig> samples;

  samples.reserve(num_path_samples);
  const float increment = 1.0f / (num_path_samples - 1);
  for (int i = 0; i < num_path_samples; i++) {
    const auto x =
        i * increment * (node_end->x - node_start->x) + node_start->x;
    const auto y =
        i * increment * (node_end->y - node_start->y) + node_start->y;
    samples.emplace_back(x, y);
  }
  return samples;
}

std::pair<std::size_t, float> RRT::nearest_node(const Nptr& sample) const {
  float min_dist = INT16_MAX;
  std::size_t min_index = 0;
  for (std::size_t i = 0; i < nodes.size(); i++) {
    float dist = std::hypot(sample->x - nodes[i]->x, sample->y - nodes[i]->y);
    if (dist < min_dist) {
      min_dist = dist;
      min_index = i;
    }
  }
  return std::make_pair(min_index, min_dist);
}

float RRT::distanceToGoal(const Nptr& sample) const {
  return std::hypot(sample->x - goal->x, sample->y - goal->y);
}

void RRT::gotoNode(const Nptr& nearest_node, Nptr& sample, float dist) const {
  float t = 1;
  if (dist > step_size) {
    t = this->step_size / dist;
  }
  sample->y = t * (sample->y - nearest_node->y) + nearest_node->y;
  sample->x = t * (sample->x - nearest_node->x) + nearest_node->x;
}

void RRT::add_edge(const int parent_idx, Nptr& child) const {
  child->parent_idx = parent_idx;
}

void RRT::print_nodes() const {
  for (int i = 0; i < nodes.size(); i++) {
    std::cout << *(nodes[i]) << std::endl;
  }
}

std::vector<RobotConfig> RRT::returnPath() const {
  std::vector<RobotConfig> path;
  int parent_idx = -1;

  // Find closest node to goal
  const auto res = nearest_node(goal);
  const auto& closest_node = nodes[res.first];
  path.emplace_back(closest_node->x, closest_node->y);
  parent_idx = closest_node->parent_idx;

  while (parent_idx != -1) {
    const auto& parent = nodes[parent_idx];
    path.emplace_back(parent->x, parent->y);
    parent_idx = parent->parent_idx;
  }
  return path;
}

void RRT::printPath() const {
  auto path = returnPath();
  std::cout << "path_x = [";
  for (const auto& pt : path) {
    std::cout << pt.x << ",";
  }
  std::cout << "]" << std::endl << "path_y = [";
  for (const auto& pt : path) {
    std::cout << pt.y << ",";
  }
  std::cout << "]" << std::endl;
}

RRT::RRTMetaData RRT::getMetaData() const {
  // Todo ADD some way to trace back parent node
  RRTMetaData data;
  data.reserve(nodes.size());
  for (const auto& node : nodes) {
    data.push_back(
        std::make_pair(RobotConfig(node->x, node->y), node->parent_idx));
  }
  return data;
}

void RRT::add_obstacle(std::unique_ptr<Obstacle> obstacle) {
  obstacle_list.push_back(std::move(obstacle));
}

PlannerResult<RRT::RRTMetaData> planPath(
    RobotConfig start, RobotConfig end, float grid_x_max, float grid_y_max,
    types::pyobstacle& circular_obstacles) {
  RRT rrt_planner(start, end, grid_x_max, grid_y_max);
  for (const auto& obstacle : circular_obstacles) {
    const auto& [x, y, center] = obstacle;
    rrt_planner.add_obstacle(std::make_unique<CircularObstacle>(x, y, center));
  }
  rrt_planner.runRRT();
  PlannerResult<RRT::RRTMetaData> result;
  result.path = rrt_planner.returnPath();
  result.metadata.data = rrt_planner.getMetaData();
  return result;
}