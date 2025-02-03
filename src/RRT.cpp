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

RRT::RRT(RobotConfig start, RobotConfig end, float grid_x_max,
         float grid_y_max) {
  root = std::make_unique<Node>(start.x, start.y);
  goal = std::make_unique<Node>(end.x, end.y);
  nodes.push_back(std::move(root));
  this->grid_x_max = grid_x_max;
  this->grid_y_max = grid_y_max;
  srand(time(NULL));
}

RRT::~RRT() {}

void RRT::runRRT() {
  int ctr = 0;
  while (ctr < max_itr) {
    // sample a point
    Nptr sample = this->sample();
    // check it is valid
    if (isValid(sample)) {
      auto res = nearest_node(sample);
      const auto& [parent_node_idx, dist] = res;
      gotoNode(nodes[parent_node_idx], sample, dist);
      add_edge(parent_node_idx, sample);
      nodes.push_back(std::move(sample));
      ctr++;
      // check distance to goal
      if (distanceToGoal(nodes.back()) <= goal_threshold_dist) {
        Nptr goal_sample = std::make_unique<Node>(goal->x, goal->y);
        add_edge(nodes.size() - 1, goal_sample);
        nodes.push_back(std::move(goal_sample));
        break;
      }
    }
  }
}

RRT::Nptr RRT::sample() {
  int x = rand() % (int)grid_x_max;
  int y = rand() % (int)grid_y_max;
  return std::make_unique<Node>(x, y);
}

bool RRT::isValid(const Nptr& node) {
  if (node->x < 0 || node->x > grid_x_max || node->y < 0 ||
      node->y > grid_y_max) {
    return false;
  }
  return true;
}

std::pair<std::size_t, float> RRT::nearest_node(const Nptr& sample) {
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

float RRT::distanceToGoal(const Nptr& sample) {
  return std::hypot(sample->x - goal->x, sample->y - goal->y);
}

void RRT::gotoNode(const Nptr& nearest_node, Nptr& sample, float dist) {
  float t = 1;
  if (dist > step_size) {
    t = this->step_size / dist;
  }
  sample->y = t * (sample->y - nearest_node->y) + nearest_node->y;
  sample->x = t * (sample->x - nearest_node->x) + nearest_node->x;
}

void RRT::add_edge(const int parent_idx, Nptr& child) {
  child->parent_idx = parent_idx;
}

void RRT::print_nodes() {
  for (int i = 0; i < nodes.size(); i++) {
    std::cout << *(nodes[i]) << std::endl;
  }
}

void RRT::print_nodes2() {
  std::cout << "nodes_x = [";
  for (int i = 0; i < nodes.size(); i++) {
    std::cout << nodes[i]->x << ",";
  }
  std::cout << "]" << std::endl << "nodes_y = [";
  for (int i = 0; i < nodes.size(); i++) {
    std::cout << nodes[i]->y << ",";
  }
  std::cout << "]" << std::endl;
}

std::vector<RobotConfig> RRT::returnPath() {
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

void RRT::printPath() {
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

RRT::RRTMetaData RRT::getMetaData() {
  // Todo ADD some way to trace back parent node
  RRTMetaData data;
  data.reserve(nodes.size());
  for (const auto& node : nodes) {
    data.push_back(
        std::make_pair(RobotConfig(node->x, node->y), node->parent_idx));
  }
  return data;
}

PlannerResult<RRT::RRTMetaData> planPath(float x_start, float y_start,
                                         float x_end, float y_end,
                                         float grid_x_max, float grid_y_max) {
  RRT rrt_planner(x_start, y_start, x_end, y_end, grid_x_max, grid_y_max);
  rrt_planner.runRRT();
  std::cout << "\n#All nodes\n";
  rrt_planner.print_nodes2();
  std::cout << "\n#Path\n";
  rrt_planner.printPath();
  PlannerResult<RRT::RRTMetaData> result;
  result.path = rrt_planner.returnPath();
  result.metadata.data = rrt_planner.getMetaData();
  return result;
}