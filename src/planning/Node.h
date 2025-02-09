#ifndef NODE_H
#define NODE_H

#include <cmath>
#include <iostream>
#include <sstream>
class Node {
 public:
  float x{NAN}, y{NAN};
  int parent_idx{-1};
  Node(float x, float y) {
    this->x = x;
    this->y = y;
    this->parent_idx = -1;
  }
  friend std::ostream& operator<<(std::ostream& os, const Node& node) {
    os << node.x << " " << node.y;
    return os;
  }
  Node() = default;
  bool operator==(const Node& other) const {
    const float EPSILON = 0.01;
    return std::abs(x - other.x) < EPSILON && std::abs(y - other.y) < EPSILON;
  }
};

// convenience type
using Nptr = std::unique_ptr<Node>;

#endif  // NODE_H