#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <cmath>
#include <iostream>

struct RobotConfig {
  float x{NAN};
  float y{NAN};
  float theta{NAN};
  RobotConfig(const float x, const float y, const float theta = NAN) {
    this->x = x;
    this->y = y;
    this->theta = theta;
  }
  RobotConfig() {}
  friend std::ostream& operator<<(std::ostream& os, const RobotConfig& config) {
    os << config.x << " " << config.y << " " << config.theta << std::endl;
    return os;
  }
};

#endif  // ROBOT_CONFIG_H