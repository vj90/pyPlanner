#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <cmath>
#include <vector>

#include "RobotConfig.h"

class Obstacle {
 public:
  Obstacle();
  Obstacle(float x, float y);
  void setCenter(float x, float y);
  std::pair<float, float> getCenter();
  // TODO change to pure virtual function and fix pybindings
  virtual bool collision(std::vector<RobotConfig> path_samples);

 private:
  float center_x{NAN};
  float center_y{NAN};
};

class CircularObstacle : public Obstacle {
 public:
  CircularObstacle(float x, float y, float radius);
  float getRadius();
  void setRadius(float radius);
  bool collision(std::vector<RobotConfig> path_samples) override;

 private:
  float radius{NAN};
};

#endif