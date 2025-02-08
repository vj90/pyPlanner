#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <cmath>
#include <vector>

#include "RobotConfig.h"

class Obstacle {
 public:
  Obstacle();
  Obstacle(const float x, const float y);
  void setCenter(const float x, const float y);
  std::pair<float, float> getCenter();
  // TODO change to pure virtual function and fix pybindings
  virtual bool collision(const std::vector<RobotConfig>& path_samples) const;

 protected:
  float center_x{NAN};
  float center_y{NAN};
};

class CircularObstacle : public Obstacle {
 public:
  CircularObstacle(const float x, const float y, const float radius);
  float getRadius();
  void setRadius(const float radius);
  bool collision(const std::vector<RobotConfig>& path_samples) const override;

 private:
  float radius{NAN};
};

#endif