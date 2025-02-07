#include "Obstacle.h"

Obstacle::Obstacle() {}

Obstacle::Obstacle(const float x, const float y) : center_x(x), center_y(y) {}

void Obstacle::setCenter(const float x, const float y) {
  center_x = x;
  center_y = y;
}

std::pair<float, float> Obstacle::getCenter() { return {center_x, center_y}; }

bool Obstacle::collision(const std::vector<RobotConfig>& path_samples) const {
  return true;
}

CircularObstacle::CircularObstacle(const float x, const float y,
                                   const float radius)
    : Obstacle(x, y), radius(radius) {}

float CircularObstacle::getRadius() { return radius; }

void CircularObstacle::setRadius(const float radius) { this->radius = radius; }

bool CircularObstacle::collision(
    const std::vector<RobotConfig>& path_samples) const {
  for (const auto& sample : path_samples) {
    if (std::hypot(sample.x - center_x, sample.y - center_y) <= radius) {
      return true;
    }
  }
  return false;
}
