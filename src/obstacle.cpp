#include "obstacle.h"

Obstacle::Obstacle() {}

Obstacle::Obstacle(float x, float y) : center_x(x), center_y(y) {}

void Obstacle::setCenter(float x, float y) {
  center_x = x;
  center_y = y;
}

std::pair<float, float> Obstacle::getCenter() { return {center_x, center_y}; }

bool Obstacle::collision(std::vector<RobotConfig> path_samples) { return true; }

CircularObstacle::CircularObstacle(float x, float y, float radius)
    : Obstacle(x, y), radius(radius) {}

float CircularObstacle::getRadius() { return radius; }

void CircularObstacle::setRadius(float radius) { this->radius = radius; }

bool CircularObstacle::collision(std::vector<RobotConfig> path_samples) {
  return false;
}
