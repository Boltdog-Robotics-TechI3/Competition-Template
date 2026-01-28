#include <cmath>
#include "util/pose.hpp"

void Pose::setPose(Pose newPose) {
    x = newPose.getX();
    y = newPose.getY();
    theta = newPose.getTheta();
}

float Pose::distanceTo(const Pose& other) {
    float dx = other.getX() - x;
    float dy = other.getY() - y;
    return std::sqrt(dx * dx + dy * dy);
}

float Pose::angleTo(const Pose& other) {
    float dx = other.getX() - x;
    float dy = other.getY() - y;
    return std::atan2(dy, dx);
}

Pose Pose::rotate(float angle) {
    float magnitude = sqrt((x*x) + (y*y));
    float theta = (atan2(x, y));

    theta += angle;

    return Pose(magnitude * sin(theta), magnitude * cos(theta), theta);
}

std::string Pose::to_string() {
    return "X: " + std::to_string(x) + " Y: " + std::to_string(y) + " Theta (rad): " + std::to_string(theta);
}
