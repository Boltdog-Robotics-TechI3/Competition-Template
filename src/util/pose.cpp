#include <cmath>
#include "util/pose.hpp"

void Pose::setPose(Pose newPose) {
    x = newPose.getX();
    y = newPose.getY();
    theta = newPose.getTheta();
}

double Pose::distanceTo(const Pose& other) {
    double dx = other.getX() - x;
    double dy = other.getY() - y;
    return std::sqrt(dx * dx + dy * dy);
}

double Pose::angleTo(const Pose& other) {
    double dx = other.getX() - x;
    double dy = other.getY() - y;
    return std::atan2(dy, dx);
}

Pose Pose::rotate(double angle) {
    double magnitude = sqrt((x*x) + (y*y));
    double theta = (atan2(x, y));

    theta += angle;

    return Pose(magnitude * sin(theta), magnitude * cos(theta), theta);
}

std::string Pose::to_string() {
    return "X: " + std::to_string(x) + " Y: " + std::to_string(y) + " Theta (rad): " + std::to_string(theta);
}
