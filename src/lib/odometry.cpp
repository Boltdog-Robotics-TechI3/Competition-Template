#include <cmath>
#include "lib/odometry.hpp"

void Odometry::reset() {
    if (leftWheel) {
        leftWheel->reset();
    }
    if (rightWheel) {
        rightWheel->reset();
    }
    if (backWheel) {
        backWheel->reset();
    }
    if (imu) {
        imu->reset(true);
        imu->tare();
    }
}

std::array<double, 4> Odometry::getReadings() {
    std::array<double, 4> readings = {0.0, 0.0, 0.0, 0.0}; // left, right, back, rotation
    if (leftWheel) {
        readings[0] = leftWheel->getDistance();
    }
    if (rightWheel) {
        readings[1] = rightWheel->getDistance();
    }
    if (backWheel) {
        readings[2] = backWheel->getDistance();
    }
    if (imu) {
        readings[3] = imu->get_rotation() * (M_PI / 180.0); // convert degrees to radians
    } else {
        readings[3] = 0.0;
    }
    return readings;
}

double Odometry::getRotationRadians() {
    if (imu) {
        return imu->get_rotation() * (M_PI / 180.0); // convert degrees to radians
    }
    return 0.0;
}

double Odometry::getRotationDegrees() {
    if (imu) {
        return imu->get_rotation();
    }
    return 0.0;
}