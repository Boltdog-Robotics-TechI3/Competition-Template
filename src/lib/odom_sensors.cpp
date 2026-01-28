#include <cmath>
#include "lib/odom_sensors.hpp"

void OdomSensors::calculateBodyFrameVelocities(float dt) {    
    float vertWheelSpeed = verticalWheel->getWheelVelocity();
    float horzWheelSpeed = horizontalWheel->getWheelVelocity();
    float angularVelocity = getAngularVelocity();

    bodyVelocityY = vertWheelSpeed - (angularVelocity * verticalWheel->getOffset());
    bodyVelocityX = horzWheelSpeed - (angularVelocity * horizontalWheel->getOffset());
}

void OdomSensors::reset() {
    previousTime = pros::millis();
    currentTime = pros::millis();

    if (verticalWheel) {
        verticalWheel->reset();
    }
    if (horizontalWheel) {
        horizontalWheel->reset();
    }
    if (imu) {
        imu->reset(true);
        imu->tare();
    }
}

void OdomSensors::update() {
    currentTime = pros::millis();

    float dt = (currentTime - previousTime) / 1000.0;

    verticalWheel->calculateVelocity(dt);
    horizontalWheel->calculateVelocity(dt);

    calculateBodyFrameVelocities(dt);

    previousTime = currentTime;
}

std::array<float, 3> OdomSensors::getReadings() {
    std::array<float, 3> readings = {0.0, 0.0, 0.0}; // vertical, horizontal, rotation
    if (verticalWheel) {
        readings[0] = verticalWheel->getDistance();
    }
    if (horizontalWheel) {
        readings[1] = horizontalWheel->getDistance();
    }
    if (imu) {
        readings[2] = imu->get_rotation() * (M_PI / 180.0); // convert degrees to radians
    } else {
        readings[2] = 0.0;
    }
    return readings;
}

float OdomSensors::getRotationRadians() {
    if (imu) {
        return imu->get_rotation() * (M_PI / 180.0); // convert degrees to radians
    }
    return 0.0;
}

float OdomSensors::getRotationDegrees() {
    if (imu) {
        return imu->get_rotation();
    }
    return 0.0;
}

float OdomSensors::getAngularVelocity() {
    if (imu) {
        return (imu->get_gyro_rate().z) * M_PI / 180;
    }

    return 0.0;
}

