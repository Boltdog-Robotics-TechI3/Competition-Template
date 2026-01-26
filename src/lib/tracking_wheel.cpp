#include <cmath>
#include "lib/tracking_wheel.hpp"



void TrackingWheel::reset() {
    lastPosition = 0.0;
    totalDistance = 0.0;
    encoder->reset();
    encoder->reset_position();
}

void TrackingWheel::reverse() {
    encoder->set_reversed(!encoder->get_reversed());
}

double TrackingWheel::getRotations() {
    return encoder->get_position() / 100.0 / 360.0; // convert degrees to rotations
}

double TrackingWheel::getDistance() {
    double rotations = getRotations();  // in rotations
    double distance = rotations * (wheelDiameter * M_PI); // in inches
    return distance;
}

void TrackingWheel::calculateVelocity() {
    currentTime = pros::millis();
    int dt = currentTime - previousTime;
    double dx = getDistance() - lastPosition;
    velocity = dx/dt;
    previousTime = currentTime;
}


