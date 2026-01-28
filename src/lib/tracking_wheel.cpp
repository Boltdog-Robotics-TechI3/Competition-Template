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

float TrackingWheel::getRotations() {
    return encoder->get_position() / 100.0 / 360.0; // convert degrees to rotations
}

float TrackingWheel::getDistance() {
    float rotations = getRotations();  // in rotations
    float distance = rotations * (wheelDiameter * M_PI); // in inches
    return distance;
}

void TrackingWheel::calculateVelocity(float dt) {
    float dx = getDistance() - lastPosition;
    velocity = dx/dt;
    lastPosition = getDistance();
}


