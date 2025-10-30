#include "chassis.hpp"
#include <cmath>

void Chassis::reset() {
    if (odometry) {
        odometry->reset();
    }
    if (drivetrain) {
        drivetrain->setMotorSpeeds({0, 0});
    }
    if (!tracking) {
        startTracking();
    }
}

void Chassis::stop() {
    if (drivetrain) {
        drivetrain->setMotorSpeeds({0, 0});
    }
}

/**
 * @brief Get the robot's current pose (position and orientation).
 * @return The robot's current pose.
 */
Pose Chassis::getPose() const { 
    return *pose; 
}

/**
 * @brief Set the robot's current pose (position and orientation).
 * @param newPose The new pose to set.
 */
void Chassis::setPose(Pose newPose) { 
    *pose = newPose; 
}

/**
 * @brief Set the robot's current pose (position and orientation) using individual values.
 * @param x The new x-coordinate.
 * @param y The new y-coordinate.
 * @param theta The new orientation (in radians).
 */
void Chassis::setPose(double x, double y, double theta) {
    pose->setX(x);
    pose->setY(y);
    pose->setTheta(theta);
}

/**
 * @brief Sets the brake mode for the chassis.
 * @param mode The brake mode to set.
 */
void Chassis::setBrakeMode(pros::motor_brake_mode_e_t mode) {
    if (drivetrain) {
        drivetrain->setBrakeMode(mode);
    }   
}   

//https://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
//TODO: Make tracking work with different odometry setups
void Chassis::trackPosition() {
    // Get current position
    std::array<double, 4> currentPose = odometry->getReadings();

    double currentLeft = currentPose[0];
    // auto currentRight = currentPose[1];
    double currentBack = currentPose[2];

    // Calculate changes since last reading
    double previousLeft = odometry->leftWheel->getLastPosition();
    // auto previousRight = odometry->rightWheel->getLastPosition();
    double previousBack = odometry->backWheel->getLastPosition();

    double leftChange = currentLeft - previousLeft;
    // auto rightChange = currentRight - previousRight;
    double backChange = currentBack - previousBack;

    // Update previous positions
    odometry->leftWheel->setLastPosition(currentLeft);
    //odometry->rightWheel->setLastPosition(currentRight);
    odometry->backWheel->setLastPosition(currentBack);

    Pose formerPosition = getPose();

    // Calculate the change in orientation
    double delTheta = odometry->getRotationRadians() - formerPosition.getTheta();
    
    // Calculate local displacement vector
    double deltaDl[2]; 
    if(delTheta == 0){
        deltaDl[0] = backChange;
        deltaDl[1] = leftChange;
    } else {
        deltaDl[0] = (2 * sin(delTheta / 2)) * ((backChange / delTheta) + (odometry->backWheel->getOffset()));
        deltaDl[1] = (2 * sin(delTheta / 2)) * ((leftChange / delTheta) + (odometry->leftWheel->getOffset()));
    }
    Pose deltaD = Pose(deltaDl[0], deltaDl[1], delTheta);
    
    // Calculate average orientation
    double thetaM = formerPosition.getTheta() + (delTheta / 2);

    // Rotate local displacement to global frame
    deltaD = deltaD.rotate(-1*thetaM);

    // Update the position
    setPose(formerPosition.getX() + deltaD.getX(), formerPosition.getY() + deltaD.getY(), odometry->getRotationRadians());
}

