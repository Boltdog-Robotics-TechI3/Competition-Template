#include <cmath>
#include "lib/chassis.hpp"
#include "pros/rtos.hpp"

Chassis::Chassis(Drivetrain *drivetrain, Odometry *odometry)
    : drivetrain(drivetrain), odometry(odometry), pose(new Pose()) {
}

void Chassis::reset() {
    if (odometry) {
        odometry->reset();
    }
    if (drivetrain) {
        drivetrain->leftMotors->move_velocity(0);
        drivetrain->rightMotors->move_velocity(0);
    }
    if (!tracking) {
        startTracking();
    }
}

void Chassis::arcade(int leftY, int rightX) {
    if (drivetrain) {
        int leftPower = leftY + rightX;
        int rightPower = leftY - rightX;
        drivetrain->leftMotors->move(leftPower);
        drivetrain->rightMotors->move(rightPower);
    }
}

void Chassis::tank(int leftY, int rightY) {
    if (drivetrain) {
        drivetrain->leftMotors->move(leftY);
        drivetrain->rightMotors->move(rightY);
    }
}

void Chassis::stop() {
    if (drivetrain) {
        drivetrain->leftMotors->move_velocity(0);
        drivetrain->rightMotors->move_velocity(0);
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
    double delTheta = odometry->getRotation() - formerPosition.getTheta();
    
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
    setPose(formerPosition.getX() + deltaD.getX(), formerPosition.getY() + deltaD.getY(), odometry->getRotation());
}

void Chassis::moveDistance(double distance) {
    if (!lateralPID || !turnPID) {
        return;
    }
    lateralPID->reset();
    lateralPID->setP(0.1);

    bool targetReached = false;
    double initialPosition = odometry ? odometry->getReadings()[0] : (drivetrain->leftMotors->get_position() + drivetrain->rightMotors->get_position()) / 2.0;
    int startTime = pros::millis();

    // while (!targetReached) {
    //     if ()
    //     double currentPosition = odometry ? odometry->getReadings()[0] : (drivetrain->leftMotors->get_position() + drivetrain->rightMotors->get_position()) / 2.0;
    //     double output = lateralPID->calculate(currentPosition, distance);
    //     drivetrain->leftMotors->move(output);
    //     drivetrain->rightMotors->move(output);
    //     pros::delay(20);
    // }

    drivetrain->leftMotors->move_velocity(0);
    drivetrain->rightMotors->move_velocity(0);
}