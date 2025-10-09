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

double Chassis::getWorldFrameHeading() {
    if (odometry && odometry->imu) {
        return -1 * Pose::degToRad(odometry->imu->get_yaw()) + M_PI_2;
    } else {
        return 0;
    }
}


/**
 * @brief Move the robot to a specific position using PID control.
 * @param targetPose The target pose to move to.
 */
void Chassis::moveTo(Pose targetPose) {
    double linearError;
    double angularError;
    double absTargetAngle;
    double leftOutput;
    double rightOutput;
    Pose robotPose = this->getPose();

    while (robotPose.distanceTo(targetPose) > 1) {

        linearError = robotPose.distanceTo(targetPose);

        absTargetAngle = robotPose.angleTo(targetPose);
		absTargetAngle = absTargetAngle < 0 ? absTargetAngle + M_TWOPI : absTargetAngle;
		
		angularError = absTargetAngle - this->getWorldFrameHeading();
		if (angularError > M_PI or angularError < (-1 * M_PI)) {
			angularError = -1 * std::copysign(1, angularError) * (M_TWOPI - abs(angularError));
		}
		
        leftOutput = linearError * 2.5 - angularError * 20;
        rightOutput = linearError * 2.5 + angularError * 20;

        
        this->tank(leftOutput, rightOutput);

        robotPose = this->getPose();

		pros::delay(20);
    }
}

//https://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
//TODO: Make tracking work with different odometry setups
#warning TODO: Figure out why x and y signs arent correct (get rid of the * -1)
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
    setPose(formerPosition.getX() + deltaD.getX(), formerPosition.getY() - deltaD.getY(), odometry->getRotation());
}