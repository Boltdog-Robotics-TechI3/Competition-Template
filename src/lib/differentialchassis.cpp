#include <cmath>
#include "lib/differentialchassis.hpp"
#include "pros/rtos.hpp"

/**
 * @brief Move the robot in arcade mode. The left joystick controls the forward/backward movement, and the right joystick controls the rotation.
 * @param leftY The value of the left joystick (forward/backward movement).
 * @param rightX The value of the right joystick (rotation).
 */
void DifferentialChassis::arcade(int leftY, int rightX) {
    if (drivetrain) {
        int leftPower = leftY + rightX;
        int rightPower = leftY - rightX;
        drivetrain->setMotorSpeeds({leftPower, rightPower});
    }
}

/**
 * @brief Move the robot in tank mode. The left joystick controls the left side motors, and the right joystick controls the right side motors.
 * @param leftY The value of the left joystick (left side motors).
 * @param rightY The value of the right joystick (right side motors).
 */
void DifferentialChassis::tank(int leftY, int rightY) {
    if (drivetrain) {
        drivetrain->setMotorSpeeds({leftY, rightY});
    }
}

/**
 * @brief Move the robot to a specific position using PID control.
 * @param targetPose The target pose to move to.
 */
void DifferentialChassis::moveToPose(Pose targetPose) {
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

/**
 * @brief Turn the robot to a specific angle using PID control.
 * 0 Degrees is facing "forward" from the starting orientation.
 * 
 * @param targetAngle The target angle to turn to (in degrees).
 */
void DifferentialChassis::turnAngle(double targetAngle) {
    // TODO: Implement TurnAngle for DifferentialChassis
}
