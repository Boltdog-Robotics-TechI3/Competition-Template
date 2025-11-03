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
    // TODO: Implement MoveTo for DifferentialChassis
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
