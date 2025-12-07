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
 * @brief Move the robot towards a specific position using a single step of PID control.
 * 
 * @note This method is intended to be called repeatedly in a loop until the target position is reached.
 * Use moveToPose() for a blocking call that handles the loop internally and if the target pose won't change during the loop.
 * Use this method if your target position may change dynamically.
 * 
 * @note This method ignores the angle of the target pose and only drives to the x and y coordinates.
 * 
 * @param targetPose The target pose to move to.
 */
void DifferentialChassis::moveToPoseStep(Pose targetPose, bool isForward) {
    double linearError = pose->distanceTo(targetPose);

    double absTargetAngle = pose->angleTo(targetPose) + (isForward ? 0 : M_PI);
    absTargetAngle = absTargetAngle < 0 ? absTargetAngle + M_TWOPI : absTargetAngle;
    
    double angularError = absTargetAngle - getWorldFrameHeading();
    if (angularError > M_PI or angularError < (-1 * M_PI)) {
        angularError = -1 * std::copysign(1, angularError) * (M_TWOPI - abs(angularError));
    }

    double lateralOutput = (lateralPID->calculate(linearError, 0) - 10) * (isForward ? -1 : 1);
    double turnOutput = -turnPID->calculate(angularError, 0);

    std::cout << "Linear Error: " << linearError << " | Angular Error: " << Pose::radToDeg(angularError) << " | Lateral Output: " << lateralOutput << " | Turn Output: " << turnOutput << std::endl;
        
    double leftOutput = lateralOutput - turnOutput;
    double rightOutput = lateralOutput + turnOutput;

    tank(leftOutput, rightOutput);
}

/**
 * @brief Move the robot to a specific position using PID control.
 * @param targetPose The target pose to move to.
 */
void DifferentialChassis::moveToPose(Pose targetPose, bool isForward) {
    while (pose->distanceTo(targetPose) > .5) {
        moveToPoseStep(targetPose, isForward);

		pros::delay(100);
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
