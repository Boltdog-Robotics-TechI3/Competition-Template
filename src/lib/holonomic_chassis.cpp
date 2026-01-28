#include <cmath>
#include "lib/holonomic_chassis.hpp"
#include "pros/rtos.hpp"

/**
 * @brief Drive the robot at a specific angle with translational and rotational speeds.
 * @param angle The angle to drive in radians.
 * @param transSpeed The translational speed.
 * @param rotSpeed The rotational speed.
 */
void HolonomicChassis::driveAngle(float angle, int transSpeed, int rotSpeed) {
	float adjustedAngle = angle;
	int x = cos(adjustedAngle) * transSpeed;
	int y = sin(adjustedAngle) * transSpeed;

	drivetrain->setMotorSpeeds({y+x+rotSpeed,
                                -y+x+rotSpeed,
                                y-x+rotSpeed,
                                -y-x+rotSpeed});
}

/**
 * @brief Move the robot in field-centric mode using joystick inputs.
 * @param leftX The x-value of the left joystick.
 * @param leftY The y-value of the left joystick.
 * @param rightX The x-value of the right joystick.
 */
void HolonomicChassis::fieldCentricDrive(int leftX, int leftY, int rightX) {
    float y = (float)leftY;
    float x = (float)leftX;
    float r = scaleInput(rightX);

    float targetAngle = atan2(y, x);
    float speed = scaleInput(sqrt(x*x + y*y));

	driveAngle(targetAngle + odometry->getRotationRadians(), speed, r);
}

/**
 * @brief Move the robot in robot-centric mode using joystick inputs.
 * @param leftX The x-value of the left joystick.
 * @param leftY The y-value of the left joystick.
 * @param rightX The x-value of the right joystick.
 */
void HolonomicChassis::robotCentricDrive(int leftX, int leftY, int rightX) {
    float y = (float)leftY;
    float x = (float)leftX;
    float r = scaleInput(rightX);

    float targetAngle = atan2(y, x);
    float speed = scaleInput(sqrt(x*x + y*y));

    driveAngle(targetAngle, speed, r);
}

/**
 * @brief Move the robot to a specific position using PID control.
 * @param targetPose The target pose to move to.
 */
void HolonomicChassis::moveToPose(Pose targetPose) {
    // TODO: Implement MoveTo for HolonomicChassis
}

/**       
 * @brief Turn the robot to a specific angle using PID control.
 * 0 Degrees is facing "forward" from the starting orientation.
 * 
 * @param targetAngle The target angle to turn to (in degrees).
 */
void HolonomicChassis::turnAngle(float targetAngle) {
    // TODO: Implement TurnAngle for HolonomicChassis
}
