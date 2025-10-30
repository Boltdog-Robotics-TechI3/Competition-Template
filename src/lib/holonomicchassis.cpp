#include <cmath>
#include "lib/holonomicchassis.hpp"
#include "pros/rtos.hpp"

void HolonomicChassis::driveAngle(double angle, int transSpeed, int rotSpeed) {
	double adjustedAngle = angle + (M_PI / 2);
	int x = cos(adjustedAngle) * transSpeed;
	int y = sin(adjustedAngle) * transSpeed;

	drivetrain->setMotorSpeeds({y+x+rotSpeed,
                                -y+x+rotSpeed,
                                y-x+rotSpeed,
                                -y-x+rotSpeed});
}

void HolonomicChassis::fieldCentricDrive(int leftX, int leftY, int rightX) {
    double y = (double)leftY;
    double x = (double)leftX;
    double r = (double)rightX;

    double targetAngle = atan2(y, x);
    double speed = sqrt(x*x + y*y);

	driveAngle(targetAngle + odometry->getRotationRadians(), speed, r);
}

void HolonomicChassis::robotCentricDrive(int leftX, int leftY, int rightX) {
    double y = (double)leftY;
    double x = (double)leftY;
    double r = (double)rightX;

    double targetAngle = atan2(y, x);
    double speed = sqrt(x*x + y*y);

    driveAngle(targetAngle, speed, r);
}

void HolonomicChassis::drive(int left, int right) {
    switch (driveMode) {
        case FIELD_CENTRIC:
            fieldCentricDrive(0, left, right);
            break;
        case ROBOT_CENTRIC:
            robotCentricDrive(0, left, right);
            break; 
        default:
            break;
    }
}

void HolonomicChassis::drive(int leftX, int leftY, int rightX) {
    switch (driveMode) {
        case FIELD_CENTRIC:
            fieldCentricDrive(leftX, leftY, rightX);
            break;
        case ROBOT_CENTRIC:
            robotCentricDrive(leftX, leftY, rightX);
            break; 
        default:
            break;
    }
}

// void HolonomicChassis::moveToPose(Pose targetPose) {
//     // TODO: Implement MoveTo for HolonomicChassis
// }



