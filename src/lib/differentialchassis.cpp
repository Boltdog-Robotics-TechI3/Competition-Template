#include <cmath>
#include "lib/differentialchassis.hpp"
#include "pros/rtos.hpp"

void DifferentialChassis::drive(int left, int right) {
    switch (driveMode) {
        case ARCADE:
            arcade(left, right);
            break;
        case TANK:
            tank(left, right);
            break; 
        default:
            break;
    }
}

void DifferentialChassis::drive(int leftX, int leftY, int rightX) {
    switch (driveMode) {
        case ARCADE:
            arcade(leftY, rightX);
            break;
        case TANK:
            tank(leftY, rightX);
            break; 
        default:
            break;
    }
}

void DifferentialChassis::arcade(int leftY, int rightX) {
    if (drivetrain) {
        int leftPower = leftY + rightX;
        int rightPower = leftY - rightX;
        drivetrain->setMotorSpeeds({leftPower, rightPower});
    }
}

void DifferentialChassis::tank(int leftY, int rightY) {
    if (drivetrain) {
        drivetrain->setMotorSpeeds({leftY, rightY});
    }
}

// void DifferentialChassis::moveToPose(Pose targetPose) {
//     // TODO: Implement MoveTo for DifferentialChassis
// }
