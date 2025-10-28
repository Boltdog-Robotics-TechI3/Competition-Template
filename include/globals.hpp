#pragma once
#include "api.h"

const double wheelDiameter = 2.75;
const double trackWidth = 12.5;
const double gearRatio = 36.0 / 48.0;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftDrive({-18, -19, 20});
pros::MotorGroup rightDrive({8, 9, -10});

pros::Motor intake(4);

Drivetrain drivetrain(&leftDrive, &rightDrive, wheelDiameter, trackWidth, gearRatio);

TrackingWheel verticalTrackingWheel(7, 2, 0, WheelPosition::LEFT);
TrackingWheel horizontalTrackingWheel(6, 2, 0, WheelPosition::BACK);

pros::IMU gyro(5);

Odometry odometry(&verticalTrackingWheel, NULL, &horizontalTrackingWheel, &gyro);

Chassis chassis(&drivetrain, &odometry);