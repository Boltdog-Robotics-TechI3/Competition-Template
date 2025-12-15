#pragma once
// Libraries
#include "api.h"
#include "lib/api.hpp"

// Bot measurements
inline double wheel_diameter = 3.25;
inline double track_width = 10.75;
inline double gear_ratio = 3.0/4.0;

// Controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
inline pros::MotorGroup rightMotors({1, 2, -3});
inline pros::MotorGroup leftMotors({-10, -9, 8});
inline pros::MotorGroup intakeMotors({6});

// Drivetrain
inline Drivetrain drivetrain(&leftMotors, &rightMotors, wheel_diameter, track_width, gear_ratio);

// Tracking Wheel
inline pros::IMU imu(7);
inline TrackingWheel horizontalTrackingWheel(5, 2.08, 0, WheelPosition::BACK);
inline TrackingWheel verticalTrackingWheel(-4, 2.08, 0.25, WheelPosition::LEFT);

// Odometry
inline Odometry odometry(&verticalTrackingWheel, NULL, &horizontalTrackingWheel, &imu);

// Chassis
inline Chassis chassis(&drivetrain, &odometry);
