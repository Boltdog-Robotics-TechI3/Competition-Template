#pragma once

#include "differentialdrivetrain.hpp"
#include "chassis.hpp"
#include "odometry.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class DifferentialChassis : public Chassis {
    public:
        DifferentialChassis(DifferentialDrivetrain *drivetrain, Odometry *odometry) 
        : Chassis(drivetrain, odometry) {}

        DifferentialChassis(DifferentialDrivetrain *drivetrain) 
        : Chassis(drivetrain, nullptr) {}

         /**
         * @brief Move the robot in arcade mode. The left joystick controls the forward/backward movement, and the right joystick controls the rotation.
         * @param leftY The value of the left joystick (forward/backward movement).
         * @param rightX The value of the right joystick (rotation).
         */
        void arcade(int leftY, int rightX);

        /**
         * @brief Move the robot in tank mode. The left joystick controls the left side motors, and the right joystick controls the right side motors.
         * @param leftY The value of the left joystick (left side motors).
         * @param rightY The value of the right joystick (right side motors).
         */
        void tank(int leftY, int rightY);

        /**
         * @brief Move the robot to a specific position using PID control.
         * @param targetPose The target pose to move to.
         */
        void moveToPose(Pose targetPose) override;

        /**
         * @brief Turn the robot to a specific angle using PID control.
         * 0 Degrees is facing "forward" from the starting orientation.
         * 
         * @param targetAngle The target angle to turn to (in degrees).
         */
        void turnAngle(double targetAngle) override;
};