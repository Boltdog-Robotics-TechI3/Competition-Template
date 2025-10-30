#pragma once

#include "differentialdrivetrain.hpp"
#include "chassis.hpp"
#include "odometry.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class DifferentialChassis : public Chassis {
    protected:
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

    public:
        enum DriveMode {
            ARCADE,
            TANK
        };

        DriveMode driveMode = ARCADE;

        DifferentialChassis(DifferentialDrivetrain *drivetrain, Odometry *odometry) 
        : Chassis(drivetrain, odometry) {}

        DifferentialChassis(DifferentialDrivetrain *drivetrain) 
        : Chassis(drivetrain, nullptr) {}

        /** 
         * @brief Sets the drive mode for the differential chassis.
         * @param mode The drive mode to set.
         */
        void setDriveMode(DriveMode mode) {
            driveMode = mode;
        }

        /**
         * @brief Gets the current drive mode of the differential chassis.
         * @return The current drive mode.
         */
        DriveMode getDriveMode() const {
            return driveMode;
        }

        void drive(int left, int right) override;

        void drive(int leftX, int leftY, int rightX) override;
        
        /**
         * @brief Move the robot to a specific position using PID control.
         * @param targetPose The target pose to move to.
         */
        // void moveToPose(Pose targetPose) override;
};