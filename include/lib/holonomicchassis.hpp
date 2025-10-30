#pragma once

#include "holonomicdrivetrain.hpp"
#include "chassis.hpp"
#include "odometry.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class HolonomicChassis : public Chassis {
    public:
        enum DriveMode {
            FIELD_CENTRIC,
            ROBOT_CENTRIC
        };

        DriveMode driveMode = FIELD_CENTRIC;

        HolonomicChassis(HolonomicDrivetrain *drivetrain, Odometry *odometry) 
        : Chassis(drivetrain, odometry) {}

        HolonomicChassis(HolonomicDrivetrain *drivetrain) 
        : Chassis(drivetrain, nullptr) {}

        /** 
         * @brief Sets the drive mode for the Holonomic chassis.
         * @param mode The drive mode to set.
         */
        void setDriveMode(DriveMode mode) {
            driveMode = mode;
        }

        /**
         * @brief Gets the current drive mode of the Holonomic chassis.
         * @return The current drive mode.
         */
        DriveMode getDriveMode() const {
            return driveMode;
        }

        void driveAngle(double angle, int transSpeed, int rotSpeed);

        void fieldCentricDrive(int leftX, int leftY, int rightX);

        void robotCentricDrive(int leftX, int leftY, int rightX);

        void drive(int left, int right) override;

        void drive(int leftX, int leftY, int rightX) override;
        
        /**
         * @brief Move the robot to a specific position using PID control.
         * @param targetPose The target pose to move to.
         */
        // void moveToPose(Pose targetPose) override;
};