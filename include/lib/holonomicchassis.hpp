#pragma once

#include "holonomicdrivetrain.hpp"
#include "chassis.hpp"
#include "odometry.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class HolonomicChassis : public Chassis {
    public:
        HolonomicChassis(HolonomicDrivetrain *drivetrain, Odometry *odometry) 
        : Chassis(drivetrain, odometry) {}

        HolonomicChassis(HolonomicDrivetrain *drivetrain) 
        : Chassis(drivetrain, nullptr) {}

        /**
        * @brief Drive the robot at a specific angle with translational and rotational speeds.
        * @param angle The angle to drive in radians.
        * @param transSpeed The translational speed.
        * @param rotSpeed The rotational speed.
        */
        void driveAngle(double angle, int transSpeed, int rotSpeed);

        /**
         * @brief Move the robot in field-centric mode using joystick inputs.
         * @param leftX The x-value of the left joystick.
         * @param leftY The y-value of the left joystick.
         * @param rightX The x-value of the right joystick.
         */
        void fieldCentricDrive(int leftX, int leftY, int rightX);

        /**
         * @brief Move the robot in robot-centric mode using joystick inputs.
         * @param leftX The x-value of the left joystick.
         * @param leftY The y-value of the left joystick.
         * @param rightX The x-value of the right joystick.
         */
        void robotCentricDrive(int leftX, int leftY, int rightX);

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