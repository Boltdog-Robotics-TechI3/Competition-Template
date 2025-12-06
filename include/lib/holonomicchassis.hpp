#pragma once

#include "holonomicdrivetrain.hpp"
#include "chassis.hpp"
#include "odometry.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class HolonomicChassis : public Chassis {
    public:
        /**
         * @brief Construct a new HolonomicChassis object with full odometry and autonomous capabilities.
         * @param drivetrain Pointer to the holonomic drivetrain.
         * @param odometry Pointer to the odometry.
         * @param lateralPID Pointer to the lateral PID controller.
         * @param turnPID Pointer to the turn PID controller.
         */
        HolonomicChassis(HolonomicDrivetrain *drivetrain, Odometry *odometry, PIDController *lateralPID, PIDController *turnPID)
        : Chassis(drivetrain, odometry, lateralPID, turnPID) {}

        /**
         * @brief Construct a new HolonomicChassis object with a drivetrain and odometry. 
         * This HolonomicChassis will have full odometry capabilities, but will not have autonomous features.
         * @param drivetrain Pointer to the holonomic drivetrain.
         * @param odometry Pointer to the odometry.
         */
        HolonomicChassis(HolonomicDrivetrain *drivetrain, Odometry *odometry)
        : Chassis(drivetrain, odometry) {}

        /**
         * @brief Construct a new HolonomicChassis object with a drivetrain and PID controllers. 
         * This HolonomicChassis will not have odometry capabilities, but will have basic autonomous capabilities.
         * @param drivetrain Pointer to the holonomic drivetrain.
         * @param lateralPID Pointer to the lateral PID controller.
         * @param turnPID Pointer to the turn PID controller.
         */
        HolonomicChassis(HolonomicDrivetrain *drivetrain, PIDController *lateralPID, PIDController *turnPID) 
        : Chassis(drivetrain, lateralPID, turnPID) {}

        /**
         * @brief Construct a new HolonomicChassis object with only a drivetrain. 
         * This HolonomicChassis will not have odometry capabilities nor autonomous features.
         * @param drivetrain Pointer to the holonomic drivetrain.
         */
        HolonomicChassis(HolonomicDrivetrain *drivetrain) 
        : Chassis(drivetrain) {}

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
         * @brief Move the robot in field-centric mode using joystick inputs with direct heading control.
         * The left joystick controls translation, and the right joystick controls the angle the robot is facing.
         * @param leftX The x-value of the left joystick.
         * @param leftY The y-value of the left joystick.
        *  @param rightX The x-value of the right joystick.
         * @param rightY The y-value of the right joystick.
         */
        void fieldCentricHeadingDrive(int leftX, int leftY, int rightX, int rightY);
        
        /**
         * @brief Move the robot towards a specific position using a single step of PID control.
         * 
         * @note This method is intended to be called repeatedly in a loop until the target position is reached.
         * Use moveToPose() for a blocking call that handles the loop internally and if the target pose won't change during the loop.
         * Use this method if your target position may change dynamically.
         * 
         * @note This method will obey the angle of the target pose while driving to the x and y coordinates.
         * 
         * @param targetPose The target pose to move to.
         */
        void moveToPoseStep(Pose targetPose) override;

        /**
         * @brief Move the robot to a specific position using PID control.
         * 
         * @note This method will obey the angle of the target pose while driving to the x and y coordinates.
         * 
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