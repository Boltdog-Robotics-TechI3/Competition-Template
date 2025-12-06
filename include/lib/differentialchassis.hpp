#pragma once

#include "differentialdrivetrain.hpp"
#include "chassis.hpp"
#include "odometry.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class DifferentialChassis : public Chassis {
    public:
        /**
         * @brief Construct a new DifferentialChassis object with full odometry and autonomous capabilities.
         * @param drivetrain Pointer to the differential drivetrain.
         * @param odometry Pointer to the odometry.
         * @param lateralPID Pointer to the lateral PID controller.
         * @param turnPID Pointer to the turn PID controller.
         */
        DifferentialChassis(DifferentialDrivetrain *drivetrain, Odometry *odometry, PIDController *lateralPID, PIDController *turnPID)
        : Chassis(drivetrain, odometry, lateralPID, turnPID) {}

        /**
         * @brief Construct a new DifferentialChassis object with a drivetrain and odometry. 
         * This DifferentialChassis will have full odometry capabilities, but will not have autonomous features.
         * @param drivetrain Pointer to the differential drivetrain.
         * @param odometry Pointer to the odometry.
         */
        DifferentialChassis(DifferentialDrivetrain *drivetrain, Odometry *odometry)
        : Chassis(drivetrain, odometry) {}

        /**
         * @brief Construct a new DifferentialChassis object with a drivetrain and PID controllers. 
         * This DifferentialChassis will not have odometry capabilities, but will have basic autonomous capabilities.
         * @param drivetrain Pointer to the differential drivetrain.
         * @param lateralPID Pointer to the lateral PID controller.
         * @param turnPID Pointer to the turn PID controller.
         */
        DifferentialChassis(DifferentialDrivetrain *drivetrain, PIDController *lateralPID, PIDController *turnPID) 
        : Chassis(drivetrain, lateralPID, turnPID) {}

        /**
         * @brief Construct a new DifferentialChassis object with only a drivetrain. 
         * This DifferentialChassis will not have odometry capabilities nor autonomous features.
         * @param drivetrain Pointer to the differential drivetrain.
         */
        DifferentialChassis(DifferentialDrivetrain *drivetrain) 
        : Chassis(drivetrain) {}

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
         * @brief Move the robot towards a specific position using a single step of PID control.
         * 
         * @note This method is intended to be called repeatedly in a loop until the target position is reached.
         * Use moveToPose() for a blocking call that handles the loop internally and if the target pose won't change during the loop.
         * Use this method if your target position may change dynamically.
         * 
         * @note This method ignores the angle of the target pose and only drives to the x and y coordinates.
         * 
         * @param targetPose The target pose to move to.
         */
        void moveToPoseStep(Pose targetPose) override;

        /**
         * @brief Move the robot to a specific position using PID control. This method blocks until the target position is reached.
         * 
         * @note This method ignores the angle of the target pose and only drives to the x and y coordinates.
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