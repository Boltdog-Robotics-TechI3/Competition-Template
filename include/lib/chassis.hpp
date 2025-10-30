#pragma once
#include "drivetrain.hpp"
#include "odometry.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class Chassis {
    protected:
        Drivetrain *drivetrain;
        Odometry *odometry;

        Pose *pose;
        PID *pidController;

        bool tracking = false;

        /**
         * @brief Calculate the robot's current position based on the odometry sensors. Runs constantly in parallel with other tasks.
        */
        void trackPosition();

        /**
         * @brief Starts the tracking task if it is not already running.
         */
        void startTracking() {
            tracking = true;
            pros::Task trackingTask([this]
            {
                while (true) {
                    trackPosition();
                    pros::delay(20); // avoid tight loop
                }
            });
        }

    public:
        Chassis(Drivetrain *drivetrain, Odometry *odometry)
        : drivetrain(drivetrain), odometry(odometry), pose(new Pose()) {}
        Chassis(Drivetrain *drivetrain) 
        : drivetrain(drivetrain), odometry(nullptr) {}
 
        /**
         * @brief Resets the pose and all of the robot's sensors to their initial state.
         */
        void reset();

        /**
         * @brief Forcefully stop the robot's motors.
         */
        void stop();

        /**
         * @brief Get the robot's current pose (position and orientation).
         * @return The robot's current pose.
         */
        Pose getPose() const;

        /**
         * @brief Set the robot's current pose (position and orientation).
         * @param newPose The new pose to set.
         */
        void setPose(Pose newPose);

        /**
         * @brief Set the robot's current pose (position and orientation) using individual values.
         * @param x The new x-coordinate.
         * @param y The new y-coordinate.
         * @param theta The new orientation (in radians).
         */
        void setPose(double x, double y, double theta);

        /**
         * @brief Sets the brake mode for the drivetrain.
         * @param mode The brake mode to set.
         */
        void setBrakeMode(pros::motor_brake_mode_e_t mode);

        /**
         * @brief Move the robot to a specific position using PID control.
         * @param targetPose The target pose to move to.
         */
        // void virtual moveToPose(Pose targetPose);

        void virtual drive(int left, int right) = 0;

        void virtual drive(int leftX, int leftY, int rightX) = 0;
};