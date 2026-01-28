#pragma once
#include <iostream>

#include "drivetrain.hpp"
#include "odom_sensors.hpp"
#include "ukf_odom.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"
#include "Eigen/Eigen"

class Chassis {
    protected:
        Drivetrain *drivetrain;
        OdomSensors *odometry;
        UKF_Odom *filter;

        Pose pose;
        PIDController *lateralPID;
        PIDController *turnPID;

        bool tracking = false;

        /**
         * @brief Calculate the robot's current position based on the odometry sensors. Runs constantly in parallel with other tasks.
        */
        void trackPosition();

        /**
         * @brief Calculate the robot's current position using an Unscented Kalman Filter. Runs constantly in parallel with other tasks.
         */
        void updateFilter();

        /**
         * @brief Runs the tracking task if it is not already running.
         */
        void runTracking() {
            pros::Task trackingTask([this]
            {
                while (true) {
                    while (tracking) {
                        odometry->update();
                        if (filter) {
                            updateFilter();
                        }
                        else {
                            trackPosition();
                        }
                        pros::delay(20); // avoid tight loop
                    }
                    pros::delay(20);
                }

                tracking = false;
                delete filter;
            });
        }

        /**
         * @brief Scales an input value based on the selected input scaling method.
         * @param input The input value to scale (-127 to 127). 
         * @return The scaled input value.
         */
        float scaleInput(int input);

    public:
        enum InputScale {
            LINEAR,
            CUBIC,
            QUINTIC,
            SIN,
            SINSQUARED,
            TAN,
            XTAN
        };

        InputScale inputScale = LINEAR;

        Chassis(Drivetrain *drivetrain, OdomSensors *odometry, UKF_Odom *filter)
        : drivetrain(drivetrain), odometry(odometry), filter(filter), pose(Pose{}) {}
        Chassis(Drivetrain *drivetrain, OdomSensors *odometry)
        : drivetrain(drivetrain), odometry(odometry), pose(Pose{}) {}
        Chassis(Drivetrain *drivetrain) 
        : drivetrain(drivetrain), odometry(nullptr), pose(Pose{}) {}
 
        /**
         * @brief Sets the input scaling method. The input scaling affects how joystick inputs are translated to motor speeds.
         * 
         * LINEAR: Direct mapping.
         * 
         * CUBIC: Cubic curve for finer control at low speeds.
         * 
         * QUINTIC: Quintic curve for even finer control at low speeds.
         * 
         * SIN: Sine curve for smooth acceleration.
         * 
         * SINSQUARED: Sine squared curve for smooth acceleration and fine control at low speeds.
         * 
         * TAN: Tangent for aggressive acceleration. (may be unstable at high inputs)
         * 
         * XTAN: Exponential tangent curve for fine control at low speeds and aggressive at high speeds. (may be unstable at high inputs)
         * 
         * Link to a graphical representation of these curves: https://www.desmos.com/calculator/xrfbyvksxi
         * 
         * @param scale The input scaling method to set.
         */
        void setInputScale(InputScale scale);

        /**
         * @brief Resets the pose and all of the robot's sensors to their initial state.
         */
        void reset();

        /**
         * @brief Forcefully stop the robot's motors.
         */
        void stop();

        /**
         * @brief Enables pose estimation.
         */
        void startTracking();

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
        void setPose(float x, float y, float theta);

        /**
         * @brief Sets the brake mode for the drivetrain.
         * @param mode The brake mode to set.
         */
        void setBrakeMode(pros::motor_brake_mode_e_t mode);

        /**
         * @brief Move the robot to a specific position using PID control.
         * @param targetPose The target pose to move to.
         */
        void virtual moveToPose(Pose targetPose) = 0;

        /**
         * @brief Turn the robot to a specific angle using PID control.
         * 0 Degrees is facing "forward" from the starting orientation.
         * 
         * @param targetAngle The target angle to turn to (in degrees).
         */
        void virtual turnAngle(float targetAngle) = 0;
};