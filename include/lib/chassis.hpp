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
        PIDController *lateralPID;
        PIDController *turnPID;

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
        /**
         * @brief Construct a new Chassis object.
         * @param drivetrain Pointer to the drivetrain object controlling the motors.
         * @param odometry Pointer to the odometry object for tracking position. If nullptr, odometry is disabled.
         */
        Chassis(Drivetrain *drivetrain, Odometry *odometry);

        /**
         * @brief Construct a new Chassis object without odometry.
         * @param drivetrain Pointer to the drivetrain object controlling the motors.
         */
        Chassis(Drivetrain *drivetrain) : drivetrain(drivetrain), odometry(nullptr) {}

        /**
         * @brief Scales an input value based on the selected input scaling method.
         * @param input The input value to scale (-127 to 127). 
         * @return The scaled input value.
         */
        double scaleInput(int input);

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

        Chassis(Drivetrain *drivetrain, Odometry *odometry)
        : drivetrain(drivetrain), odometry(odometry), pose(new Pose()) {}
        Chassis(Drivetrain *drivetrain) 
        : drivetrain(drivetrain), odometry(nullptr) {}
 
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
        void virtual moveToPose(Pose targetPose) = 0;

        /**
         * @brief Turn the robot to a specific angle using PID control.
         * 0 Degrees is facing "forward" from the starting orientation.
         * 
         * @param targetAngle The target angle to turn to (in degrees).
         */
        void virtual turnAngle(double targetAngle) = 0;
};