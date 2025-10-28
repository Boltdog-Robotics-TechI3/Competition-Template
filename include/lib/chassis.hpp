#pragma once
#include "drivetrain.hpp"
#include "odometry.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class Chassis {
    private:
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
         * @brief Resets the pose and all of the robot's sensors to their initial state.
         */
        void reset();

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
         * @brief Forcefully stop the robot's motors.
         */
        void stop();

        /**
         * @brief Get the robot's current pose (position and orientation).
         * @return The robot's current pose.
         */
        Pose getPose() const { return *pose; }

        /**
         * @brief Set the robot's current pose (position and orientation).
         * @param newPose The new pose to set.
         */
        void setPose(Pose newPose) { *pose = newPose; }

        /**
         * @brief Set the robot's current pose (position and orientation) using individual values.
         * @param x The new x-coordinate.
         * @param y The new y-coordinate.
         * @param theta The new orientation (in radians).
         */
        void setPose(double x, double y, double theta) {
            pose->setX(x);
            pose->setY(y);
            pose->setTheta(theta);
        }

        void moveDistance(double distance);
        void turnAngle(double angle);
        /**
         * @brief Move the robot to a specific position using PID control.
         * @param targetPose The target pose to move to.
         */
        void moveTo(Pose targetPose);

        /**
         * @brief Set the brake mode for the drivetrain motors.
         * @param mode The brake mode to set (e.g., pros::E_MOTOR_BRAKE_COAST, pros::E_MOTOR_BRAKE_BRAKE, pros::E_MOTOR_BRAKE_HOLD).
         */
        void setBrakeMode(pros::motor_brake_mode_e_t mode) {
            if (drivetrain) {
                drivetrain->leftMotors->set_brake_mode(mode);
                drivetrain->rightMotors->set_brake_mode(mode);
            }
        }
};