#pragma once

#include <array>
#include "pros/imu.hpp"
#include "tracking_wheel.hpp"

class OdomSensors {
    private:   
        TrackingWheel *verticalWheel;
        TrackingWheel *horizontalWheel;
        pros::IMU *imu;

        float bodyVelocityX = 0;
        float bodyVelocityY = 0;

        int previousTime = 0;
        int currentTime = 0;

        friend class Chassis;
    public:
        /**
         * @brief Construct a new OdomSensors object.
         * @param verticalWheel Pointer to the vertical tracking wheel.
         * @param horizontalWheel Pointer to the horizontal tracking wheel.
         * @param imu Pointer to the IMU sensor.
         */
        OdomSensors(TrackingWheel *verticalWheel, TrackingWheel *horizontalWheel, pros::IMU *imu)
        : verticalWheel(verticalWheel), horizontalWheel(horizontalWheel), imu(imu) {
            previousTime = pros::millis();
            currentTime = pros::millis();
        }

        /**
         * @brief Construct a new OdomSensors object without an IMU.
         * @param verticalWheel Pointer to the vertical tracking wheel.
         * @param horizontalWheel Pointer to the horizontal tracking wheel.
         */
        OdomSensors(TrackingWheel *verticalWheel, TrackingWheel *horizontalWheel) 
        : verticalWheel(verticalWheel), horizontalWheel(horizontalWheel), imu(nullptr) {
            previousTime = pros::millis();
            currentTime = pros::millis();
        }

        /**
         * @brief Construct a new OdomSensors object with only an IMU.
         * @param imu Pointer to the IMU sensor.
         */
        OdomSensors(pros::IMU *imu) : verticalWheel(nullptr), horizontalWheel(nullptr), imu(imu) {
            previousTime = pros::millis();
            currentTime = pros::millis();
        }

        /**
         * @brief Construct a new OdomSensors object with no sensors.
         */
        OdomSensors() : verticalWheel(nullptr), horizontalWheel(nullptr), imu(nullptr) {
            previousTime = pros::millis();
            currentTime = pros::millis();
        }

        /**
         * @brief Resets all odometry sensors to their initial state.
         */
        void reset();

        /**
         * @brief Calculates values that use the sensor data, such as body frame velocities from tracking wheels.
         */
        void update();


        /**
         * @brief Get the current readings from the odometry sensors.
         * @return An array containing the vertical and horizontal wheel +distances (in inches), and the IMU heading (in radians, if available).
         */
        std::array<float, 3> getReadings();

        /** 
         * @brief Get the current rotation from the IMU.
         * @return The current rotation in radians. If no IMU is present, returns 0.
         */
        float getRotationRadians();

        /** 
         * @brief Get the current rotation from the IMU.
         * @return The current rotation in degrees. If no IMU is present, returns 0.
         */
        float getRotationDegrees();

        /**
         * @brief Get the angular velocity reading from the gyro.
         * @return The angular velocity in radians per second.
         */
        float getAngularVelocity();

        void calculateBodyFrameVelocities(float dt);

        float getBodyVelX() { return bodyVelocityX; }

        float getBodyVelY() { return bodyVelocityY; }
    
};