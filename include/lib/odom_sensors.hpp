#pragma once

#include <array>
#include "pros/imu.hpp"
#include "tracking_wheel.hpp"

class OdomSensors {
    private:   
        TrackingWheel *verticalWheel;
        TrackingWheel *horizontalWheel;
        pros::IMU *imu;

        double bodyVelocityX;
        double bodyVelocityY;

        int previousTime;
        int currentTime;

        void calculateBodyFrameVelocities();

        friend class Chassis;
    public:
        /**
         * @brief Construct a new OdomSensors object.
         * @param verticalWheel Pointer to the vertical tracking wheel.
         * @param horizontalWheel Pointer to the horizontal tracking wheel.
         * @param imu Pointer to the IMU sensor.
         */
        OdomSensors(TrackingWheel *verticalWheel, TrackingWheel *horizontalWheel, pros::IMU *imu)
        : verticalWheel(verticalWheel), horizontalWheel(horizontalWheel), imu(imu) {}

        /**
         * @brief Construct a new OdomSensors object without an IMU.
         * @param verticalWheel Pointer to the vertical tracking wheel.
         * @param horizontalWheel Pointer to the horizontal tracking wheel.
         */
        OdomSensors(TrackingWheel *verticalWheel, TrackingWheel *horizontalWheel) 
        : verticalWheel(verticalWheel), horizontalWheel(horizontalWheel), imu(nullptr) {}

        /**
         * @brief Construct a new OdomSensors object with only an IMU.
         * @param imu Pointer to the IMU sensor.
         */
        OdomSensors(pros::IMU *imu) : verticalWheel(nullptr), horizontalWheel(nullptr), imu(imu) {}

        /**
         * @brief Construct a new OdomSensors object with no sensors.
         */
        OdomSensors() : verticalWheel(nullptr), horizontalWheel(nullptr), imu(nullptr) {}

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
        std::array<double, 3> getReadings();

        /** 
         * @brief Get the current rotation from the IMU.
         * @return The current rotation in radians. If no IMU is present, returns 0.
         */
        double getRotationRadians();

        /** 
         * @brief Get the current rotation from the IMU.
         * @return The current rotation in degrees. If no IMU is present, returns 0.
         */
        double getRotationDegrees();

        /**
         * @brief Get the angular velocity reading from the gyro.
         * @return The angular velocity in radians per second.
         */
        double getAngularVelocity();
    
};