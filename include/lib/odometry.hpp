#pragma once

#include <array>
#include "pros/imu.hpp"
#include "trackingwheel.hpp"

class Odometry {
    private:   
        TrackingWheel *leftWheel;
        TrackingWheel *rightWheel;
        TrackingWheel *backWheel;
        pros::IMU *imu;

        friend class Chassis;
    public:
        /**
         * @brief Construct a new Odometry object.
         * @param leftWheel Pointer to the left tracking wheel.
         * @param rightWheel Pointer to the right tracking wheel.
         * @param backWheel Pointer to the back tracking wheel.
         * @param imu Pointer to the IMU sensor.
         */
        Odometry(TrackingWheel *leftWheel, TrackingWheel *rightWheel, TrackingWheel *backWheel, pros::IMU *imu)
        : leftWheel(leftWheel), rightWheel(rightWheel), backWheel(backWheel), imu(imu) {}

        /**
         * @brief Construct a new Odometry object without an IMU.
         * @param leftWheel Pointer to the left tracking wheel.
         * @param rightWheel Pointer to the right tracking wheel.
         * @param backWheel Pointer to the back tracking wheel.
         */
        Odometry(TrackingWheel *leftWheel, TrackingWheel *rightWheel, TrackingWheel *backWheel) 
        : leftWheel(leftWheel), rightWheel(rightWheel), backWheel(backWheel), imu(nullptr) {}

        /**
         * @brief Construct a new Odometry object with only an IMU.
         * @param imu Pointer to the IMU sensor.
         */
        Odometry(pros::IMU *imu) : leftWheel(nullptr), rightWheel(nullptr), backWheel(nullptr), imu(imu) {}

        /**
         * @brief Construct a new Odometry object with no sensors.
         */
        Odometry() : leftWheel(nullptr), rightWheel(nullptr), backWheel(nullptr), imu(nullptr) {}

        /**
         * @brief Resets all odometry sensors to their initial state.
         */
        void reset();

        /**
         * @brief Get the current readings from the odometry sensors.
         * @return An array containing the left, right, and back wheel distances (in inches), and the IMU heading (in radians, if available).
         */
        std::array<double, 4> getReadings();

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
};