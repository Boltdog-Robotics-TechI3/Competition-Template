#pragma once

#include "pros/rotation.hpp"

enum class WheelPosition {
    LEFT,
    RIGHT,
    BACK
};

class TrackingWheel {
    private:
        pros::Rotation* encoder;
        double wheelDiameter; // in inches
        double offset;
        WheelPosition orientation;
        double lastPosition = 0.0; // in inches
        double totalDistance = 0.0; // in inches
        friend class Chassis;
        
    public:
        /**
         * @brief Construct a new Tracking Wheel object.
         * @param port The port number the tracking wheel's encoder is connected to.
         * @param wheelDiameter The diameter of the tracking wheel in inches.
         * @param offset The offset of the tracking wheel from the robot's center of rotation in inches. Positive values are forward/right, negative values are backward/left.
         * @param orientation The orientation of the tracking wheel (LEFT, RIGHT, or BACK).
         */
        TrackingWheel(int port, double wheelDiameter, double offset, WheelPosition orientation)
        : encoder(new pros::Rotation(port)), wheelDiameter(wheelDiameter), offset(offset), orientation(orientation) {}

        /**
         * @brief Resets the tracking wheel's encoder to zero.
         */
        void reset();

        /**
         * @brief Reverses the direction of the tracking wheel's encoder.
         */
        void reverse();

        /**
         * @brief Sets the last recorded position of the tracking wheel. Used for calculating distance traveled.
         * @param position The new last position in inches.
         */
        void setLastPosition(double position) { lastPosition = position; }

        /**
         * @brief Get the last recorded position of the tracking wheel.
         * @return The last position in inches.
         */
        double getLastPosition() { return lastPosition; }

        /**
         * @brief Get the current distance traveled by the tracking wheel since the last reset.
         * @return The distance in inches.
         */
        double getDistance();

        /**
         * @brief Get the current rotation of the tracking wheel.
         * @return The rotation in degrees.
         */
        double getRotations();

        /**
         * @brief Get the offset of the tracking wheel from the robot's center of rotation.
         * @return The offset in inches.
         */
        double getOffset() { return offset; }

        /**
         * @brief Get the orientation of the tracking wheel.
         * @return The orientation (LEFT, RIGHT, or BACK).
         */
        WheelPosition getOrientation() { return orientation; }

        /**
         * @brief Get the wheel diameter of the tracking wheel.
         * @return The wheel diameter in inches.
         */
        double getWheelDiameter() { return wheelDiameter; }
};