#pragma once
#include <string>
#include <cmath>

class Pose {
    private:
        double x; // in inches
        double y; // in inches
        double theta; // in radians 

    public:
        Pose(double x, double y, double theta) : x(x), y(y), theta(theta) {}
        Pose() : x(0), y(0), theta(0) {}

        /**
         * @brief Resets the pose to the origin (0,0) with 0 radians orientation.
         */
        void reset() { x = 0; y = 0; theta = 0; }

        double getX() const { return x; }
        double getY() const { return y; }
        double getTheta() const { return theta; }

        void setX(double x) { this->x = x; }
        void setY(double y) { this->y = y; }
        void setTheta(double theta) { this->theta = theta; }


        /**
         * @brief Sets the pose using individual values.
         * @param x The new x-coordinate.
         * @param y The new y-coordinate.
         * @param theta The new orientation (in radians).
        */
        void setPose(double x, double y, double theta) {
            this->x = x;
            this->y = y;
            this->theta = theta;
        }

        /**
         * @brief Sets the pose using another Pose object.
         * @param newPose The new pose to set.
        */
        void setPose(Pose newPose);

        /**
         * @brief Calculates the distance to another pose.
         * @param other The other pose to calculate the distance to.
         * @return The distance in inches.
         */
        double distanceTo(const Pose& other);

        /**
         * @brief Calculates the angle to another pose.
         * @param other The other pose to calculate the angle to.
         * @return The angle in radians.
         */
        double angleTo(const Pose& other);

        /**
         * @brief Offsets the current pose by another pose.
         * @param other The pose to offset by.
         * @return A new Pose object representing the offset pose.
         */
        Pose offset(const Pose& other);

        /**
         * @brief Offsets the current pose by the opposite of another pose.
         * @param other The pose to offset by.
         * @return A new Pose object representing the offset pose.
         */
        Pose negativeOffset(const Pose& other);

        /**
         * @brief Rotates the pose by a given angle.
         * @param angle The angle to rotate by (in radians).
         * @return A new Pose object representing the rotated pose.
         */
        Pose rotate(double angle);

        static double degToRad(double degrees) { return degrees * (M_PI / 180.0); }
        static double radToDeg(double radians) { return radians * (180.0 / M_PI); }

        std::string to_string();
};