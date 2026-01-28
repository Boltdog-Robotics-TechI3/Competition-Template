#pragma once
#include <string>

class Pose {
    private:
        float x; // in inches
        float y; // in inches
        float theta; // in radians 

    public:
        Pose(float x, float y, float theta) : x(x), y(y), theta(theta) {}
        Pose() : x(0), y(0), theta(0) {}

        /**
         * @brief Resets the pose to the origin (0,0) with 0 radians orientation.
         */
        void reset() { x = 0; y = 0; theta = 0; }

        float getX() const { return x; }
        float getY() const { return y; }
        float getTheta() const { return theta; }

        void setX(float x) { this->x = x; }
        void setY(float y) { this->y = y; }
        void setTheta(float theta) { this->theta = theta; }


        /**
         * @brief Sets the pose using individual values.
         * @param x The new x-coordinate.
         * @param y The new y-coordinate.
         * @param theta The new orientation (in radians).
        */
        void setPose(float x, float y, float theta) {
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
        float distanceTo(const Pose& other);

        /**
         * @brief Calculates the angle to another pose.
         * @param other The other pose to calculate the angle to.
         * @return The angle in radians.
         */
        float angleTo(const Pose& other);

        /**
         * @brief Rotates the pose by a given angle.
         * @param angle The angle to rotate by (in radians).
         * @return A new Pose object representing the rotated pose.
         */
        Pose rotate(float angle);

        std::string to_string();
};