#pragma once

#include "pros/motor_group.hpp"
#include <vector>
#include <initializer_list>

class Drivetrain {
    protected: 
        double wheelDiameter;
        double wheelTrackWidth;
        double gearRatio;

    public:

        Drivetrain(double wheelDiameter, double wheelTrackWidth, double gearRatio)
        : wheelDiameter(wheelDiameter), wheelTrackWidth(wheelTrackWidth), gearRatio(gearRatio) {}

         /**
         * @brief Get the wheel diameter.
         * @return The wheel diameter in inches.
         */
        double getWheelDiameter() const { return wheelDiameter; }

        /**
         * @brief Set the wheel diameter.
         * @param diameter The new wheel diameter in inches.
         */
        void setWheelDiameter(double diameter) { wheelDiameter = diameter; }

        /**
         * @brief Get the wheel track width (distance between left and right wheels).
         * @return The wheel track width in inches.
         */
        double getWheelTrackWidth() const { return wheelTrackWidth; }
        
        /**
         * @brief Set the wheel track width (distance between left and right wheels).
         * @param width The new wheel track width in inches.
         */
        void setWheelTrackWidth(double width) { wheelTrackWidth = width; }

        /**
         * @brief Get the gear ratio (output (wheel) speed / input (motor) speed).
         * @return The gear ratio.
         */
        double getGearRatio() const { return gearRatio; }
        /**
         * @brief Set the gear ratio (output (wheel) speed / input (motor) speed).
         * @param ratio The new gear ratio.
         */
        void setGearRatio(double ratio) { gearRatio = ratio; }

        /**
         * Sets the brake mode of the drivetrain.
         */
        void virtual setBrakeMode(pros::motor_brake_mode_e_t mode) = 0;

        /** 
         * Sets the current limit for all motors on the drivetrain.
         * 
         * @param limit The current limit in milliamps.
        */
        void virtual setCurrentLimit(int32_t limit) = 0;

        /**
         * Returns the brake mode of the drivetrain.
         */
        pros::v5::MotorBrake virtual getBrakeMode() = 0;

        /**
         * Returns a list of the motors' temperatures.
         */
        std::vector<std::vector<double>> virtual getTemperatures() = 0;

        /**
         * Returns a list of the motors' current draws.
         */
        std::vector<std::vector<int32_t>> virtual getCurrentDraws() = 0;

        /**
         * Returns a list of the motors' voltages.
         */
        std::vector<std::vector<int32_t>> virtual getVoltages() = 0;

        /**
         * Returns a list of the motors' velocities.
         */
        std::vector<std::vector<double>> virtual getVelocities() = 0;

        /** 
         * Returns a list of the motor groups on this drivetrain.
         */
        std::vector<pros::MotorGroup*> virtual getMotors() = 0;

        /**
         * Sets the speeds of the motors based on the speeds given
         */
        void virtual setMotorSpeeds(std::initializer_list<int> speeds) = 0;
};
