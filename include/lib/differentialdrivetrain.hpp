#pragma once

#include "lib/drivetrain.hpp"
#include <array>

class DifferentialDrivetrain : public Drivetrain {
    private:
        pros::MotorGroup *leftMotors;
        pros::MotorGroup *rightMotors;
        friend class Chassis;
        
    public:
        DifferentialDrivetrain(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors, double wheelDiameter, double wheelTrackWidth, double gearRatio)
        : Drivetrain(wheelDiameter, wheelTrackWidth, gearRatio), leftMotors(leftMotors), rightMotors(rightMotors) {}

        DifferentialDrivetrain() : Drivetrain(0, 0, 0), leftMotors(nullptr), rightMotors(nullptr) {}

        /**
         * Sets the brake mode of the drivetrain.
         */
        void setBrakeMode(pros::motor_brake_mode_e_t mode) override;

        /** 
         * Sets the current limit for all motors on the drivetrain.
         * 
         * @param limit The current limit in milliamps.
        */
        void setCurrentLimit(int32_t limit) override;

        /**
         * Returns the brake mode of the drivetrain.
         */
        pros::v5::MotorBrake getBrakeMode() override;

        /**
         * Returns a list of the motors' temperatures.
         */
        std::vector<std::vector<double>> getTemperatures() override;
 
        /**
         * Returns a list of the motors' current draws.
         */
        std::vector<std::vector<int32_t>> getCurrentDraws() override;

        /**
        * Returns a list of the motors' voltages.
        */
        std::vector<std::vector<int32_t>> getVoltages() override;

        /**
         * Returns a list of the motors' velocities.
         */
        std::vector<std::vector<double>> getVelocities() override;

        /** 
         * Returns a list of the motor groups on this drivetrain.
        */
        std::vector<pros::MotorGroup*> getMotors() override;

        /**
         * Sets the speeds of the motors based on the speeds given
         */
        void setMotorSpeeds(std::initializer_list<int> speeds) override;
};