#pragma once

#include "lib/drivetrain.hpp"
#include <array>

class HolonomicDrivetrain : public Drivetrain {
    private:
        pros::MotorGroup *frontLeftModule;
        pros::MotorGroup *frontRightModule;
        pros::MotorGroup *backLeftModule;
        pros::MotorGroup *backRightModule;

        pros::MotorGroup *sideMotors;

        friend class Chassis;

    public:
        HolonomicDrivetrain(pros::MotorGroup *leftFrontModule, pros::MotorGroup *rightFrontModule,
                           pros::MotorGroup *leftBackModule, pros::MotorGroup *rightBackModule,
                           pros::MotorGroup *sideMotors,
                           double wheelDiameter, double wheelTrackWidth, double gearRatio)
        : Drivetrain(wheelDiameter, wheelTrackWidth, gearRatio),
          frontLeftModule(leftFrontModule), frontRightModule(rightFrontModule),
          backLeftModule(leftBackModule), backRightModule(rightBackModule),
          sideMotors(sideMotors) {}

        HolonomicDrivetrain(pros::MotorGroup *leftFrontModule, pros::MotorGroup *rightFrontModule,
                           pros::MotorGroup *leftBackModule, pros::MotorGroup *rightBackModule,
                           double wheelDiameter, double wheelTrackWidth, double gearRatio)
        : Drivetrain(wheelDiameter, wheelTrackWidth, gearRatio),
          frontLeftModule(leftFrontModule), frontRightModule(rightFrontModule),
          backLeftModule(leftBackModule), backRightModule(rightBackModule),
          sideMotors(nullptr) {}

        HolonomicDrivetrain() : Drivetrain(0, 0, 0),
                                frontLeftModule(nullptr), frontRightModule(nullptr),
                                backLeftModule(nullptr), backRightModule(nullptr),
                                sideMotors(nullptr) {}

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