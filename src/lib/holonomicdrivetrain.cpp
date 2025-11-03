#include "holonomicdrivetrain.hpp"

/**
 * Sets the brake mode of this Holonomic Drivetrain.
 * 
 * @param mode The brake mode to set.
 */
void HolonomicDrivetrain::setBrakeMode(pros::motor_brake_mode_e_t mode) {
    frontLeftModule->set_brake_mode(mode);
    frontRightModule->set_brake_mode(mode);
    backLeftModule->set_brake_mode(mode);
    backRightModule->set_brake_mode(mode);

    if (sideMotors != nullptr) {
        sideMotors->set_brake_mode(mode);
    }
}

/**
 * Returns the brake mode of this Holonomic Drivetrain.
 * 
 * @return The brake mode of the drivetrain.
 */
pros::v5::MotorBrake HolonomicDrivetrain::getBrakeMode() {
    return frontLeftModule->get_brake_mode();
};

/** 
 * Sets the current limit for all motors on the drivetrain.
 * 
 * @param limit The current limit in milliamps.
*/
void HolonomicDrivetrain::setCurrentLimit(int32_t limit) {
    frontLeftModule->set_current_limit(limit);
    frontRightModule->set_current_limit(limit);
    backLeftModule->set_current_limit(limit);
    backRightModule->set_current_limit(limit);

    if (sideMotors != nullptr) {
        sideMotors->set_current_limit(limit);
    }
}

/**
 * Returns a list of the motors' temperatures.
 * The first element is a list of the front left module's temperatures.
 * The second element is a list of the front right module's temperatures.
 * The third element is a list of the back left module's temperatures.
 * The fourth element is a list of the back right module's temperatures.
 * If side motors exist, the fifth element is a list of the side motors' temperatures.
 * 
 * 
 * @return A 2D vector containing the temperatures of the motors
 */
std::vector<std::vector<double>> HolonomicDrivetrain::getTemperatures() {
    std::vector<std::vector<double>> temperatures;
    temperatures.push_back(frontLeftModule->get_temperature_all());
    temperatures.push_back(frontRightModule->get_temperature_all());
    temperatures.push_back(backLeftModule->get_temperature_all());
    temperatures.push_back(backRightModule->get_temperature_all());

    if (sideMotors != nullptr) {
        temperatures.push_back(sideMotors->get_temperature_all());
    }

    return temperatures;
}  

/**
 * Returns a list of the motors' current draws.
 * The first element is a list of the front left module's current draws.
 * The second element is a list of the front right module's current draws.
 * The third element is a list of the back left module's current draws.
 * The fourth element is a list of the back right module's current draws.
 * 
 * If side motors exist, the fifth element is a list of the side motors' current draws. 
 * 
 * @return A 2D vector containing the current draws of the motors
 */
std::vector<std::vector<int32_t>> HolonomicDrivetrain::getCurrentDraws() {
    std::vector<std::vector<int32_t>> currentDraws;
    currentDraws.push_back(frontLeftModule->get_current_draw_all());
    currentDraws.push_back(frontRightModule->get_current_draw_all());
    currentDraws.push_back(backLeftModule->get_current_draw_all());
    currentDraws.push_back(backRightModule->get_current_draw_all());

    if (sideMotors != nullptr) {
        currentDraws.push_back(sideMotors->get_current_draw_all());
    }

    return currentDraws;
}

/**
 * Returns a list of the motors' voltages.
 * The first element is a list of the front left module's voltages.
 * The second element is a list of the front right module's voltages.
 * The third element is a list of the back left module's voltages.
 * The fourth element is a list of the back right module's voltages.
 * 
 * If side motors exist, the fifth element is a list of the side motors' voltages.
 * 
 * @return A 2D vector containing the voltages of the motors
 */
std::vector<std::vector<int32_t>> HolonomicDrivetrain::getVoltages() {
    std::vector<std::vector<int32_t>> voltages;
    voltages.push_back(frontLeftModule->get_voltage_all());
    voltages.push_back(frontRightModule->get_voltage_all());
    voltages.push_back(backLeftModule->get_voltage_all());
    voltages.push_back(backRightModule->get_voltage_all());

    if (sideMotors != nullptr) {
        voltages.push_back(sideMotors->get_voltage_all());
    }

    return voltages;
}

/**
 * Returns a list of the motors' velocities.
 * The first element is a list of the front left module's velocities.
 * The second element is a list of the front right module's velocities.
 * The third element is a list of the back left module's velocities.
 * The fourth element is a list of the back right module's velocities.
 * 
 * If side motors exist, the fifth element is a list of the side motors' velocities.
 * 
 * @return A 2D vector containing the velocities of the motors
 */
std::vector<std::vector<double>> HolonomicDrivetrain::getVelocities() {
    std::vector<std::vector<double>> velocities;
    velocities.push_back(frontLeftModule->get_actual_velocity_all());
    velocities.push_back(frontRightModule->get_actual_velocity_all());
    velocities.push_back(backLeftModule->get_actual_velocity_all());
    velocities.push_back(backRightModule->get_actual_velocity_all());

    if (sideMotors != nullptr) {
        velocities.push_back(sideMotors->get_actual_velocity_all());
    }

    return velocities;
}

/** 
 * Returns a list of the motor groups on this drivetrain.
 * The first motor group represents the front left module.
 * The second motor group represents the front right module.
 * The third motor group represents the back left module.
 * The fourth motor group represents the back right module.
 * 
 * If side motors exist, the fifth motor group represents the side motors.
 * 
 * @return A vector containing the left and right motor groups.
*/
std::vector<pros::MotorGroup*> HolonomicDrivetrain::getMotors() {
    std::vector<pros::MotorGroup*> motors;
    motors.push_back(frontLeftModule);
    motors.push_back(frontRightModule);
    motors.push_back(backLeftModule);
    motors.push_back(backRightModule);

    if (sideMotors != nullptr) {
        motors.push_back(sideMotors);
    }
    return motors;
}

/**
 * Sets the speeds of the motors based on the speeds given.
 * The speeds array should contain the speeds for the front left, front right,
 * back left, back right, and optionally side motors in that order.
 * 
 * @param speeds An array containing the speeds for the motors.
 */
void HolonomicDrivetrain::setMotorSpeeds(std::initializer_list<int> speeds) {
    frontLeftModule->move(speeds.begin()[0]);
    frontRightModule->move(speeds.begin()[1]);
    backLeftModule->move(speeds.begin()[2]);
    backRightModule->move(speeds.begin()[3]);

    if (sideMotors != nullptr && speeds.size() >= 5) {
        sideMotors->move(speeds.begin()[4]);
    }
}


