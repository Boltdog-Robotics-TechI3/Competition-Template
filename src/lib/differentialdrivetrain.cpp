#include "lib/differentialdrivetrain.hpp"

/**
 * Sets the brake mode of this differential drivetrain.
 * 
 * @param mode The brake mode to set.
 */
void DifferentialDrivetrain::setBrakeMode(pros::motor_brake_mode_e_t mode) {
    leftMotors->set_brake_mode(mode);
    rightMotors->set_brake_mode(mode);
}

/**
 * Returns the brake mode of this differential drivetrain.
 * 
 * @return The brake mode of the drivetrain.
 */
pros::v5::MotorBrake DifferentialDrivetrain::getBrakeMode() {
    return leftMotors->get_brake_mode();
};

/** 
 * Sets the current limit for all motors on the drivetrain.
 * 
 * @param limit The current limit in milliamps.
*/
void DifferentialDrivetrain::setCurrentLimit(int32_t limit) {
    leftMotors->set_current_limit(limit);
    rightMotors->set_current_limit(limit);
}

/**
 * Returns a list of the motors' temperatures.
 * The first argument is a list of the left motors' temperatures.
 * The second argument is a list of the right motors' temperatures.
 * 
 * @return A 2D vector containing the temperatures of the left and right motors.
 */
std::vector<std::vector<double>> DifferentialDrivetrain::getTemperatures() {
    std::vector<std::vector<double>> temperatures;
    temperatures.push_back(leftMotors->get_temperature_all());
    temperatures.push_back(rightMotors->get_temperature_all());

    return temperatures;
}  

/**
 * Returns a list of the motors' current draws.
 * The first argument is a list of the left motors' current draws.
 * The second argument is a list of the right motors' current draws.
 * 
 * @return A 2D vector containing the current draws of the left and right motors.
 */
std::vector<std::vector<int32_t>> DifferentialDrivetrain::getCurrentDraws() {
    std::vector<std::vector<int32_t>> currentDraws;
    currentDraws.push_back(leftMotors->get_current_draw_all());
    currentDraws.push_back(rightMotors->get_current_draw_all());

    return currentDraws;
}

/**
 * Returns a list of the motors' voltages.
 * The first argument is a list of the left motors' voltages.
 * The second argument is a list of the right motors' voltages.
 * 
 * @return A 2D vector containing the voltages of the left and right motors.
 */
std::vector<std::vector<int32_t>> DifferentialDrivetrain::getVoltages() {
    std::vector<std::vector<int32_t>> voltages;
    voltages.push_back(leftMotors->get_voltage_all());
    voltages.push_back(rightMotors->get_voltage_all());

    return voltages;
}

/**
 * Returns a list of the motors' velocities.
 * The first argument is a list of the left motors' velocities.
 * The second argument is a list of the right motors' velocities.
 * 
 * @return A 2D vector containing the velocities of the left and right motors.
 */
std::vector<std::vector<double>> DifferentialDrivetrain::getVelocities() {
    std::vector<std::vector<double>> velocities;
    velocities.push_back(leftMotors->get_actual_velocity_all());
    velocities.push_back(rightMotors->get_actual_velocity_all());

    return velocities;
}

/** 
 * Returns a list of the motor groups on this drivetrain.
 * The first motor group represents the left drive motors.
 * The second motor group represents the right drive motors.
 * 
 * @return A vector containing the left and right motor groups.
*/
std::vector<pros::MotorGroup*> DifferentialDrivetrain::getMotors() {
    std::vector<pros::MotorGroup*> motors;
    motors.push_back(leftMotors);
    motors.push_back(rightMotors);
    return motors;
}

/**
 * Sets the speeds of the motors based on the speeds given.
 * The first speed is assigned to the left motors.
 * The second speed is assigned to the right motors.
 * 
 * @param speeds An array containing the speeds for the left and right motors.
 */
void DifferentialDrivetrain::setMotorSpeeds(std::initializer_list<int> speeds) {
    leftMotors->move(speeds.begin()[0]);
    rightMotors->move(speeds.begin()[1]);
}


