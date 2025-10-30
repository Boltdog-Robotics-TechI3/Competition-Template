#include "main.h"

double wheelDiameter = 3.25;
double trackWidth = 12.125;
double gearRatio = 1.0/1.0;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup frontLeftMotors({-4, 5});
pros::MotorGroup frontRightMotors({-9, 10});
pros::MotorGroup backLeftMotors({-11, 12});
pros::MotorGroup backRightMotors({-19, 20});

pros::IMU gyro(2); 

// TrackingWheel backWheel(5, 2, 0, WheelPosition::BACK); 
// TrackingWheel leftWheel(4, 2, 0, WheelPosition::LEFT); 

HolonomicDrivetrain drivetrain = HolonomicDrivetrain(&frontLeftMotors, &frontRightMotors, 
													 &backLeftMotors, &backRightMotors, 
													 wheelDiameter, trackWidth, gearRatio);
Odometry odometry = Odometry(&gyro);

HolonomicChassis chassis = HolonomicChassis(&drivetrain, &odometry);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	while (gyro.is_calibrating());

	chassis.reset();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (!false) {
		chassis.drive(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),
					  controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
					  controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

		// controller.set_text(0, 0, chassis.getPose().to_string());

		pros::delay(20);
	}
}