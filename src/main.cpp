#include "main.h"

PIDController lateralPID(1, 0.0, 0);
PIDController turnPID(2, 0.0, 0);

// Bot measurements
double wheel_diameter = 3.25;
double track_width = 10.75;
double gear_ratio = 3.0/4.0;

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
pros::MotorGroup rightMotors({1, 2, -3});
pros::MotorGroup leftMotors({-10, -9, 8});
pros::MotorGroup intakeMotors({6});

// Drivetrain
DifferentialDrivetrain drivetrain(&leftMotors, &rightMotors, wheel_diameter, track_width, gear_ratio);

// Tracking Wheel
pros::IMU imu(7);
TrackingWheel horizontalTrackingWheel(5, 2.08, 0, WheelPosition::BACK);
TrackingWheel verticalTrackingWheel(-4, 2.08, 0.25, WheelPosition::LEFT);

// Odometry
Odometry odometry(&verticalTrackingWheel, NULL, &horizontalTrackingWheel, &imu);

// Chassis
DifferentialChassis chassis(&drivetrain, &odometry, &lateralPID, &turnPID);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
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
void autonomous() {
	chassis.moveToPose(Pose(24, 6, 0));
}

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
	controller.clear();
	while (true) {
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);


		// chassis.arcade(leftY, rightX);

		// if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
		// 	autonomous();
		// }

		
		// std::cout << "STICK: " << val << std::endl;
		// std::cout << "ANGLE: " << std::to_string(imu.get_yaw()) << std::endl;

		pros::delay(1000);
	}
}
