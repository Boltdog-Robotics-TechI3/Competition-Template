#include "main.h"


// Bot measurements
float wheel_diameter = 3.25;
float track_width = 10.75;
float gear_ratio = 3.0/4.0;

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
pros::MotorGroup rightMotors({11, 12, -13});
pros::MotorGroup leftMotors({18, -19, -20});

// Drivetrain
DifferentialDrivetrain drivetrain(&leftMotors, &rightMotors, wheel_diameter, track_width, gear_ratio);

// Tracking Wheel
pros::IMU gyro(15);
// TrackingWheel horizontalTrackingWheel(-17, 2.08, 4.375, WheelPosition::HORIZONTAL);

TrackingWheel horizontalTrackingWheel(-17, 2.08, 1, WheelPosition::HORIZONTAL);
TrackingWheel verticalTrackingWheel(-16, 2.08, 1.875, WheelPosition::VERTICAL);

// Odometry
OdomSensors odometry(&verticalTrackingWheel, &horizontalTrackingWheel, &gyro);

// Unscented Kalman Filter
UKF_Odom filter(&odometry);

// Chassis
DifferentialChassis chassis(&drivetrain, &odometry, &filter);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	gyro.reset(true);
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
	chassis.startTracking();
	controller.clear();

	while (true) {
		// auto angularVelocities = gyro.get_gyro_rate();
		// std::cout << angularVelocities.z << std::endl;

		int throttle = controller.get_analog(ANALOG_LEFT_Y);
		int turn = controller.get_analog(ANALOG_RIGHT_X);
		chassis.arcade(throttle,turn);

		// std::cout << "Vel X: " << odometry.getBodyVelX() << std::endl;
		// std::cout << "Vel Y: " << verticalTrackingWheel.getWheelVelocity() << std::endl;
			// std::cout << "Vel Y: " << horizontalTrackingWheel.getDistance() << std::endl;


		// std::cout << "Vel X: " << odometry.getBodyVelX() << std::endl;
		// std::cout << "Vel Y: " << odometry.getBodyVelY() << std::endl;
		controller.set_text(0, 0, chassis.getPose());.
		
		// controller.print(0, 0, "body Vel: %.2f", odometry.getBodyVelY());
		// controller.print(1, 0, "Vel: %.2f", verticalTrackingWheel.getWheelVelocity());

		pros::delay(20);
	}
}
