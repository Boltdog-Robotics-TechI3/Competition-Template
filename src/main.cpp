#include "main.h"
#include <vector>


pros::Controller controller(pros::E_CONTROLLER_MASTER);


// double wheelDiameter = 3.25;
// double trackWidth = 12.125;
// double gearRatio = 1.0/1.0;

// pros::MotorGroup frontLeftMotors({-4, 5});
// pros::MotorGroup frontRightMotors({-9, 10});
// pros::MotorGroup backLeftMotors({-11, 12});
// pros::MotorGroup backRightMotors({-19, 20});

// pros::IMU imu(7);

// HolonomicDrivetrain drivetrain = HolonomicDrivetrain(&frontLeftMotors, &frontRightMotors, 
// 													 &backLeftMotors, &backRightMotors, 
// 													 wheelDiameter, trackWidth, gearRatio);

// Odometry odometry = Odometry(&imu);

// HolonomicChassis chassis = HolonomicChassis(&drivetrain, &odometry);

double wheelDiameter = 3.25;
double trackWidth = 10.75;
double gearRatio = 3.0/4.0;

pros::MotorGroup rightMotors({1, 2, -3});
pros::MotorGroup leftMotors({-10, -9, 8});
pros::Motor intake(6);

pros::IMU imu(7);
TrackingWheel horizontalTrackingWheel(5, 2.08, 0, WheelPosition::BACK);
TrackingWheel verticalTrackingWheel(-4, 2.08, 0.25, WheelPosition::LEFT);

Odometry odometry(&verticalTrackingWheel, NULL, &horizontalTrackingWheel, &imu);

DifferentialDrivetrain drivetrain = DifferentialDrivetrain(&leftMotors, &rightMotors, 
														   wheelDiameter, trackWidth, gearRatio);

DifferentialChassis chassis = DifferentialChassis(&drivetrain, &odometry);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.reset();
	chassis.setInputScale(Chassis::SINSQUARED);
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
	std::vector<std::vector<int32_t>> vals;

	while (1) {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			imu.tare();
		}

		chassis.arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
					   controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

		

		// chassis.fieldCentricDrive(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),
		// 							controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
		// 							controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

		// controller.set_text(0, 0, std::to_string(odometry.getRotationDegrees()));

		std::cout << "Pose X: " << chassis.getPose().getX() << " Y: " << chassis.getPose().getY() << " Theta: " << chassis.getPose().getTheta() << std::endl;

		pros::delay(20);
	}
}