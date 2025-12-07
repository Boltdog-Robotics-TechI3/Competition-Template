#include "main.h"

// Bot measurements
double wheel_diameter = 3.25;
double track_width = 10.75;
double gear_ratio = 3.0/4.0;

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
pros::MotorGroup rightMotors({16, 17, -18});
pros::MotorGroup leftMotors({-13, -14, 15});
pros::MotorGroup intakeMotors({-19, 20});
pros::Motor ejectorMotor(-12);

// Pneumatics
pros::adi::Pneumatics hood('H', false);

// Drivetrain
DifferentialDrivetrain drivetrain(&leftMotors, &rightMotors, wheel_diameter, track_width, gear_ratio);

// Tracking Wheel
pros::IMU imu(11);
TrackingWheel horizontalTrackingWheel(2, 2.08, 0, WheelPosition::BACK);
TrackingWheel verticalTrackingWheel(1, 2.08, 0.25, WheelPosition::LEFT);

// Odometry
Odometry odometry(&verticalTrackingWheel, NULL, &horizontalTrackingWheel, &imu);

// Chassis
DifferentialChassis chassis(&drivetrain, &odometry, new PIDController(3, 0.0, 0.3), new PIDController(30, 0.0, 0.0));

PurePursuitController autobuilder(&chassis, 12);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	imu.reset(true);
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
	intakeMotors.move(100);
	// Trajectory testTrajectory = TrajectoryGenerator::generateTrajectory({
	// 	Pose(0, 0, Pose::degToRad(0)),
	// 	// Pose(6, 18, Pose::degToRad(0)),
	// 	Pose(0, 24, Pose::degToRad(-45)),
	// 	Pose(-48, 36, Pose::degToRad(-45)),
	// 	// Pose(-52, 60, Pose::degToRad(0)),
	// 	Pose(-54, 100, Pose::degToRad(0)),
	// 	// Pose(6, 80, Pose::degToRad(135)),
	// 	// Pose(12, 36, Pose::degToRad(180))

	// });

	// Trajectory testBackwardsTrajectory = TrajectoryGenerator::generateTrajectory({
	// 	Pose(-54, 100, Pose::degToRad(0)),
	// 	Pose(-48, 36, Pose::degToRad(-45)),
	// 	Pose(0, 24, Pose::degToRad(-45)),
	// 	Pose(0, 0, Pose::degToRad(0))
	// });

	Trajectory testTrajectory = TrajectoryGenerator::generateTrajectory({
		Pose(0, 0, Pose::degToRad(0)),
		// Pose(6, 18, Pose::degToRad(0)),
		Pose(0, 60, Pose::degToRad(60)),
		Pose(24, 57, Pose::degToRad(90)),
		// Pose(-52, 60, Pose::degToRad(0)),
		// Pose(-54, 100, Pose::degToRad(0)),
		// Pose(6, 80, Pose::degToRad(135)),
		// Pose(12, 36, Pose::degToRad(180))


	});

	// Trajectory testBackwardsTrajectory = TrajectoryGenerator::generateTrajectory({
	// 	Pose(0, 0, Pose::degToRad(0)),
	// 	Pose(12, -24, Pose::degToRad(-45)),
	// 	Pose(24, -36, Pose::degToRad(0)),
	// 	// Pose(0, 0, Pose::degToRad(0))
	// });

	// chassis.moveToPose(Pose(-24, 36, 0));
	autobuilder.followPath(testTrajectory, true);
	autobuilder.reset();
	// pros::delay(500);
	// autobuilder.reset();
	// autobuilder.followPath(testBackwardsTrajectory, false);

	pros::delay(1000);
	intakeMotors.move(0);
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
	while (true) {
		double leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		Pose pose = chassis.getPose();

		chassis.arcade(leftY, rightX);

		controller.print(1, 0, "X: %.2f Y: %.2f H: %.2f", pose.getX(), pose.getY(), Pose::radToDeg(pose.getTheta()));

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			autonomous();
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intakeMotors.move(-100);
			ejectorMotor.move(-100);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			intakeMotors.move(100);
			ejectorMotor.move(0);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intakeMotors.move(100);
			ejectorMotor.move(100);
		} else {
			intakeMotors.move(0);
			ejectorMotor.move(0);
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			hood.extend();
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			hood.retract();
		}



		pros::delay(20);
	}
}