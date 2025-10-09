#include "main.h"
#include <cmath>

pros::Controller driver(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-7, -8, 9, 10});
pros::MotorGroup rightMotors({-1, -2, 3, 4});
double wheelDiameter = 3.25;
double trackWidth = 10.875;
double gearRatio = 1.0; // 1:1

pros::IMU gyro(11); // IMU on port 20

TrackingWheel backWheel(19, 2.125, 0, WheelPosition::BACK); // Back wheel on port 19
TrackingWheel leftWheel(18, 2.125, 0, WheelPosition::LEFT); // Left wheel on ports 18

Drivetrain drivetrain = Drivetrain(&leftMotors, &rightMotors, wheelDiameter, trackWidth, gearRatio);
Odometry odometry = Odometry(&leftWheel, NULL, &backWheel, &gyro); // OdomSensors with left and back wheels

Chassis chassis = Chassis(&drivetrain, &odometry);
PurePursuitController purePursuitController(&chassis, 6);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	gyro.reset(true);
	chassis.reset(); // Reset the chassis controller

	// pros::lcd::initialize();
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
	Trajectory testTrajectory = TrajectoryGenerator::generateTrajectory({
		Pose(0, 0, Pose::degToRad(0)),
		// Pose(6, 18, Pose::degToRad(0)),
		Pose(0, 24, Pose::degToRad(-45)),
		Pose(-48, 36, Pose::degToRad(-45)),
		// Pose(-52, 60, Pose::degToRad(0)),
		Pose(-54, 100, Pose::degToRad(45)),
		Pose(6, 80, Pose::degToRad(135)),
		Pose(12, 36, Pose::degToRad(180))

	});

	// chassis.moveTo(Pose(12, 24, 0));
	purePursuitController.followPath(testTrajectory, &driver);
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


	Pose pose;

	// std::vector<Pose> waypoints = testTrajectory.getSplineWaypoints();

	while (true) {
		chassis.arcade(driver.get_analog(ANALOG_LEFT_Y), driver.get_analog(ANALOG_RIGHT_X));

		pose = chassis.getPose();

		if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			autonomous();
		}


		// for (Pose waypoint : waypoints) {
		// 	// driver.set_text(1, 0, std::to_string(Pose::radToDeg((-1 * waypoint.getTheta()) + (M_PI / 2.0))));
		// 	driver.set_text(1, 0, waypoint.to_string());
		// 	pros::delay(500);
		// }

		// driver.set_text(1,0, std::to_string((-1 * gyro.get_yaw() + 90)));
		driver.print(1, 0, "X: %.2f Y: %.2f H: %.2f", pose.getX(), pose.getY(), Pose::radToDeg(pose.getTheta()));
		pros::delay(20);
	}
}