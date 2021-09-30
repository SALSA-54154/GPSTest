#include "main.h"

pros::Controller cont(pros::E_CONTROLLER_MASTER);

auto chassis = okapi::ChassisControllerBuilder()
				   .withMotors(
					   1,  // Top left
					   -2, // Top right (reversed)
					   -4, // Bottom right (reversed)
					   3   // Bottom left
					   )
				   .withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 20_in}, okapi::imev5GreenTPR})
				   .build();
auto xdrive = std::dynamic_pointer_cast<okapi::XDriveModel>(chassis->getModel());
pros::Gps gpsPrimary(9, 0.0, 0.0254 * 4.0);
// pros::Gps gpsSecondary(10, 0.0, 0.0, 180.0, 0.0, 0.0254 * 2.0);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

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

// Movement function
void goTo(okapi::Point ipoint, okapi::QAngle iangle, okapi::IterativePosPIDController::Gains idriveGains, okapi::IterativePosPIDController::Gains iturnGains)
{
	// Create PID controllers using OkapiLib to be used in the do-while loop
	auto xPID = okapi::IterativePosPIDController(idriveGains, okapi::TimeUtilFactory().withSettledUtilParams(0.08));
	auto yPID = okapi::IterativePosPIDController(idriveGains, okapi::TimeUtilFactory().withSettledUtilParams(0.08));
	auto yawPID = okapi::IterativePosPIDController(iturnGains, okapi::TimeUtilFactory().withSettledUtilParams(8.0));

	// Set the target for the PID controllers to the input parameters
	xPID.setTarget(ipoint.x.convert(okapi::meter));
	yPID.setTarget(ipoint.y.convert(okapi::meter));
	yawPID.setTarget(iangle.convert(okapi::degree));

	// Set upper and lower limits so the motors don't stall
	constexpr double lowerLimit = 0.10, upperLimit = 1.0;
	xPID.setControllerSetTargetLimits(upperLimit, lowerLimit);
	yPID.setControllerSetTargetLimits(upperLimit, lowerLimit);
	yawPID.setControllerSetTargetLimits(upperLimit, lowerLimit);

	// Declare variables to be used in the loop
	pros::c::gps_status_s_t gpsData;
	double xPow, yPow, yawPow, yawRadians;

	do
	{
		// Get GPS position
		// Check if primary GPS can see
		// if (gpsPrimary.get_error() < .01)
		// Use primary GPS data
		gpsData = gpsPrimary.get_status();
		// else
		// {
		// 	// Use secondary GPS data
		// 	gpsData = gpsSecondary.get_status();

		// 	// Flip sensor yaw 180
		// 	gpsData.yaw += 180.0;
		// 	if (gpsData.yaw > 180.0)
		// 		gpsData.yaw = 180.0 - gpsData.yaw;
		// }

		xPow = xPID.step(gpsData.x);
		yPow = yPID.step(gpsData.y);
		yawPow = yawPID.step(gpsData.yaw);

		yawRadians = okapi::degreeToRadian * gpsData.yaw;
		xPow = xPow * std::cos(yawRadians) - yPow * std::sin(yawRadians);
		yPow = xPow * std::sin(yawRadians) + yPow * std::cos(yawRadians);

		// Rotate the vectors of the x and y error to match the rotation of the robot on the field and
		// Make the chassis move based on error values
		xdrive->xArcade(xPow, yPow, yawPow);

		// Printouts for debugging
		pros::screen::print(TEXT_MEDIUM, 1, "X Position: %3f", gpsData.x);
		pros::screen::print(TEXT_MEDIUM, 2, "Y Position: %3f", gpsData.y);
		pros::screen::print(TEXT_MEDIUM, 3, "Yaw: %3f", gpsData.yaw);
		pros::screen::print(TEXT_MEDIUM, 4, "xSettled: %d", xPID.isSettled());
		pros::screen::print(TEXT_MEDIUM, 5, "ySettled: %d", yPID.isSettled());
		pros::screen::print(TEXT_MEDIUM, 6, "yawSettled: %d", yawPID.isSettled());

		pros::delay(20);

		// Check if chassis is settled to exit the loop
	} while (!xPID.isSettled() || !yPID.isSettled() || !yawPID.isSettled());

	// Stop chassis motion
	xdrive->stop();
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
void opcontrol()
{
	gpsPrimary.get_status();
	pros::delay(500);
	const okapi::IterativePosPIDController::Gains driveGains = {-2.0, 0.0, -0.01}, turnGains = {1.0 / 90.0, 0.0, 0.001};

	// goTo({36_in, 36_in}, 45_deg, driveGains, turnGains);
	// goTo({-36_in, 36_in}, -45_deg, driveGains, turnGains);
	// goTo({-24_in, 0_in}, -90_deg, driveGains, turnGains);
	// goTo({-36_in, -36_in}, -135_deg, driveGains, turnGains);
	// goTo({36_in, -36_in}, 135_deg, driveGains, turnGains);
	goTo({0_in, 0_in}, 0_deg, driveGains, turnGains);
}
