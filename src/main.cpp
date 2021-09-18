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
pros::Gps gpsPrimary(9, 0, 4, 180);
pros::Gps gpsSecondary(10, 0, 2, 0);

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
void goTo(double ix, double iy, double iyaw, double resolution)
{
	// PID (or just PD in this case) values to be tuned
	constexpr double driveP = 1.0, driveD = 1.0, turnP = 1.0 / 180.0, turnD = 1.0;

	// Declare variables to be used in the loop
	pros::c::gps_status_s_t gpsData = gpsPrimary.get_status();
	double xErr = gpsData.x - ix, yErr = gpsData.y - iy, yawErr = iyaw - gpsData.yaw, xPrevErr, yPrevErr, yawPrevErr, xPow, yPow, yawPow, yawRadians;

	// Set upper and lower limits so the motors don't stall
	constexpr double lowerLimit = 0.10, upperLimit = 1.0;

	do
	{
		// Get GPS position
		gpsData = gpsPrimary.get_status();

		// Save previous error values for calculation below
		xPrevErr = xErr;
		yPrevErr = yErr;
		yawPrevErr = yawErr;

		// Set initial values for x, y, and yaw for calculations below
		xErr = gpsData.x - ix;
		yErr = gpsData.y - iy;
		yawErr = iyaw - gpsData.yaw;

		// Scale the errors to a power percentage based on PID values
		xPow = xErr * driveP + (xErr - xPrevErr) * driveD;
		yPow = yErr * driveP + (yErr - yPrevErr) * driveD;
		yawPow = yawErr * turnP + (yawErr - yawPrevErr) * turnD;

		// Rotate the vectors of the x and y error to match the rotation of the robot on the field
		yawRadians = gpsData.yaw * M_PI / 180.0;
		xPow = xPow * std::cos(yawRadians) - yPow * std::sin(yawRadians);
		yPow = xPow * std::sin(yawRadians) + yPow * std::cos(yawRadians);

		// Limit the values to an upper and lower limit so the motor always makes the wheels move
		// The drive function needs to be a value between 0 and 1 as it is a percentage
		xPow = std::copysign(std::clamp(std::abs(xPow), lowerLimit, upperLimit), xPow);
		yPow = std::copysign(std::clamp(std::abs(yPow), lowerLimit, upperLimit), yPow);
		yawPow = std::copysign(std::clamp(std::abs(yawPow), lowerLimit, upperLimit), yawPow);

		// Make the chassis move based on error values
		xdrive->xArcade(xPow, yPow, yawPow);

		// Printouts for debugging
		pros::screen::print(TEXT_MEDIUM, 2, "X Position: %3f", gpsData.x);
		pros::screen::print(TEXT_MEDIUM, 3, "Y Position: %3f", gpsData.y);
		pros::screen::print(TEXT_MEDIUM, 4, "Yaw: %3f", gpsData.yaw);

		pros::delay(20);

		// Check if position is within <resolution> cm and <resolution>° of the target to exit the loop
	} while (std::abs(xErr) > resolution * .005 || std::abs(yErr) > resolution * .005 || std::abs(yawErr) > resolution * .5);

	// Stop chassis motion
	xdrive->stop();
}

double inchesToMeters(double inches)
{
	return 0.0254 * inches;
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
	goTo(inchesToMeters(36), inchesToMeters(36), 45);
	goTo(inchesToMeters(-36), inchesToMeters(36), -45);
	goTo(inchesToMeters(-24), inchesToMeters(0), -90);
	goTo(inchesToMeters(-36), inchesToMeters(-36), -135);
	goTo(inchesToMeters(36), inchesToMeters(-36), 135);
	goTo(inchesToMeters(0), inchesToMeters(0), 0);
}
