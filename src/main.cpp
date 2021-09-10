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
pros::Gps gps(9, 0, 5, 180);

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

void goTo(double ix, double iy, double iyaw)
{
	pros::c::gps_status_s_t gpsData = gps.get_status();
	double x, y, yaw, yawRadians;
	constexpr double lowerLimit = 0.10, upperLimit = 1.0;
	// const double initialDist = sqrt(pow(gpsData.x - ix, 2.0) + pow(gpsData.y - iy, 2.0));
	// pros::screen::print(TEXT_MEDIUM, 1, "%% Turn: %3f", initialDist);
	// double pctTurnPower;

	do
	{
		// pctTurnPower = 1.0 - sqrt(gpsData.x * gpsData.x + gpsData.y * gpsData.y) / initialDist;
		gpsData = gps.get_status();
		x = gpsData.x - ix;
		y = gpsData.y - iy;
		yaw = gpsData.yaw;
		yawRadians = yaw * M_PI / 180.0;
		x = x * std::cos(yawRadians) - y * std::sin(yawRadians);
		y = x * std::sin(yawRadians) + y * std::cos(yawRadians);
		pros::screen::print(TEXT_MEDIUM, 5, "X Shifted: %3f", x);
		pros::screen::print(TEXT_MEDIUM, 6, "Y Shifted: %3f", y);
		x /= 2.0;
		y /= 2.0;
		yaw = -(yaw - iyaw) / 180.0;
		// Limits the value to a lower and upper limit
		x = std::copysign(std::clamp(std::abs(x), lowerLimit, upperLimit), x);
		y = std::copysign(std::clamp(std::abs(y), lowerLimit, upperLimit), y);
		yaw = std::copysign(std::clamp(std::abs(yaw), lowerLimit, upperLimit), yaw);
		xdrive->xArcade(x, y, yaw);
		pros::screen::print(TEXT_MEDIUM, 2, "X Position: %3f", gpsData.x);
		pros::screen::print(TEXT_MEDIUM, 3, "Y Position: %3f", gpsData.y);
		pros::screen::print(TEXT_MEDIUM, 4, "Yaw: %3f", gpsData.yaw);
		pros::delay(20);
	} while (std::abs(gpsData.x - ix) > .01 || std::abs(gpsData.y - iy) > .01 || std::abs(gpsData.yaw - iyaw) > 0.5);
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
	gps.get_status();
	pros::delay(500);
	goTo(inchesToMeters(36), inchesToMeters(36), 45);
	goTo(inchesToMeters(-36), inchesToMeters(36), -45);
	goTo(inchesToMeters(-24), inchesToMeters(0), -90);
	goTo(inchesToMeters(-36), inchesToMeters(-36), -135);
	goTo(inchesToMeters(36), inchesToMeters(-36), 135);
	goTo(inchesToMeters(0), inchesToMeters(0), 0);
}
