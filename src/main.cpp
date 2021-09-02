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
	pros::c::gps_status_s_t gpsData = gps.get_status();
	double x, y, yaw;
	constexpr double lowerLimit = .06;
	pros::delay(500);
	do
	{
		gpsData = gps.get_status();
		x = gpsData.x / 2;
		x = std::copysign(std::clamp(std::abs(x), lowerLimit, 1.0), x);
		y = gpsData.y / 2;
		y = std::copysign(std::clamp(std::abs(y), lowerLimit, 1.0), y);
		yaw = -gpsData.yaw / 180;
		yaw = std::copysign(std::clamp(std::abs(yaw), lowerLimit, 1.0), yaw);
		xdrive->xArcade(x, y, yaw);
		pros::screen::print(TEXT_MEDIUM, 2, "X Position: %3f", gpsData.x);
		pros::screen::print(TEXT_MEDIUM, 2, "Y Position: %3f", gpsData.y);
		pros::screen::print(TEXT_MEDIUM, 5, "Yaw: %3f", gpsData.yaw);
		pros::delay(20);
	} while (abs(gpsData.x) > .02 || abs(gpsData.y) > .02 || abs(gpsData.yaw) > 1.0);
	xdrive->stop();
}
