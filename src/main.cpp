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
pros::Gps primary_gps(9, 0, 4, 180);
pros::Gps secondary_gps2(10, 0, 2, 0);

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

int motor_speed;

int proportional_tune = 1;
int derivative_tune = 1;

int turn_proportional_tune = 1;
int turn_derivative_tune = 1;

//PID for driving to X and Y coordinate
int PID (double dist, double curr, double p, double d){
  double prevError = 0.0;
  double Error = dist-curr;
  double dError = Error-prevError;

  double mtrspeed = p * Error + d * dError;

  double velmax = 85;

  if (mtrspeed > velmax){
    mtrspeed = velmax;
  } 
  
  if (mtrspeed < -velmax){
    mtrspeed = -velmax;
  }

  prevError = Error;

  return (int) mtrspeed; 
}

// PID for turning to a desired angle
int Turn_PID (double goal_angle, double curr_angle, double p, double d){
  double prevError = 0.0;
  double Error = goal_angle - curr_angle;
  double dError = Error-prevError;

  double mtrspeed = p * Error + d * dError;

  double velmax = 85;

  if (mtrspeed > velmax){
    mtrspeed = velmax;
  } 
  
  if (mtrspeed < -velmax){
    mtrspeed = -velmax;
  }

  prevError = Error;

  return (int) mtrspeed; 
}


// Movement function
void goTo(double ix, double iy, double iyaw)
{
	// Declare variables to be used in the loop
	pros::c::gps_status_s_t gpsData = primary_gps.get_status();
	double x, y, yaw, yawRadians;

	// Set upper and lower limits so the motors don't stall
	constexpr double lowerLimit = 0.10, upperLimit = 1.0;

	do
	{
		// Get GPS position
		gpsData = primary_gps.get_status();

		// Set initial values for x, y, and yaw for calculations below
		x = gpsData.x - ix;
		y = gpsData.y - iy;
		yaw = gpsData.yaw;

		// Rotate the vectors of the x and y error to match the rotation of the robot on the field
		yawRadians = yaw * M_PI / 180.0;
		x = x * std::cos(yawRadians) - y * std::sin(yawRadians);
		y = x * std::sin(yawRadians) + y * std::cos(yawRadians);

		// TO BE REPLACED WITH PID
		// Scale the values down so at 2m from the target, the drivebase moves at 100% power
		//x /= 2.0;
		//y /= 2.0;

		//PID initial concept
		double x_power = PID(x, 0, proportional_tune, derivative_tune);
		double y_power = PID(y, 0, proportional_tune, derivative_tune);
		double yaw_power = PID(iyaw, yaw, turn_proportional_tune, turn_derivative_tune);

		// Scale down the yaw value (and negate it) so at 180° from the target, it turns at 100% power
		yaw = -(yaw - iyaw) / 180.0;

		// Limit the values to an upper and lower limit so the motor always makes the wheels move
		// The drive function needs to be a value between 0 and 1
		x = std::copysign(std::clamp(std::abs(x_power/100), lowerLimit, upperLimit), x_power/100);
		y = std::copysign(std::clamp(std::abs(y_power/100), lowerLimit, upperLimit), y_power/100);
		yaw = std::copysign(std::clamp(std::abs(yaw_power/100), lowerLimit, upperLimit), yaw_power/100);

		// Make the chassis move based on error values
		xdrive->xArcade(x, y, yaw);

		// Printouts for debugging
		pros::screen::print(TEXT_MEDIUM, 2, "X Position: %3f", gpsData.x);
		pros::screen::print(TEXT_MEDIUM, 3, "Y Position: %3f", gpsData.y);
		pros::screen::print(TEXT_MEDIUM, 4, "Yaw: %3f", gpsData.yaw);

		pros::delay(20);

		// Check if position is within 2 cm and 1° of the target to exit the loop
	} while (std::abs(gpsData.x - ix) > .01 || std::abs(gpsData.y - iy) > .01 || std::abs(gpsData.yaw - iyaw) > 0.5);

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
	gps.get_status();
	pros::delay(500);
	goTo(inchesToMeters(36), inchesToMeters(36), 45);
	goTo(inchesToMeters(-36), inchesToMeters(36), -45);
	goTo(inchesToMeters(-24), inchesToMeters(0), -90);
	goTo(inchesToMeters(-36), inchesToMeters(-36), -135);
	goTo(inchesToMeters(36), inchesToMeters(-36), 135);
	goTo(inchesToMeters(0), inchesToMeters(0), 0);
}
