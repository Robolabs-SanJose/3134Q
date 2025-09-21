#include "main.h"
#include "pros/motors.hpp"
#include <cmath>
// L1 will intake the balls
// L2 will score onto the lower middle goal
// R1 will score into the higher middle goal
// R2 will score onto the higher side goals
// X will expand the piston to take the balls out of the tube
// Y will retract the piston
#define LEFT_MOTOR_A_PORT 1
#define LEFT_MOTOR_B_PORT 2
#define LEFT_MOTOR_C_PORT 3

#define RIGHT_MOTOR_A_PORT 6
#define RIGHT_MOTOR_B_PORT 9
#define RIGHT_MOTOR_C_PORT 4

#define INERTIAL_PORT 21

#define LOWWERROLLER_PORT 7
#define UPPERROLLER_PORT 10
#define MIDDLEROLLER_PORT 5
//#define BASKET_PORT 17

#define BOB_PORT 'a'
#define JEFF_PORT 'b'

// #define ROTATION_PORT 5
// #define ELEVATOR_PORT 19

// pros::Rotation RotationSensor(ROTATION_PORT);

// pros::Motor Elevator(ELEVATOR_PORT);
// pros::Motor HighStakes(port, pros::motor_gearset_e:: E_MOTOR_GEAR_RED);

pros::MotorGroup LeftDriveSmart({LEFT_MOTOR_A_PORT, LEFT_MOTOR_B_PORT, LEFT_MOTOR_C_PORT});		// Creates a motor group with forwards ports 1 & 4 and reversed port 7
pros::MotorGroup RightDriveSmart({RIGHT_MOTOR_A_PORT, RIGHT_MOTOR_B_PORT, RIGHT_MOTOR_C_PORT}); // Creates a motor group with forwards port 2 and reversed port 4 and 7
pros::Imu Inertial(INERTIAL_PORT);
pros::MotorGroup smartdrive({LEFT_MOTOR_A_PORT, LEFT_MOTOR_B_PORT, -LEFT_MOTOR_C_PORT, RIGHT_MOTOR_A_PORT, RIGHT_MOTOR_B_PORT, -RIGHT_MOTOR_C_PORT});
pros::MotorGroup Intake({UPPERROLLER_PORT,MIDDLEROLLER_PORT,-LOWWERROLLER_PORT});
pros::MotorGroup Outtake({-UPPERROLLER_PORT,-MIDDLEROLLER_PORT,LOWWERROLLER_PORT});
pros::MotorGroup ScoreMiddle({UPPERROLLER_PORT,-MIDDLEROLLER_PORT,-LOWWERROLLER_PORT});

pros::ADIDigitalOut Bob({BOB_PORT});
pros::ADIDigitalOut Jeff({JEFF_PORT});

bool bobState = false;
bool jeffState = false;

void ToggleBob() // scorring pistens
{
	bobState = !bobState;	 // Toggle the state
	Bob.set_value(bobState); // Update the digital output
	pros::delay(200);					 // Delay for debouncing
}
void ToggleJeff()
{
	jeffState = !jeffState;
	Jeff.set_value(jeffState);
	pros::delay(200);
}
/**
 * A callback function for LLEMU's centerk button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
	Inertial.reset();
	// RotationSensor.reset();
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

enum Direction
{
	clockwise,
	counterclockwise
};

enum forback
{
	forward,
	backward
};

void TurnDegrees(pros::IMU &inertial, Direction dir, int degrees)
{
	Inertial.reset();
	pros::delay(1000);
	int initial = Inertial.get_heading();
	int targetdeg;

	if (dir == clockwise)
	{
		targetdeg = (initial + degrees) % 360;
		LeftDriveSmart.move_velocity(20);
		RightDriveSmart.move_velocity(20);

		while (inertial.get_heading() < targetdeg)
		{
			pros::delay(5);
		}
	}
	else if (dir == counterclockwise)
	{
		targetdeg = 360 - degrees;

		RightDriveSmart.move_velocity(-20);
		LeftDriveSmart.move_velocity(-20);

		while (inertial.get_heading() > targetdeg || inertial.get_heading() < 5)
		{
			pros::delay(5);
		}
	}

	// Stop the motors
	LeftDriveSmart.move_velocity(0);
	RightDriveSmart.move_velocity(0);
}

void inertialMove(int speed, int duration, forback FB)
{
	// DURATION IN MILLISECONDS

	int initial = Inertial.get_heading(); // Get initial heading

	int leftSpeed;
	int rightSpeed;

	if (FB == backward)
	{
		rightSpeed = -speed;
		leftSpeed = speed;
	}
	else
	{
		leftSpeed = -speed;
		rightSpeed = speed;
	}
	double kp = .1;
	int endTime = pros::millis() + duration;

	while (pros::millis() < endTime)
	{
		int currentHeading = Inertial.get_heading();
		int error = currentHeading - initial;
		int correction = error * kp;

		// Apply correction temporarily without modifying base speeds
		LeftDriveSmart.move_velocity(leftSpeed + correction);
		RightDriveSmart.move_velocity(rightSpeed - correction);

		pros::delay(5); // Small delay for loop efficiency
	}

	// Stop the motors after duration ends
	LeftDriveSmart.move_velocity(0);
	RightDriveSmart.move_velocity(0);
}

void moveForward(int speed)
{
	RightDriveSmart.move_velocity(-speed);
	LeftDriveSmart.move_velocity(speed);
}
void driveStop()
{
	RightDriveSmart.move_velocity(0);
	LeftDriveSmart.move_velocity(0);
}
void turn(int speed, int dir)
{
	if (dir == 1)
	{
		RightDriveSmart.move_velocity(-speed);
		LeftDriveSmart.move_velocity(-speed);
		// turn left
	}
	if (dir == 0)
	{
		RightDriveSmart.move_velocity(speed);
		LeftDriveSmart.move_velocity(speed);
		// turn right
	}
}

void autonomous()
{
	
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
	pros::Controller Controller1(pros::E_CONTROLLER_MASTER);

	while (true)
	{
		// Calculate drivetrain motor velocities
		// Left joystick (up/down) for forward/backward (Axis3)
		// Right joystick (left/right) for turning (Axis1)
		int turn = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);	 // Forward/backward
		int forward = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); // Turning

		// Compute motor speeds for tank drive
		int drivetrainLeftSideSpeed = (forward - turn);	  // Left motor speed
		int drivetrainRightSideSpeed = -(forward + turn); // Right motor speed

		// Deadband logic to prevent small joystick movements from moving the robot
		const int deadband = 25; // Threshold for joystick input
		if (abs(drivetrainLeftSideSpeed) < deadband)
		{
			drivetrainLeftSideSpeed = 0;
		}
		if (abs(drivetrainRightSideSpeed) < deadband)
		{
			drivetrainRightSideSpeed = 0;
		}

		// Set motor velocities
		LeftDriveSmart.move_velocity((drivetrainRightSideSpeed * 2)); // Adjust scaling as needed
		RightDriveSmart.move_velocity(-(drivetrainLeftSideSpeed * 2));

		// Control Clamp and Flag using buttons
		if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			ToggleBob();
		}
		if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
		{
			ToggleJeff();
		}
		if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
		}
		if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
		}

		// Control Intake using shoulder buttons (L1/L2)
		if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			ScoreMiddle.move_velocity(200);
		}
		else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
		}
		else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			Intake.move_velocity(200);
		}
		else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			Outtake.move_velocity(200);
		}
		else
		{
			Intake.move_velocity(0);
			ScoreMiddle.move_velocity(0);
			Outtake.move_velocity(0);
		}

		// Delay to prevent CPU overload
		pros::delay(20);
	}
}
