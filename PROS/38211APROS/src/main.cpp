#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;
using namespace okapi::literals;

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor FrontLeft(20, true);
pros::Motor BackLeft(19);
pros::Motor FrontRight(9);
pros::Motor BackRight(10, true);

okapi::MotorGroup leftDrive({20, 19});
okapi::MotorGroup rightDrive({9, 10});


// Chassis Controller - lets us drive the robot around with open- or closed-loop control
 auto drive = ChassisControllerBuilder()
		 .withMotors(leftDrive, rightDrive)
		 // Green gear set, 4 in wheel diam, 11.5 in wheel track
		 .withDimensions(AbstractMotor::gearset::green, {{4_in, 9_in}, imev5GreenTPR})
		 .build();
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
	drive->setMaxVelocity(30);
	drive->moveDistance(12_in); // Drive forward 12 inches
	drive->turnAngle(90_deg);   // Turn in place 90 degrees
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor FrontLeft(20, true);
	pros::Motor BackLeft(19);
	pros::Motor FrontRight(9);
	pros::Motor BackRight(10, true);

	okapi::MotorGroup leftDrive({20, 19});
	okapi::MotorGroup rightDrive({9, 10});


	// Chassis Controller - lets us drive the robot around with open- or closed-loop control
	 auto drive = ChassisControllerBuilder()
			 .withMotors(leftDrive, rightDrive)
			 // Green gear set, 4 in wheel diam, 11.5 in wheel track
			 .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
			 .build();

	 // Joystick to read analog values for tank or arcade control
	 // Master controller by default
	 Controller controller;

	 // Arm related objects
	 ControllerButton armUpButton(ControllerDigital::R2);
	 ControllerButton armDownButton(ControllerDigital::R1);
	 Motor armMotor(-8);

	 // Button to run our sample autonomous routine
	 ControllerButton runAutoButton(ControllerDigital::X);

	 while (true) {
			 // Tank drive with left and right sticks
			 drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
															 controller.getAnalog(ControllerAnalog::rightY));

					 // else, the arm isn't all the way down
					 if (armUpButton.isPressed()) {
							 armMotor.moveVoltage(12000);
					 } else if (armDownButton.isPressed()) {
							 armMotor.moveVoltage(-12000);
					 } else {
							 armMotor.moveVoltage(0);
					 }
			 }

			 // Run the test autonomous routine if we press the button
			 if (runAutoButton.changedToPressed()) {
					 // Drive the robot in a square pattern using closed-loop control
					 for (int i = 0; i < 4; i++) {
							 drive->moveDistance(12_in); // Drive forward 12 inches
							 drive->turnAngle(90_deg);   // Turn in place 90 degrees
					 }
			 }

			 // Wait and give up the time we don't need to other tasks.
			 // Additionally, joystick values, motor telemetry, etc. all updates every 10 ms.
			 pros::delay(10);


	}
