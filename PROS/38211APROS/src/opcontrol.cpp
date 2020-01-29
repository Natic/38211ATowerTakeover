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
     .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
     .build();

void opcontrol() {




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
