#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;
using namespace okapi::literals;

pros::Controller master(pros::E_CONTROLLER_MASTER);
 Motor FrontLeft(-20);
 Motor BackLeft(19);
 Motor FrontRight(9);
 Motor BackRight(-10);
 Motor armMotor(-8);

okapi::MotorGroup leftDrive({20, 19});
okapi::MotorGroup rightDrive({9, 10});

Controller controller;

ControllerButton armUpButton(ControllerDigital::R2);
ControllerButton armDownButton(ControllerDigital::R1);

auto drive = ChassisControllerBuilder()
    .withMotors(leftDrive, rightDrive)
    // Green gear set, 4 in wheel diam, 11.5 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 9_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void tankControl(){
  // Tank drive with left and right sticks
  drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
                          controller.getAnalog(ControllerAnalog::rightY));
}
////////////////////////////////////////////////////////////////////////////
//Makes the lift go up
////////////////////////////////////////////////////////////////////////////
void liftArm(){
  armMotor.moveVoltage(12000);
}
////////////////////////////////////////////////////////////////////////////
//Makes the lift go down
////////////////////////////////////////////////////////////////////////////
void lowerArm(){
   armMotor.moveVoltage(-12000);
}
////////////////////////////////////////////////////////////////////////////
//Stops the lift and makes it hold its position
////////////////////////////////////////////////////////////////////////////
void stopArm(){
  armMotor.moveVoltage(0);
  armMotor.setBrakeMode(AbstractMotor::brakeMode::hold);
}
////////////////////////////////////////////////////////////////////////////
//Maps controller buttons to the lift movements
////////////////////////////////////////////////////////////////////////////
void armControl(){
  // else, the arm isn't all the way down
  if (armUpButton.isPressed()) {
      liftArm();
  } else if (armDownButton.isPressed()) {
      lowerArm();
  } else {
     stopArm();
  }
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// Chassis Controller - lets us drive the robot around with open- or closed-loop control

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

void autonomous() {

  // set the state to zero
  drive->setState({0_in, 0_in, 0_deg});
  // turn 45 degrees and drive approximately 1.4 ft
  drive->driveToPoint({1_ft, 1_ft});
  // turn approximately 45 degrees to end up at 90 degrees
  drive->turnToAngle(90_deg);
  // turn approximately -90 degrees to face {5_ft, 0_ft} which is to the north of the robot
  drive->turnToPoint({5_ft, 0_ft});
  /*
	drive->setMaxVelocity(30);
	drive->moveDistance(12_in); // Drive forward 12 inches
	drive->turnAngle(90_deg);   // Turn in place 90 degrees*/
}

void opcontrol() {
	 while (true) {

     // Chassis Controller - lets us drive the robot around with open- or closed-loop control

        tankControl();
        armControl();
			  pros::delay(10);
	}
}
