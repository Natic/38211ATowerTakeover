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
 Motor intakeL(11);
 Motor intakeR(-1);
 pros::Motor Tilter(18, pros::E_MOTOR_GEARSET_36);


okapi::MotorGroup leftDrive({-20, 19});
okapi::MotorGroup rightDrive({9, -10});
okapi::MotorGroup Intakes({11,-1});

Controller controller;

ControllerButton armUpButton(ControllerDigital::R2);
ControllerButton armDownButton(ControllerDigital::R1);
ControllerButton intakeButton(ControllerDigital::L1);
ControllerButton outtakeButton(ControllerDigital::L2);
ControllerButton swapButton(ControllerDigital::A);
ControllerButton tilterUpButton(ControllerDigital::X);
ControllerButton tilterDownButton(ControllerDigital::B);
ControllerButton tilterUpButtonSlow(ControllerDigital::up);
ControllerButton tilterDownButtonSlow(ControllerDigital::down);
ControllerButton tilterSetupButton(ControllerDigital::Y);
ControllerButton deployButton(ControllerDigital::left);

float driveMode=0;

float deployMode=0;

auto drive = ChassisControllerBuilder()
    .withMotors(leftDrive, rightDrive)
    // Green gear set, 4 in wheel diam, 11.5 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 9_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

////////////////////////////////////////////////////////////////////////////
/*

  Start of functions

*/
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//Swaps between tank and arcade drive schemes
////////////////////////////////////////////////////////////////////////////
void driveSwap(){
  if(swapButton.isPressed() && driveMode==0){
    driveMode=1;
  }else
  driveMode=0;
  }

////////////////////////////////////////////////////////////////////////////
//Controls thee drivee schemes
////////////////////////////////////////////////////////////////////////////
void driveControl(){
  if(driveMode==0){
  // Tank drive with left and right sticks
  drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
                          controller.getAnalog(ControllerAnalog::rightY));
  }else if (driveMode==1){
    // Arcade drive with the left stick.
  drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
                            controller.getAnalog(ControllerAnalog::leftX));
  }
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
  if (armUpButton.isPressed()) {
      liftArm();
  } else if (armDownButton.isPressed()) {
      lowerArm();
  } else {
     stopArm();
  }
}
////////////////////////////////////////////////////////////////////////////
//Spin intakes forward
////////////////////////////////////////////////////////////////////////////
void Intake(){
  Intakes.moveVoltage(12000);
}
////////////////////////////////////////////////////////////////////////////
//Spin intakes outward
////////////////////////////////////////////////////////////////////////////
void Outtake(){
  Intakes.moveVoltage(-6000);
}
////////////////////////////////////////////////////////////////////////////
//Stop intakes
////////////////////////////////////////////////////////////////////////////
void stopIntake(){
  Intakes.moveVoltage(0);
  Intakes.setBrakeMode(AbstractMotor::brakeMode::coast);
}
////////////////////////////////////////////////////////////////////////////
//Maps controller buttons to intakes
////////////////////////////////////////////////////////////////////////////
void intakeControl(){
  if (intakeButton.isPressed()) {
      Intake();
  } else if (outtakeButton.isPressed()) {
      Outtake();
  } else {
     stopIntake();
  }
}
////////////////////////////////////////////////////////////////////////////
//Moves the tilter forward
////////////////////////////////////////////////////////////////////////////
void tiltForward(){
  Tilter.move_voltage(12000);
}
////////////////////////////////////////////////////////////////////////////
//Moves the tilter backwards
////////////////////////////////////////////////////////////////////////////
void tiltBack(){
  Tilter.move_voltage(-12000);
}
////////////////////////////////////////////////////////////////////////////
//Stops the tilter
////////////////////////////////////////////////////////////////////////////
void stopTilter(){
  Tilter.move_voltage(0);
  Tilter.set_brake_mode(MOTOR_BRAKE_HOLD);
}
////////////////////////////////////////////////////////////////////////////
//Maps the tilter movements to the controller
////////////////////////////////////////////////////////////////////////////
void tilterControl(){
  if (tilterUpButton.isPressed()) {
      tiltForward();
  } else if (tilterDownButton.isPressed()) {
      tiltBack();
  } else {
     stopTilter();
  }
}
////////////////////////////////////////////////////////////////////////////
//Moves tilter to set location
////////////////////////////////////////////////////////////////////////////
void tiltMacro(){
  if(tilterSetupButton.isPressed()){
    Tilter.move_relative(30, 100);
  }
}
////////////////////////////////////////////////////////////////////////////
//Deploys the tray
////////////////////////////////////////////////////////////////////////////
void deploy(){
  Intakes.moveVoltage(-12000);
  liftArm();
  tiltForward();
  pros::delay(1000);
  tiltBack();
  lowerArm();
  pros::delay(1000);
  stopTilter();
  stopIntake();
  stopArm();
}
////////////////////////////////////////////////////////////////////////////
//Button to deploy the tray
////////////////////////////////////////////////////////////////////////////
void deployMacro(){
  if (deployButton.isPressed()){
    deploy();
}
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
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

  deploy();

  drive->setMaxVelocity(50);
  // set the state to zero
  drive->setState({0_in, 0_in, 0_deg});
  // turn 45 degrees and drive approximately 1.4 ft
  drive->driveToPoint({5.5_ft, 0_ft});

  pros::delay(300);
  // turn approximately 45 degrees to end up at 90 degrees
  drive->setMaxVelocity(30);

  drive->turnToAngle(45_deg);

  pros::delay(200);

  drive->setState({0_in, 0_in, 0_deg});

  drive->driveToPoint({3_ft, 0_ft});

  tiltForward();

  pros::delay(2000);
  //drive->turnToAngle(-180_deg);

  //  drive->driveToPoint({2_ft, 3_ft});
  // turn approximately -90 degrees to face {5_ft, 0_ft} which is to the north of the robot
  //drive->turnToPoint({5_ft, 0_ft});
  /*
	drive->setMaxVelocity(30);
	drive->moveDistance(12_in); // Drive forward 12 inches
	drive->turnAngle(90_deg);   // Turn in place 90 degrees*/
}

void opcontrol() {
	 while (true) {
        deployMacro();
        driveSwap();
        driveControl();
        armControl();
        intakeControl();
        tilterControl();
        tiltMacro();
        //tilterControlSlow();
			  pros::delay(10);
	}
}
