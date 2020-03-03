#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;
using namespace okapi::literals;
#define IMU_PORT 12

pros::Controller master(pros::E_CONTROLLER_MASTER);
 Motor FrontLeft(-20);
 Motor BackLeft(19);
 Motor FrontRight(9);
 Motor BackRight(-10);
 Motor armMotor(-8);
 Motor intakeL(11);
 Motor intakeR(-1);
 pros::Motor Tilter(17, pros::E_MOTOR_GEARSET_36);

pros::Imu imu_sensor(IMU_PORT);

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

void slowIntake(){
  Intakes.moveVoltage(8500);
}
////////////////////////////////////////////////////////////////////////////
//Spin intakes outward
////////////////////////////////////////////////////////////////////////////
void Outtake(){
  Intakes.moveVoltage(-8500);
}

void slowOuttake(){
  Intakes.moveVoltage(-5500);
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
//Deploys the tray and sets it farther forward for back autos
////////////////////////////////////////////////////////////////////////////
void deployBack(){
  Intakes.moveVoltage(-12000);
  liftArm();
  tiltForward();
  pros::delay(1000);
  tiltBack();
  lowerArm();
  pros::delay(500);
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
//A basic turning function for the IMU sensor
////////////////////////////////////////////////////////////////////////////
void turnIMU(int left, int right, int value){
    while(fabs(imu_sensor.get_rotation() < value)){
        leftDrive.moveVoltage(left);
        rightDrive.moveVoltage(right);
        pros::delay(1);
    }
    leftDrive.moveVoltage(0);
    rightDrive.moveVoltage(0);
    leftDrive.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightDrive.setBrakeMode(AbstractMotor::brakeMode::hold);
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
/*

  Start of autonomous functions

*/
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//A test function for the IMU sensor
////////////////////////////////////////////////////////////////////////////
void testIMU(){
while(fabs(imu_sensor.is_calibrating()== true)){
  pros::delay(10);
}
  turnIMU(6000, -6000, -50);
}
////////////////////////////////////////////////////////////////////////////
//A test function for the odometry tracking
////////////////////////////////////////////////////////////////////////////
void testOdom(){
  drive->setState({0_in, 0_in, 0_deg});

  drive->driveToPoint({1_ft, 0_ft});

  drive->turnToAngle(-45_deg);

  drive->driveToPoint({3_ft, 0_ft});
}
////////////////////////////////////////////////////////////////////////////
//A five point auto for the blue small goal
////////////////////////////////////////////////////////////////////////////
void blueFront(){
    //deploys the tray
    deploy();
    pros::delay(500);
    //starts the intakes
    Intake();
    //moves forward to intake 5 cubes
    drive->setMaxVelocity(60);
    drive->moveDistance(30_in);

    pros::delay(300);
    //drives back to align turn
    drive->moveDistance(-15_in);
    //turns to face the small goal
    drive->setMaxVelocity(30);
    drive->turnAngle(-130_deg);

    pros::delay(200);
    //aligns for the goal using time as to not have the encoders get stuck
    leftDrive.moveVoltage(6000);
    rightDrive.moveVoltage(6000);
    pros::delay(700);
    leftDrive.moveVoltage(0);
    rightDrive.moveVoltage(0);
    //moves the intakes to properly index the cubes
    stopIntake();
    Outtake();
    pros::delay(300);
    stopIntake();
    //stacks the cubes
    tiltForward();
    pros::delay(3000);
    stopTilter();
    Outtake();
    //back away from the stack
    drive->moveDistance(-10_in);
}
////////////////////////////////////////////////////////////////////////////
//A 5 point auto for the red small goal
////////////////////////////////////////////////////////////////////////////
void redFront(){
  //deploys the tray
  deploy();
  //starts the intakes
  Intake();
  //moves forward to intake 5 cubes
  drive->setMaxVelocity(60);
  drive->moveDistance(30_in);

  pros::delay(300);
  //drives back to align turn
  drive->moveDistance(-15_in);
  //turns to face the small goal
  drive->setMaxVelocity(30);
  drive->turnAngle(132_deg);

  pros::delay(200);
  //aligns for the goal using time as to not have the encoders get stuck
  leftDrive.moveVoltage(6000);
  rightDrive.moveVoltage(6000);
  pros::delay(700);
  leftDrive.moveVoltage(0);
  rightDrive.moveVoltage(0);
  //moves the intakes to properly index the cubes
  stopIntake();
  Outtake();
  pros::delay(300);
  stopIntake();
  //stacks the cubes
  tiltForward();
  pros::delay(3000);
  stopTilter();
  Outtake();
  //back away from the stack
  drive->moveDistance(-10_in);
}
////////////////////////////////////////////////////////////////////////////
//A 2 point auto for the blue small goal
////////////////////////////////////////////////////////////////////////////
void blueBack(){
  //deploy the tray
  deploy();
  //start the intakes
  Intake();
  //move forward to grab 2 cubes
  drive->setMaxVelocity(40);
  drive->moveDistance(15_in);

  pros::delay(300);
  //turn to grab 3rd cube
  drive->setMaxVelocity(30);
  drive->turnAngle(90_deg);

  pros::delay(300);
  //drive forward to grab 3rd cube
  drive->moveDistance(15_in);

  pros::delay(200);
  //drive back to align with goal
  drive->moveDistance(-3_in);

  pros::delay(100);
  //turn to face goal
  drive->turnAngle(30_deg);

  pros::delay(200);
  //drive into goal using time as to not mess up encoders
  leftDrive.moveVoltage(4000);
  rightDrive.moveVoltage(4000);
  pros::delay(600);
  leftDrive.moveVoltage(0);
  rightDrive.moveVoltage(0);
  //outtake 2 cubes
  Outtake();

}
////////////////////////////////////////////////////////////////////////////
//A 36 point auto for the 1 minute skills challenge
////////////////////////////////////////////////////////////////////////////
void Skills(){
    //deploy the tray
    deploy();

    leftDrive.moveVoltage(-3000);
    rightDrive.moveVoltage(-3000);
    pros::delay(800);
    leftDrive.moveVoltage(0);
    rightDrive.moveVoltage(0);

    pros::delay(1000);
    //start intakes
    Intake();
    //drive forward and intake 5 cubes
    drive->setMaxVelocity(40);
    drive->moveDistance(30_in);

    pros::delay(300);
    //turn to align with second line of cubes
    drive->setMaxVelocity(20);
    drive->turnAngle(-37_deg);
    //drive to align with second line of cubes
    drive->setMaxVelocity(40);
    drive->moveDistance(-28_in);
    //turn to aligm woth second line of cubes pt.2 electric boogaloo
    drive->setMaxVelocity(20);
    drive->turnAngle(41_deg);
    //intake 2 cubes from the 2nd line
    drive->setMaxVelocity(40);
    drive->moveDistance(10_in);
    //move tilter forward so the lift can move
    tiltForward();
    pros::delay(500);
    stopTilter();
    //move forward  cube
    drive->moveDistance(5_in);

    pros::delay(200);
    //stop intakes and set brake mode to hold
    stopIntake();
    Intakes.setBrakeMode(AbstractMotor::brakeMode::hold);
    //move tilter forward as to balance COG
    tiltForward();
    pros::delay(800);
    stopTilter();
    //hold intakes
    Intakes.setBrakeMode(AbstractMotor::brakeMode::hold);
    //turn to align with the small goal
    drive->setMaxVelocity(20);
    drive->turnAngle(-123_deg);
    //move forward to align with the small goal
    drive->setMaxVelocity(30);
    drive->moveDistance(27_in);

    leftDrive.moveVoltage(3000);
    rightDrive.moveVoltage(3000);
    pros::delay(1000);
    leftDrive.moveVoltage(0);
    rightDrive.moveVoltage(0);
    //outtake to properly index the cubes
    stopIntake();
    //slowOuttake();
    pros::delay(250);
    stopIntake();
    //deploy the stack of cubes
    tiltForward();
    pros::delay(2850);
    stopTilter();
    pros::delay(400);
    //outtake to index the bottom cube
    Outtake();
    pros::delay(100);
    //drive back from the stack of cubes
    drive->moveDistance(-9_in);
    //move the tilter back so the lift can move up
    tiltBack();
    pros::delay(2300);
    stopTilter();
    stopIntake();
    //turn to face the medium tower
    drive->setMaxVelocity(40);
    drive->turnAngle(-137_deg);
    //drive to the medium tower
    slowIntake();
    drive->moveDistance(17_in);
    stopIntake();
    //intake the cube by the medium tower


    drive->moveDistance(-5_in);

    liftArm();
    pros::delay(1250);
    stopArm();

    leftDrive.moveVoltage(3000);
    rightDrive.moveVoltage(3000);
    pros::delay(2000);
    leftDrive.moveVoltage(0);
    rightDrive.moveVoltage(0);


    slowOuttake();
    pros::delay(2000);
    stopIntake();

    drive->setMaxVelocity(50);
    drive->moveDistance(-18_in);

    drive->turnAngle(-87_deg);


    lowerArm();
    leftDrive.moveVoltage(-5000);
    rightDrive.moveVoltage(-5000);
    pros::delay(1200);
    stopArm();
    pros::delay(600);
    leftDrive.moveVoltage(0);
    rightDrive.moveVoltage(0);

    slowIntake();

    drive->moveDistance(32_in);
    stopIntake();
    drive->moveDistance(-3_in);

    liftArm();
    pros::delay(1000);
    stopArm();

    drive->moveDistance(2_in);

    slowOuttake();
    pros::delay(2000);
    stopIntake();

    drive->moveDistance(-5_in);


}
////////////////////////////////////////////////////////////////////////////
//A 4 pt auto for the big goal
////////////////////////////////////////////////////////////////////////////
void blueBackNew(){
  //deploy the tray
  deployBack();
  lowerArm();
  pros::delay(200);
  stopArm();
  //start intakes
  Intake();
  //intake 2 cubes
  drive->setMaxVelocity(60);
  drive->moveDistance(14_in);
  //turn to grab the 3rd cube from middle tower
  drive->setMaxVelocity(50);
  drive->turnAngle(-90_deg);

  pros::delay(100);
  //grab cube from middle tower
  drive->moveDistance(6_in);
  //turn to grab the 4th cube
  drive->turnAngle(174_deg);

  //move and grab the 4th cube
  drive->setMaxVelocity(60);
  drive->moveDistance(22_in);
  Outtake();
  pros::delay(400);
  stopIntake();
  //start moving the tiler forward to save time
  tiltForward();
  //turn to face goal
  drive->setMaxVelocity(50);
  drive->turnAngle(42_deg);
  //index the cubes

  //line up for goal
  leftDrive.moveVoltage(3000);
  rightDrive.moveVoltage(3000);

  pros::delay(400);



  pros::delay(1200);
  //deploy cubes
  Outtake();
  tiltBack();
  //back up from the stack
  leftDrive.moveVoltage(-4000);
  rightDrive.moveVoltage(-4000);
  pros::delay(1200);
  leftDrive.moveVoltage(0);
  leftDrive.moveVoltage(0);
  stopTilter();

  drive->turnAngle(-90_deg);

}
////////////////////////////////////////////////////////////////////////////
//A 4 pt auto for the big goal
////////////////////////////////////////////////////////////////////////////
void redBackNew(){
  //deploy the tray
  deployBack();
  lowerArm();
  pros::delay(200);
  stopArm();
  //start intakes
  Intake();
  //intake 2 cubes
  drive->setMaxVelocity(60);
  drive->moveDistance(14_in);
  //turn to grab the 3rd cube from middle tower
  drive->setMaxVelocity(50);
  drive->turnAngle(90_deg);

  pros::delay(100);
  //grab cube from middle tower
  drive->moveDistance(6_in);
  //turn to grab the 4th cube
  drive->turnAngle(-174_deg);

  //move and grab the 4th cube
  drive->setMaxVelocity(60);
  drive->moveDistance(22_in);
  Outtake();
  pros::delay(400);
  stopIntake();
  //start moving the tiler forward to save time
  tiltForward();
  //turn to face goal
  drive->setMaxVelocity(50);
  drive->turnAngle(-42_deg);
  //index the cubes

  //line up for goal
  leftDrive.moveVoltage(3000);
  rightDrive.moveVoltage(3000);

  pros::delay(400);



  pros::delay(1200);
  //deploy cubes
  Outtake();
  tiltBack();
  //back up from the stack
  leftDrive.moveVoltage(-4000);
  rightDrive.moveVoltage(-4000);
  pros::delay(1200);
  leftDrive.moveVoltage(0);
  leftDrive.moveVoltage(0);
  stopTilter();

  drive->turnAngle(90_deg);

}
////////////////////////////////////////////////////////////////////////////
//A 2 point auto for the big goal
////////////////////////////////////////////////////////////////////////////
void redBack(){
  //deploy tray
  deploy();
  //start intakes
  Intake();
  //move and grab 2 cubes
  drive->setMaxVelocity(40);
  drive->moveDistance(15_in);

  pros::delay(300);
  //turn to face 3rd cube
  drive->setMaxVelocity(30);
  drive->turnAngle(-90_deg);

  pros::delay(300);
  //move and grab 3rd cube
  drive->moveDistance(15_in);

  pros::delay(200);
  //back up to align with big goal
  drive->moveDistance(-3_in);

  pros::delay(100);
  //turn to align with big goal
  drive->turnAngle(-30_deg);

  pros::delay(200);
  //drive into big goal using time as to not mess up motors
  leftDrive.moveVoltage(4000);
  rightDrive.moveVoltage(4000);
  pros::delay(600);
  leftDrive.moveVoltage(0);
  rightDrive.moveVoltage(0);
  //outtake 2 cubes
  Outtake();

}
////////////////////////////////////////////////////////////////////////////
//1pt consistent auto
////////////////////////////////////////////////////////////////////////////
void pushcube(){
  //drive back to score 1 cube
  leftDrive.moveVoltage(-6000);
  rightDrive.moveVoltage(-6000);

  pros::delay(3000);

  leftDrive.moveVoltage(0);
  rightDrive.moveVoltage(0);
  //drive forward to not contact the cube
  leftDrive.moveVoltage(6000);
  rightDrive.moveVoltage(6000);

  pros::delay(2000);

  leftDrive.moveVoltage(0);
  rightDrive.moveVoltage(0);
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
imu_sensor.reset();
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
void competition_initialize() {
}

void autonomous() {
  //blueFront();
  //redFront();
  //blueBack();
  //redBack();
  //pushcube();
  Skills();
  //redBackNew();
  //blueBackNew();
  //testIMU();
  //testOdom
}

void opcontrol() {
	 while (true) {
        printf("IMU get rotation: %f degrees\n", imu_sensor.get_rotation());
        deployMacro();
        driveSwap();
        driveControl();
        armControl();
        intakeControl();
        tilterControl();
        tiltMacro();
			  pros::delay(10);
	}
}
