#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FrontLeft            motor         20              
// BackLeft             motor         19              
// BackRight            motor         10              
// FrontRight           motor         9               
// Tilter               motor         18              
// IntakeL              motor         11              
// IntakeR              motor         1               
// Lift                 motor         8               
// Pot                  pot           A               
// Inertial             inertial      12              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include <list>
#include <algorithm>
#include <fstream>
#include <iostream>

using namespace vex;


// A global instance of competition
competition Competition;

motor_group   leftDrive( FrontLeft, BackLeft);
motor_group   rightDrive( FrontRight, BackRight);
smartdrive    robotDrive( leftDrive, rightDrive, Inertial, 12.56, 16, 16, distanceUnits::in );

float driveMode = 0;
int tilterRetract = 400;
// End of gloabal variables
// Start of PID
//settings
double kP = 0.0001;
double kI = 0.000001;
double kD = 0.3;
double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

//auto settings
int desiredValue = 200;
int desiredTurnValue = 0;

int error;//SensorValue - Desired Value 
int prevError = 0;//Position 20 mi ago
int derivative;//er- prev er : speed
int totalError = 0;

int turnError;//SensorValue - Desired Value 
int turnPrevError = 0;//Position 20 mi ago
int turnDerivative;//er- prev er : speed
int turnTotalError = 0;

bool resetDriveSensors = false;

//varmodified for use
bool enableDrivePID = true;

int drivePID(){

  while(enableDrivePID){

    if(resetDriveSensors){
      resetDriveSensors = false;

    FrontLeft.setPosition(0, degrees);
    BackLeft.setPosition(0, degrees);
    FrontRight.setPosition(0, degrees);
    BackRight.setPosition(0, degrees);

    }

    int leftMotorPosition = FrontLeft.position(degrees);
    int righttMotorPosition = FrontRight.position(degrees);

    /////////////////////////////////////////////////////////
    //Lateral Movement PID
    ////////////////////////////////////////////////////////
    int averagePosition = (leftMotorPosition + righttMotorPosition)/2;

    //Potential
    error = averagePosition - desiredValue;
    //Derivative
    derivative = error - prevError;
    //Integral
    //totalError += error;

    double lateralMotorPower = error * kP + derivative * kD;

    /////////////////////////////////////////////////////////
    //Turning Movement PID
    ////////////////////////////////////////////////////////

    int turnDifference = (leftMotorPosition - righttMotorPosition)/2;

    //Potential
    turnError = turnDifference - desiredTurnValue;
    //Derivative
    turnDerivative = turnError - turnPrevError;
    //Integral
    //turnTotalError += turnError;

    double turnMotorPower = turnError * turnkP + derivative * turnkD;

    ////////////////////////////////////////////////////////

    FrontLeft.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    BackLeft.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    FrontRight.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    BackRight.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }

  return 1;
}

// End of Mathematical functions
//Start of Autonomous Phase functions

/* Created by Josh
This function allows us to set the speeds of the left and right
side of the drive train to a certain value until the built in Encoders
inside of the V5 Smart Motors change by a certain ammount. It
is useful for programming autos, and it is more accurate than
just running motors for a set amount of time.*/

void autoDriveEncoder(int left, int right, double distance){ 
    double startEncoder = FrontLeft.rotation(rotationUnits::deg);
    while(fabs(FrontLeft.rotation(rotationUnits::deg) - startEncoder) < distance){
        FrontLeft.spin(directionType::fwd, left, velocityUnits::pct);
        BackLeft.spin(directionType::fwd, left, velocityUnits::pct);
        FrontRight.spin(directionType::fwd, right, velocityUnits::pct);
        FrontLeft.spin(directionType::fwd, right, velocityUnits::pct);
    }
    FrontLeft.stop();
    FrontRight.stop();
    BackLeft.stop();
    BackRight.stop();
    task::sleep(100);
}

/* Created by Josh
This function allows us to set the speeds of the left and right
side of the drive train for a set ammount of time. It is useful
for programming autos in places where encoders tend to fail.*/

void autoDriveTime(int left, int right, int time){
    FrontLeft.spin(directionType::fwd, left, velocityUnits::pct);
    BackLeft.spin(directionType::fwd, left, velocityUnits::pct);
    FrontRight.spin(directionType::fwd, right, velocityUnits::pct);
    BackRight.spin(directionType::fwd, right, velocityUnits::pct);
    task::sleep(time);
    FrontLeft.stop();
    FrontRight.stop();
    BackLeft.stop();
    BackRight.stop();
    task::sleep(50);
}

/* Created by Josh
This function allows us to set the speeds of the left and right
side of the drive train for a set ammount of time. It is useful
for programming autos in places where encoders tend to fail.*/

void autoTurnEncoder(int left, int right, int value){
    BackRight.resetRotation();
    while(fabs(BackRight.rotation(rotationUnits::deg)) < value){
        FrontLeft.spin(directionType::fwd, left, velocityUnits::pct);
        BackLeft.spin(directionType::fwd, left, velocityUnits::pct);
        FrontRight.spin(directionType::fwd, right, velocityUnits::pct);
        BackRight.spin(directionType::fwd, right, velocityUnits::pct);
        task::sleep(1);
    }
    FrontLeft.stop();
    FrontRight.stop();
    BackLeft.stop();
    BackRight.stop();
}

/* Created by Josh
This function allows us to set the speeds of the left and right
side of the drive train for a set ammount of time. It is useful
for programming autos in places where encoders tend to fail.*/

void autoTurnEncoder2(int left, int right, int value){
    BackLeft.resetRotation();
    while(fabs(BackLeft.rotation(rotationUnits::deg)) < value){
        FrontLeft.spin(directionType::fwd, left, velocityUnits::pct);
        BackLeft.spin(directionType::fwd, left, velocityUnits::pct);
        FrontRight.spin(directionType::fwd, right, velocityUnits::pct);
        BackRight.spin(directionType::fwd, right, velocityUnits::pct);
        task::sleep(1);
    }
    FrontLeft.stop();
    FrontRight.stop();
    BackLeft.stop();
    BackRight.stop();
}

/* Created by Josh
This function uses a PID (Proportion Integral Derivative)
loop in order to better control the drive train while it approaches
a target encoder value. It will slow down and adjust it's speed
mathematically to insure the results that come out of it will be accurate.
*/




    



/* Created by Josh

*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void autoSKILLS(){
   
}

/* Created by Josh

*/


 
/* Created by Josh
This auto, blueFrontNew, takes a ball underneath the slanted cap
near the front tile. It then goes back toward the starting tile,
turns, and flips another cap. Afterward, it does a swing turn to 
quickly even out against the wall and clears out the post of flags
on the blue side.
*/
void redFront(){
    //drive forward and intake 4 cubes
    IntakeL.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    IntakeR.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    Lift.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    task::sleep(1000);
    Lift.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    task::sleep(600);
    IntakeR.spin(directionType::fwd, 100, velocityUnits::pct);
    IntakeL.spin(directionType::fwd, 100, velocityUnits::pct); 
    task::sleep(800);
    Lift.stop();
    autoDriveEncoder(20,20,1000);
    //drive back
    autoDriveEncoder(-40,-40,500);
    IntakeR.spin(directionType::fwd, 40, velocityUnits::pct);
    IntakeL.spin(directionType::fwd, 40, velocityUnits::pct); 
    task::sleep(300);
    //turn to score goal
    autoTurnEncoder(40, -40, 190);
    IntakeL.stop();
    IntakeR.stop();
    //drive forward to the goal
    autoDriveTime(20, 20, 900);
    IntakeL.spin(directionType::rev, 45, velocityUnits::pct);
    IntakeR.spin(directionType::rev, 45, velocityUnits::pct);
    task::sleep(300);
    //score in the goal
    Tilter.spin(directionType::fwd, 100,velocityUnits::pct);
    task::sleep(2300);
    Tilter.stop();
    autoDriveEncoder(-20,-20,100);
    IntakeL.stop();
    IntakeR.stop();
}
void blueFront(){
    //drive forward and intake 4 cubes
    IntakeL.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    IntakeR.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    Lift.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    task::sleep(1000);
    Lift.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    task::sleep(600);
    IntakeR.spin(directionType::fwd, 100, velocityUnits::pct);
    IntakeL.spin(directionType::fwd, 100, velocityUnits::pct); 
    task::sleep(800);
    Lift.stop();
    autoDriveEncoder(20,20,1000);
    //drive back
    autoDriveEncoder(-40,-40,500);
    IntakeR.spin(directionType::fwd, 40, velocityUnits::pct);
    IntakeL.spin(directionType::fwd, 40, velocityUnits::pct); 
    task::sleep(300);
    //turn to score goal
    autoTurnEncoder(-40, 40, 210);
    IntakeL.stop();
    IntakeR.stop();
    //drive forward to the goal
    autoDriveTime(40, 40, 1200);
    IntakeL.spin(directionType::rev, 45, velocityUnits::pct);
    IntakeR.spin(directionType::rev, 45, velocityUnits::pct);
    task::sleep(300);
    //score in the goal
    Tilter.spin(directionType::fwd, 100,velocityUnits::pct);
    task::sleep(2300);
    Tilter.stop();
    autoDriveEncoder(-20,-20,100);
    IntakeL.stop();
    IntakeR.stop();
}

void blueBack(){
    IntakeL.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    IntakeR.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    Lift.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    task::sleep(1000);
    Lift.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    task::sleep(600);
    IntakeR.spin(directionType::fwd, 100, velocityUnits::pct);
    IntakeL.spin(directionType::fwd, 100, velocityUnits::pct); 
    task::sleep(800);
    Lift.stop();
    autoDriveEncoder(20,20,1000);
    //turn to goal
    autoTurnEncoder(40, -40, 190);
    //drive back
    autoDriveEncoder(20,20,1000);
    //intake cube drop
    IntakeL.spin(directionType::rev, 45, velocityUnits::pct);
    IntakeR.spin(directionType::rev, 45, velocityUnits::pct);
    task::sleep(300);
    //score in the goal
    Tilter.spin(directionType::fwd, 100,velocityUnits::pct);
    task::sleep(2300);
    Tilter.stop();
    autoDriveEncoder(-20,-20,100);
    IntakeL.stop();
    IntakeR.stop();
}


void PIDtest(){
resetDriveSensors= true;
desiredValue=300;
desiredTurnValue=600;
task::sleep(1000);
resetDriveSensors= true;
desiredValue=300;
desiredTurnValue=300;


}


void redBack(){
        FrontLeft.spin(vex::directionType::fwd, -50, vex::velocityUnits::pct);
        BackLeft.spin(vex::directionType::fwd, -50, vex::velocityUnits::pct);
        FrontRight.spin(vex::directionType::fwd, -50, vex::velocityUnits::pct);
        BackRight.spin(vex::directionType::fwd, -50, vex::velocityUnits::pct);
        task::sleep(5000);
        FrontLeft.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        BackLeft.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        FrontRight.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        BackRight.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        task::sleep(1000);
        FrontLeft.stop();
        BackLeft.stop();
        FrontRight.stop();
        BackRight.stop();   
}
 void Skill(){
   IntakeL.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    IntakeR.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    Lift.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    task::sleep(1000);
    Lift.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    task::sleep(600);
    IntakeR.spin(directionType::fwd, 100, velocityUnits::pct);
    IntakeL.spin(directionType::fwd, 100, velocityUnits::pct); 
    task::sleep(800);
    Lift.stop();
    autoDriveEncoder(20,20,1000);
    //go backwards
    autoTurnEncoder(-30, 30, 100);
    autoDriveEncoder(-20,-20,700);
    //go for 3 cubes
    autoTurnEncoder(30, -30, 100);
    IntakeR.spin(directionType::fwd, 100, velocityUnits::pct);
    IntakeL.spin(directionType::fwd, 100, velocityUnits::pct); 
    task::sleep(800);
    autoDriveEncoder(20,20,1000);
    //turn towards goal
    autoTurnEncoder(40, -40, 190);
    autoDriveEncoder(30,30,800);
    //score in goal
    Tilter.spin(directionType::fwd, 100,velocityUnits::pct);
    task::sleep(2300);
    Tilter.stop();
    autoDriveEncoder(-20,-20,100);
    IntakeL.stop();
    IntakeR.stop();
    //
 }

 void drivetrainTest(){
  robotDrive.driveFor(forward, 36, inches);
  robotDrive.turnFor(right, 90, degrees, 30, velocityUnits::pct);
}

//End of Autonomous Phase functions
// Start of Drive Phase functions

/* 
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void trayRetract(){
  bool waitForCompletion = false;
  Tilter.setVelocity(-100, velocityUnits::pct);
  Tilter.spinToPosition(tilterRetract, degrees, waitForCompletion);
  Tilter.setBrake(brakeType::hold);

}

void Retract(){
Controller1.ButtonY.pressed(trayRetract);
}


void manualDrive(){ 
    if(driveMode == 0) {
        FrontLeft.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::velocityUnits::pct); //(Axis3+Axis4)/2
        BackLeft.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::velocityUnits::pct); //(Axis3+Axis4)/2
        FrontRight.spin(vex::directionType::fwd, Controller1.Axis2.value(), vex::velocityUnits::pct);//(Axis3-Axis4)/2
        BackRight.spin(vex::directionType::fwd, Controller1.Axis2.value(), vex::velocityUnits::pct);//(Axis3-Axis4)/2
    }
    else if(driveMode == 1) {
       FrontLeft.spin(directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis1.value())/2, velocityUnits::pct); //(Axis3+Axis4)/2;
       BackLeft.spin(directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis1.value())/2, velocityUnits::pct); //(Axis3+Axis4)/2;
	     FrontRight.spin(directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value())/2, velocityUnits::pct);//(Axis3-Axis4)/2;
	     BackRight.spin(directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value())/2, velocityUnits::pct);//(Axis3-Axis4)/2;
    }
}
void slowIntake(){ 
  if(Controller1.ButtonLeft.pressing()) {
            IntakeL.spin(directionType::fwd, 50,velocityUnits::pct);
            IntakeR.spin(directionType::fwd, 50,velocityUnits::pct);
        }
        else if(Controller1.ButtonRight.pressing()) {
            IntakeL.spin(directionType::rev, 50,velocityUnits::pct);
            IntakeR.spin(directionType::rev, 50,velocityUnits::pct);
        }        else {
            IntakeL.stop(brakeType::brake);
            IntakeR.stop(brakeType::brake);
        }
        
}
void trigger() {
  if(driveMode == 0) {
    wait(100,msec);
    driveMode = 1;
  }
  else if(driveMode == 1) {
    wait(100,msec);
    driveMode = 0;
  }
}

void driveSwap() {
  Controller1.ButtonA.pressed(trigger);
} 

void manualLift(){ 
     if(Controller1.ButtonR2.pressing()) {
            Lift.spin(directionType::fwd, 100,velocityUnits::pct);
            Tilter.spin(directionType::fwd, 75,velocityUnits::pct);
        }
        else if(Controller1.ButtonR1.pressing()) {
            Lift.spin(directionType::rev, 100, velocityUnits::pct);
            Tilter.spin(directionType::rev, 75, velocityUnits::pct);
        }
        else {
            Lift.stop(brakeType::brake);
        }
}

void manualIntake(){ 
     if(Controller1.ButtonL1.pressing()) {
            IntakeL.spin(directionType::fwd, 100,velocityUnits::pct);
            IntakeR.spin(directionType::fwd, 100,velocityUnits::pct);
        }
        

}
void manualOuttake(){
         if(Controller1.ButtonL2.pressing()&& Pot.angle(rotationUnits::deg) >= 200) {
            IntakeL.spin(directionType::rev, 50, velocityUnits::pct);
            IntakeR.spin(directionType::rev, 50, velocityUnits::pct);
        }else if(Controller1.ButtonL2.pressing()&& Pot.angle(rotationUnits::deg) <= 200) {
            IntakeL.spin(directionType::rev, 100, velocityUnits::pct);
            IntakeR.spin(directionType::rev, 100, velocityUnits::pct);
    }
}


void manualTilter(){ 
     if(Controller1.ButtonX.pressing()) {
            Tilter.spin(directionType::fwd, 100,velocityUnits::pct);
        }
        else if(Controller1.ButtonB.pressing()) {
            Tilter.spin(directionType::rev, 100, velocityUnits::pct);
        }
}
void manualTilterSlow(){ 
     if(Controller1.ButtonUp.pressing()) {
            Tilter.spin(directionType::fwd, 50,velocityUnits::pct);
        }
        else if(Controller1.ButtonDown.pressing()) {
            Tilter.spin(directionType::rev, 50, velocityUnits::pct);
        }
        else {
            Tilter.stop(brakeType::brake);
        }
}


////////////////////////////////////////////////
////////////////////////////////////////////////


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


void autonomous(void) {

    vex::task bruh(drivePID);
    enableDrivePID = true;
    PIDtest();
    //drivetrainTest();
    //redBack();
    //redFront();
    //blueFront();
    //blueBack();
    //redFront();
    //blueFront();
    task::sleep(5);

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {   
        enableDrivePID = false;
        Retract();
        driveSwap();
        slowIntake();
        manualIntake();
        manualTilterSlow();
        manualDrive();
        manualLift();
        manualTilter();
        manualOuttake();
        //doubleShot();
        task::sleep(5);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
