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

void autonomous() {
	drive->setMaxVelocity(30);
	drive->moveDistance(12_in); // Drive forward 12 inches
	drive->turnAngle(90_deg);   // Turn in place 90 degrees
}
