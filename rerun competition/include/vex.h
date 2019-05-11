/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "v5.h"
#include "v5_vcs.h"

vex::brain      Brain;
vex::motor      leftMotorFront(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor      leftMotorMiddle(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor      leftMotorBack(vex::PORT3, vex::gearSetting::ratio18_1, false);
vex::motor      rightMotorFront(vex::PORT4, vex::gearSetting::ratio18_1, true);
vex::motor      rightMotorMiddle(vex::PORT5, vex::gearSetting::ratio18_1, true);
vex::motor      rightMotorBack(vex::PORT6, vex::gearSetting::ratio18_1, true);
vex::controller con(vex::controllerType::primary);
