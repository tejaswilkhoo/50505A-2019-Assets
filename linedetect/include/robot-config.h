#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
vex::brain Brain;
vex::controller Controller = vex::controller();

vex::motor Leftdrivefront = vex::motor(vex::PORT2,vex::gearSetting::ratio18_1,true);
vex::motor Leftdriveback = vex::motor(vex::PORT11,vex::gearSetting::ratio18_1,true);
vex::motor Rightdrivefront = vex::motor(vex::PORT10,vex::gearSetting::ratio18_1,false);
vex::motor Rightdriveback = vex::motor(vex::PORT20,vex::gearSetting::ratio18_1,false);

vex::motor Intake = vex::motor(vex::PORT5,vex::gearSetting::ratio18_1,false);

vex::motor Angle = vex::motor(vex::PORT15,vex::gearSetting::ratio18_1,false);
vex::motor Descorer = vex::motor(vex::PORT16,vex::gearSetting::ratio18_1,false);
vex::motor Shooter = vex::motor(vex::PORT6,vex::gearSetting::ratio36_1,true);

vex::line Line = vex::line(Brain.ThreeWirePort.F);
