#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "auton/autonFunctions.h"

void right_auton()
{
    
}

void carry_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,12,2000);
}

void left_auton() {}

void elim_auton() {}

void awp_auton() {}

void skills_auton() {}