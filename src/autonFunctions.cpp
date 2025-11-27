#include "autonFunctions.h"
#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"


void intake()
{
    intakeMotor.move_voltage(12000);
}

void outtake()
{
    intakeMotor.move_voltage(-12000);
}

void score_bottomgoal()
{
    intakeMotor.move_voltage(-12000);
}

void score_longgoal()
{
    intake();
    topMotor.move_voltage(12000);
}

void score_midgoal()
{
    intake();
    topMotor.move_voltage(12000);
    if(!C.is_extended())
        C.extend();
}

void intake_stop()
{
    intakeMotor.move_voltage(0);
    topMotor.move_voltage(0);
}

void matchload_prep()
{
    C.toggle();
}

void longgoal_prep()
{

}

void reset_odometry()
{

}



