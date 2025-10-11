#include "autonFunctions.h"
#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"


void intake()
{
    if(midRollerHeight.is_extended())
    {
        midRollerHeight.retract();
    }
    agitator.move_voltage(6500);
    intakeMotor.move_voltage(12000);
    topRoller.retract();
    midMotor.move_voltage(-12000);
}

void outtake()
{
    midMotor.move_voltage(12000);
    agitator.move_voltage(-6500);
    intakeMotor.move_voltage(-12000);
}

void score_bottomgoal()
{
    midMotor.move_voltage(12000);
    agitator.move_voltage(-6500);
    intakeMotor.move_voltage(-8000);
}

void score_longgoal()
{
    midMotor.move_voltage(-12000);
    agitator.move_voltage(-6500);
    intakeMotor.move_voltage(12000);
}

void score_midgoal()
{
    agitator.move_voltage(-6500);
    intakeMotor.move_voltage(12000);
    midMotor.move_voltage(12000);
}

void intake_stop()
{
    midMotor.move_voltage(0);
    intakeMotor.move_voltage(0);
    agitator.move_voltage(0);
}

void matchload_prep()
{
    if(aligner.is_extended())
    {
        aligner.retract();
    }
    pros::delay(30);
    if(!matchload_mech.is_extended())
    {
        matchload_mech.extend();
    }
}

void longgoal_prep()
{
    if(matchload_mech.is_extended())
    {
        matchload_mech.retract();
    }
    pros::delay(30);
    if(!aligner.is_extended())
    {
        aligner.extend();
    }
}

