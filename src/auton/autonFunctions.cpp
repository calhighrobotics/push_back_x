#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"


void intake(int power = 12000)
{
    intakeMotor.move_voltage(power);
}

void outtake(int power = 12000)
{
    intakeMotor.move_voltage(-power);
}

void score_bottomgoal(int power = 12000)
{
    intakeMotor.move_voltage(-power);
}

void score_longgoal(int power = 12000)
{
    intake(power);
    topMotor.move_voltage(power);
}

void score_midgoal(int power = 12000)
{
    intake(power);
    topMotor.move_voltage(power);
    if(!A.is_extended()) A.extend();
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



