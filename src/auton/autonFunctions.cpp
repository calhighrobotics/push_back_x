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

void matchload_state(bool state)
{
    if(state && !C.is_extended())
    {
        C.extend();
    }
    else if(C.is_extended())
    {
        C.retract();
    }
    else {
        std::cout << "Matchload state unchanged\n" << std::endl;
    }
}

void longgoal_prep()
{

}

void reset_odometry()
{

}

void match_load_wiggle(int time = 1000)
{
    u_int32_t  start_time = pros::millis();
    while (pros::millis() - start_time < time) {
        chassis.curvature(50, 0.5, false);
        pros::delay(100);
        chassis.curvature(50, -0.5, false);
        pros::delay(100);
    }
}



