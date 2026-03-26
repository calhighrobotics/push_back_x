#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>
#include "colorSort.h"

void intake(int power = 600)
{
    if(hood.is_extended())
    {
        hood.retract();
    }
    intakeMotor.move_velocity(power);
    outtakeMotor.move_velocity(-power);
}

void outtake(int power =  600)
{
    if(!intake_lift.is_extended())
        intake_lift.extend();
    intakeMotor.move_velocity(-power);
    outtakeMotor.move_velocity(power);
}

void score_longgoal(int power = 600, Color allianceColor = Color::RED)
{
    if(!hood.is_extended())
    {
        hood.extend();
    }
    intakeMotor.move_velocity(power);
    outtakeMotor.move_velocity(-power);
    storageMotor.move_velocity(-power);
}

void intake_stop(bool hood_state = false)
{
    intakeMotor.brake();
    outtakeMotor.brake();
    storageMotor.brake();
    if(!hood_state)
        hood.retract();
    else
        hood.extend();
    if(!intake_lift.is_extended())
    {
        intake_lift.retract();
    }
}

void score_midgoal(int power = 600)
{
    intakeMotor.move_velocity(power);
    outtakeMotor.move_velocity(-power);
    storageMotor.move_velocity(power);
}

void score_midgoal_auton(int power = 12000, Color allianceColor = Color::RED, int time = -1)
{

    
}

void score_longgoal_auton(int power = 12000, Color allianceColor = Color::RED, int time = -1)
{

}

void intake_to_basket()
{

}

void matchload_state(bool state)
{
    if(state)
    {
        if(matchloader.is_extended()) {matchloader.retract();}
    }
    else
    {
        if(!matchloader.is_extended()) {matchloader.extend();}
    }
}


void relativeMotion(float expected_x, float expected_y, float expected_theta, float distance, int timeout_ms, bool forw = true, float earlyExit = 0)
{
    lemlib::Pose targetPose(
        expected_x + distance * std::sin(lemlib::degToRad(expected_theta)),
        expected_y + distance * std::cos(lemlib::degToRad(expected_theta)),
        expected_theta
    );

    chassis.moveToPoint(targetPose.x, targetPose.y, timeout_ms, {.forwards = forw, .earlyExitRange = earlyExit});
}

double calculateAngle(double robotHeading) {
    double d1 = frontDistance.get();
    double d2 = frontDistance2.get() + 10;

    if (d1 > 1800 || d2 > 1800) return -1;
    double wallAngleDeg = atan2((d1 - d2) * 0.0393701, 10.75) * 180.0 / M_PI;

    if ((robotHeading >= 315 && robotHeading < 360) || (robotHeading >= 0 && robotHeading < 45)) {
        return wallAngleDeg;
    }
    else if (robotHeading >= 45 && robotHeading < 135) {
        return 90 + wallAngleDeg;
    }
    else if (robotHeading >= 135 && robotHeading < 225) {
        return 180 + wallAngleDeg;
    }
    else if (robotHeading >= 225 && robotHeading < 315) {
        return 270 + wallAngleDeg;
    }

    return -1;
}