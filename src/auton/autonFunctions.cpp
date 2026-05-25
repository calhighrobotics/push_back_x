// Copyright 2026 California High Robotics, Team 1516X
// SPDX-License-Identifier: GPL-3.0-or-later

#include "autonFunctions.h"
#include "globals.h" 
#include "lemlib/util.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <sys/types.h>
#include "colorSort.h"
#include "IntakeAntiJam.h"
#include "ltv.h"


const VelocityControllerConfig config{
5.09498070723,
0.23396112183,
1.22496514644,
0.964796393603,
0.522291806997,
10.933332351,
44.6788428062,
};

void antiJamTask() {
    while (true) {
        jamManager.update();
        pros::delay(10);
    }
}

void intake(int power)
{
    if(hood.is_extended()) { hood.retract(); }
    jamManager.set_velocities((int)power/3, -(int)power/3, power);
}

void outtake(int power)
{
    if(!intake_lift.is_extended()) { intake_lift.extend(); }
    jamManager.set_velocities(-((int)power/3), ((int)power/3), -power);
}

void score_longgoal(int power, Color allianceColor)
{
    if(!hood.is_extended()) { hood.extend(); }
    jamManager.set_velocities((int)power/3, -(int)power/3, power);
}

void intake_stop(bool hood_state)
{
    jamManager.stop();
    if(!hood_state) hood.retract();
    else hood.extend();
    
    if(intake_lift.is_extended()) { intake_lift.retract(); }
}

void score_midgoal(int power)
{
    jamManager.set_velocities((int)power/3, (int)power/3, power);
}

void score_midgoal_auton(int power, Color allianceColor, int time)
{
    u_int32_t start_time = pros::millis();
    if(time != -1) {   
        while(pros::millis() - start_time < (u_int32_t)time) {
            jamManager.set_velocities((int)power/3, (int)power/3, power);
            chassis.tank(-20, -20);
            pros::delay(10);
        }
    }
    chassis.brake();
    intake_stop();
}

void score_longgoal_auton(int power, Color allianceColor, int time)
{
    u_int32_t start_time = pros::millis();
    if(!hood.is_extended()) {hood.extend();}
    if(time != -1) {   
        while(pros::millis() - start_time < (u_int32_t)time) {
            if(allianceColor != Color::NONE)
            {
                Color detected = get_color();
                if(detected != Color::NONE && detected != allianceColor && color_sort_enable)
                {
                    intake_stop();
                    jamManager.set_velocities(-200, 600, -200);
                    pros::delay(100);
                    intake_stop();
                    break;
                }
            }
            jamManager.set_velocities((int)power/3, -(int)power/3, power);
            chassis.tank(-20, -20);
            pros::delay(10); 
        }
    }
    chassis.brake();
    intake_stop();
}
void intake_to_basket()
{

}

void matchload_state(bool state)
{
    if(state)
    {
        if(!matchloader.is_extended()) {matchloader.extend();}
    }
    else
    {
        if(matchloader.is_extended()) {matchloader.retract();}
    }
}


void relativeMotion(float expected_x, float expected_y, float expected_theta, float distance, int timeout_ms, bool forw, float earlyExit)
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