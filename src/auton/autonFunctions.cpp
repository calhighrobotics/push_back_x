#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>
#include "colorSort.h"

void intake(int power = 12000)
{
    intakeFunnel.extend();
    intakeMotor.move_voltage(power);
    topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    topMotor.brake();
}

void outtake(int power = 12000)
{
    topMotor.move_voltage(-12000);
    if(low_ramp_down_time >= 1000)
        intakeMotor.move_voltage( -12000);
    else
        intakeMotor.move_voltage(-power);
}

void score_bottomgoal(int power = 12000)
{
    intakeMotor.move_voltage(-power);
}

void score_longgoal(int power = 12000, Color allianceColor = Color::RED)
{
    intake(power);
    if(get_color() != allianceColor && get_color() != Color::NONE && color_sort_enable)
    {
        topMotor.move_voltage(-8000);
        pros::delay(33);
        std::cout << "Color Rejected" << std::endl;
    }
    else
    {
        topMotor.move_voltage(power);
        std::cout << "Color Accepted" << std::endl;
    }

}

void intake_stop()
{
    intakeMotor.move_voltage(0);
    topMotor.move_voltage(0);
}

void score_midgoal(int power = 12000)
{
    if(get_color() != allianceColor && get_color() != Color::NONE && color_sort_enable)
    {
        topMotor.move_voltage(-10000);
        pros::delay(33);
    }
    else {
        if (ramp_up_time >= 800)
        {
            topMotor.move_voltage(5000); // Prev: 4000
            intakeMotor.move_voltage(12000);
        }
        else if(ramp_up_time >= 400)
        {
            topMotor.move_voltage(6000);
            intakeMotor.move_voltage(12000);
        }
        else {
            topMotor.move_voltage(10000);
            intakeMotor.move_voltage(12000);
        }
    }
    if(midgoal_first)
    {
        if(!color_sort_enable)
        {
            midgoal_first = false;
            trapDoor.extend();
            intakeMotor.move_voltage(12000);
            topMotor.move_voltage(6000);
        }
        else if(color_sort_enable && get_color() == allianceColor)
        {
            midgoal_first = false;
            trapDoor.extend();
            intakeMotor.move_voltage(12000);
            topMotor.move_voltage(6000);
        }
    }
    //if (ramp_up_time >= 1600)
    //{
    //    topMotor.move_voltage(4000);
    //}
    //topMotor.move_voltage(10000);
    
}


void score_longgoal_auton(int power = 12000, Color allianceColor = Color::RED)
{
    leftMotors.move(-50);
    rightMotors.move(-50);
    score_longgoal(power, allianceColor);
}

void intake_to_basket()
{
    intake();
    topMotor.move_voltage(-6000);
}



void resting_state(bool trapDoor_commanded = false)
{
    intake_stop();
    if(!trapDoor_commanded)
        trapDoor.retract();
    topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void matchload_state(bool state)
{
    if(state)
        matchload.extend();
    else if(!state)
        matchload.retract();
}

void longgoal_prep()
{

}

void reset_odometry()
{

}

void matchload_wiggle(int time = 1000, int speed = 100)
{
    uint32_t startTime = pros::millis();
    double initialHeading = chassis.getPose().theta;
    double targetHeading = std::round(initialHeading / 90.0) * 90.0;
    bool flip = false;

    while ((pros::millis() - startTime) < time) {
        if (flip) {
            leftMotors.move(-speed);
            rightMotors.move(speed);
        } else {
            leftMotors.move(speed);
            rightMotors.move(-speed);
        }
        flip = !flip;
        pros::delay(100);
    }

    leftMotors.brake();
    rightMotors.brake();
    pros::delay(50);
    chassis.turnToHeading(targetHeading, 800);
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

void matchload_counter(int balls, int time_ms)
{
    pros::Task counter_task([=]() {
        int count = 0;
        uint32_t start_time = pros::millis();
        while(pros::millis() - start_time < time_ms && count < balls)
        {
            start_time = pros::millis();
            if(frontDistance.get_object_size() <= 70)
            {
                count++;
            }
            pros::Task::delay_until(&start_time, 10);
        }
        intake_stop();
        pros::Task::current().remove();
    });
}
/*
void score_from_basket()
{
    color_sort_enable = false;
    topMotor.move(12000);     
    intakeMotor.move(-12000);
}
*/