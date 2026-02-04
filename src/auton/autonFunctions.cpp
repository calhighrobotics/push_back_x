#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "MCL.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>
#include "colorSort.h"

void intake(int power = 12000)
{
    intakeMotor.move_voltage(power);
    topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    topMotor.brake();
}

void outtake(int power = 12000)
{
    
    if(low_ramp_down_time >= 400)
        intakeMotor.move_voltage( -6000);
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
        topMotor.move_voltage(-12000);
        pros::delay(100);
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
        topMotor.move_voltage(-8000);
    }
    else {
        if (ramp_up_time >= 400)
        {
            topMotor.move_voltage(4000);
            intakeMotor.move_voltage(10000);
        }
        else {
            topMotor.move_voltage(12000);
            intakeMotor.move_voltage(12000);
        }
    }
    if(midgoal_first)
    {
        //topMotor.move_voltage(-12000);
        //pros::delay(250);
        midgoal_first = false;
        if(!trapDoor.is_extended()) trapDoor.extend();
        intakeMotor.move_voltage(12000);
        topMotor.move_voltage(6000);
        pros::delay(500);
    }
    //if (ramp_up_time >= 1600)
    //{
    //    topMotor.move_voltage(4000);
    //}
    //topMotor.move_voltage(10000);
    
}

void score_longgoal_auton(int power = 12000, Color allianceColor = Color::RED)
{
    leftMotors.move(-30);
    rightMotors.move(-30);
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
    descore.extend();
}

void matchload_state(bool state)
{
    if(state && !matchload.is_extended())
    {
        matchload.extend();
    }
    else if(matchload.is_extended())
    {
        matchload.retract();
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

void matchload_wiggle(int time = 1000, int speed = 100)
{
    u_int32_t start_time = pros::millis();
    int sign = 1;
   while(pros::millis() - start_time < time)
   {
        leftMotors.move_voltage(3000 * sign);
        rightMotors.move_voltage(3000 * sign);
        sign *= -1;
        pros::delay(speed);
   }
   leftMotors.brake();
   rightMotors.brake();
}

void MCL_reset(bool x = true, bool y = true)
{
    MCL::particle_mutex.take();
    float X = x ? MCL::X : chassis.getPose().x;
    float Y = y ? MCL::Y : chassis.getPose().y;
    MCL::particle_mutex.give();
    chassis.setPose(X, Y, chassis.getPose().theta);
}

pros::Task* fusionTask = nullptr;

void fusion_loop_fn(void* ignore) {
    const uint32_t LOOP_DELAY_MS = 10;
    uint32_t start_time = pros::millis();

    const float MCL_GAIN = 0.05f; 

    while (true) {
        lemlib::Pose odomPose = chassis.getPose(true);
        
        MCL::particle_mutex.take();
        float target_x = MCL::X;
        float target_y = MCL::Y;
        MCL::particle_mutex.give();

        float new_x = odomPose.x + (target_x - odomPose.x) * MCL_GAIN;
        float new_y = odomPose.y + (target_y - odomPose.y) * MCL_GAIN;
        chassis.setPose(new_x, new_y, odomPose.theta);

        pros::Task::delay_until(&start_time, LOOP_DELAY_MS);
    }
}

void enable_fused_odometry(bool enable) {
    if (enable) {
        if (fusionTask == nullptr) {
            fusionTask = new pros::Task(fusion_loop_fn, NULL, "MCL_Fusion");
        }
    } else {
        if (fusionTask != nullptr) {
            fusionTask->remove(); 
            delete fusionTask;  
            fusionTask = nullptr; 
        }
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