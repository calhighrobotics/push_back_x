#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "MCL.h"
#include "pros/rtos.hpp"

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
    if(!trapDoor.is_extended()) trapDoor.extend();
}

void intake_stop()
{
    intakeMotor.move_voltage(0);
    topMotor.move_voltage(0);
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

void matchload_wiggle(int time = 1000)
{
    u_int32_t start_time = pros::millis();
    int sign = 1;
   while(pros::millis() - start_time < 1000)
   {
        leftMotors.move_voltage(3000 * sign);
        rightMotors.move_voltage(3000 * sign);
        sign *= -1;
   }
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




