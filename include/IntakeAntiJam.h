#pragma once

#include "pros/motors.hpp" 
#include <algorithm>      
#include <cmath>           

class IntakeAntiJam {
    private:
        pros::Motor& intakeMotor;
        pros::Motor& outtakeMotor;
        pros::Motor& storageMotor;
        
        const int maxTemp;
    
        float cmd_intake;
        float cmd_outtake;
        float cmd_storage;
    
        int jam_trigger_loops;
        int reverse_duration_ms;
        
        int jam_counter;
        bool is_unjamming;
    
        bool anti_jam_enabled;

    public:
        IntakeAntiJam(pros::Motor& i_motor, pros::Motor& o_motor, pros::Motor& s_motor, int temp = 55);

        void set_velocities(float i_vel, float o_vel, float s_vel);
        void stop();
        void update();
        
        void enable_anti_jam(bool enable);
};