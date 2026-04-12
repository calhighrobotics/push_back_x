#pragma once

#include "pros/motors.hpp" // Needed for pros::Motor
#include <algorithm>       // Needed for std::max
#include <cmath>           // Needed for std::abs

class IntakeAntiJam {
    private:
        pros::Motor& intakeMotor;
        pros::Motor& outtakeMotor;
        pros::Motor& storageMotor;
        
        const int maxTemp;
        
        // Track what we WANT the motors to do
        float cmd_intake;
        float cmd_outtake;
        float cmd_storage;
        
        // Tuning
        float velocity_threshold;
        int jam_trigger_loops;
        int reverse_duration_ms;
        
        int jam_counter;
        bool is_unjamming;
        
        // Toggle for anti-jam detection
        bool anti_jam_enabled;

    public:
        // Constructor declaration
        IntakeAntiJam(pros::Motor& i_motor, pros::Motor& o_motor, pros::Motor& s_motor, int temp = 55);

        // Method declarations
        void set_velocities(float i_vel, float o_vel, float s_vel);
        void stop();
        void update();
        
        // New function to enable/disable anti-jam
        void enable_anti_jam(bool enable);
};