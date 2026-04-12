#include "IntakeAntiJam.h"
#include "pros/rtos.hpp" // Needed for pros::delay

// Constructor
IntakeAntiJam::IntakeAntiJam(pros::Motor& i_motor, pros::Motor& o_motor, pros::Motor& s_motor, int temp) 
    : intakeMotor(i_motor), outtakeMotor(o_motor), storageMotor(s_motor), maxTemp(temp) 
{
    cmd_intake = 0;
    cmd_outtake = 0;
    cmd_storage = 0;
    
    velocity_threshold = 15.0; // RPM
    jam_trigger_loops = 15;    // 150ms
    reverse_duration_ms = 100; // Spit out time
    
    jam_counter = 0;
    is_unjamming = false;
    anti_jam_enabled = true;   // Default to enabled
}

// Enable or disable anti-jam detection
void IntakeAntiJam::enable_anti_jam(bool enable) {
    anti_jam_enabled = enable;
    
    // Reset the counter if disabled so it doesn't instantly trigger when re-enabled
    if (!anti_jam_enabled) {
        jam_counter = 0;
    }
}

// Set Velocities
void IntakeAntiJam::set_velocities(float i_vel, float o_vel, float s_vel) {
    cmd_intake = i_vel;
    cmd_outtake = o_vel;
    cmd_storage = s_vel;
    
    if (!is_unjamming) {
        intakeMotor.move_velocity(cmd_intake);
        outtakeMotor.move_velocity(cmd_outtake);
        storageMotor.move_velocity(cmd_storage);
    }
}

// Stop
void IntakeAntiJam::stop() {
    set_velocities(0, 0, 0);
    if (!is_unjamming) {
        intakeMotor.brake();
        outtakeMotor.brake();
        storageMotor.brake();
    }
}

// Update (Run this in your background task)
void IntakeAntiJam::update() {
    // 1. Safety Temp Check (Always runs, regardless of anti-jam state)
    double max_t = std::max({intakeMotor.get_temperature(), 
                             outtakeMotor.get_temperature(), 
                             storageMotor.get_temperature()});
    if (max_t > maxTemp) {
        intakeMotor.move_velocity(0);
        outtakeMotor.move_velocity(0);
        storageMotor.move_velocity(0);
        return;
    }

    // If anti-jam is disabled, skip the detection and unjamming logic
    if (!anti_jam_enabled) {
        return; 
    }

    // 2. Jam Detection
    bool is_stuck = false;
    if (std::abs(cmd_intake) > 0 && std::abs(intakeMotor.get_actual_velocity()) < velocity_threshold) is_stuck = true;
    if (std::abs(cmd_storage) > 0 && std::abs(storageMotor.get_actual_velocity()) < velocity_threshold) is_stuck = true;

    if (is_stuck) {
        jam_counter++;
    } else {
        jam_counter = 0;
    }

    // 3. Unjam Action
    if (jam_counter > jam_trigger_loops) {
        is_unjamming = true; 
        
        intakeMotor.move_velocity(-cmd_intake);
        outtakeMotor.move_velocity(-cmd_outtake);
        storageMotor.move_velocity(-cmd_storage);
        
        pros::delay(reverse_duration_ms); 
        
        intakeMotor.move_velocity(cmd_intake);
        outtakeMotor.move_velocity(cmd_outtake);
        storageMotor.move_velocity(cmd_storage);
        
        jam_counter = 0;
        is_unjamming = false; 
        
        pros::delay(150); 
    }
}