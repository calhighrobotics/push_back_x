#include "IntakeAntiJam.h"
#include "pros/rtos.hpp" 
#include "globals.h"

IntakeAntiJam::IntakeAntiJam(pros::Motor& i_motor, pros::Motor& o_motor, pros::Motor& s_motor, int temp) 
    : intakeMotor(i_motor), outtakeMotor(o_motor), storageMotor(s_motor), maxTemp(temp) 
{
    cmd_intake = 0;
    cmd_outtake = 0;
    cmd_storage = 0;
    
    jam_trigger_loops = 15; 
    reverse_duration_ms = 250; 
    
    jam_counter = 0;
    is_unjamming = false;
    anti_jam_enabled = true;  
}

void IntakeAntiJam::enable_anti_jam(bool enable) {
    anti_jam_enabled = enable;

    if (!anti_jam_enabled) {
        jam_counter = 0;
    }
}

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

void IntakeAntiJam::stop() {
    set_velocities(0, 0, 0);
    if (!is_unjamming) {
        intakeMotor.brake();
        outtakeMotor.brake();
        storageMotor.brake();
    }
}

void IntakeAntiJam::update() {
    double max_t = std::max({intakeMotor.get_temperature(), 
                             outtakeMotor.get_temperature(), 
                             storageMotor.get_temperature()});
    if (max_t > maxTemp) {
        intakeMotor.move_velocity(0);
        outtakeMotor.move_velocity(0);
        storageMotor.move_velocity(0);
        return;
    }

    if (!anti_jam_enabled || matchloader.is_extended()) {
        jam_counter = 0;
        return; 
    }

    bool is_stuck = false;

    if (std::abs(cmd_intake) > 0) {
        std::int32_t intake_eff = intakeMotor.get_efficiency();
        if (intake_eff >= 7 && intake_eff <= 20) {
            is_stuck = true;
        }
    }

    if (std::abs(cmd_storage) > 0) {
        std::int32_t storage_eff = storageMotor.get_efficiency();
        if (storage_eff >= 7 && storage_eff <= 20) {
            is_stuck = true;
        }
    }

    if (is_stuck) {
        jam_counter++;
    } else {
        jam_counter = 0;
    }

    if (jam_counter > jam_trigger_loops) {
        is_unjamming = true; 
        
        intakeMotor.move_velocity(-cmd_intake);
        outtakeMotor.move_velocity(0);
        if(!intake_lift.is_extended())
        {
            storageMotor.move_velocity(0);
        }
        else {
            storageMotor.move_velocity(-cmd_storage);
        }

        
        pros::delay(reverse_duration_ms); 
        
        intakeMotor.move_velocity(cmd_intake);
        outtakeMotor.move_velocity(cmd_outtake);
        storageMotor.move_velocity(cmd_storage);
        
        jam_counter = 0;
        is_unjamming = false; 
        
        pros::delay(150); 
    }
}