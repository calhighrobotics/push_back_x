#include "globals.h"
#include <sys/types.h>


const double PITCH_CLIMB_THRESHOLD = 15.0; // Degrees robot tilts up when climbing
const double PITCH_LEVEL_THRESHOLD = 5.0;  // Degrees considered "flat"
const double CROSSING_TIMEOUT = 3000;      // MS to abort if stuck (Safety)
const double DRIVE_SPEED =  90;            // Motor voltage (0-127) for power
const double HEADING_KP = 2.0;             // Strength of heading correction

void crossBarrier() {
    double targetHeading = imu.get_heading();
    
    bool hasClimbed = false;
    bool hasDropped = false;
    
    uint32_t startTime = pros::millis();
    while (true) {
        u_int32_t currentTime = pros::millis();
        if (pros::millis() - startTime > CROSSING_TIMEOUT) {
            chassis.tank(-127, -127);
            pros::delay(300);
            chassis.tank(0, 0);
            break; 
        }

        double currentPitch = imu.get_pitch(); 
        double currentHeading = imu.get_heading();
        double error = targetHeading - currentHeading;
        
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double turnOffset = error * HEADING_KP;

        chassis.tank(DRIVE_SPEED + turnOffset, DRIVE_SPEED - turnOffset);
        if (!hasClimbed && currentPitch > PITCH_CLIMB_THRESHOLD) {
            hasClimbed = true;
            std::cout << "Status: CLIMBING" << std::endl;
        }

        if (hasClimbed && !hasDropped && currentPitch < PITCH_LEVEL_THRESHOLD) {
            hasDropped = true;
            std::cout << "Status: DROPPING" << std::endl;
        }

        if (hasClimbed && hasDropped && std::abs(currentPitch) < PITCH_LEVEL_THRESHOLD) {
            std::cout << "Status: LANDED" << std::endl;
            
            pros::delay(200); 
            chassis.tank(0, 0);
            break; 
        }
        pros::Task::delay_until(&currentTime, 10);
    }
}