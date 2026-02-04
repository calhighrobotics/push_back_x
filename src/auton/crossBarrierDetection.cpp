#include "globals.h"
#include <sys/types.h>
#include <cmath> // Required for std::abs

// --- Tuning Constants ---
const double PITCH_CLIMB_THRESHOLD = 9; // Degrees robot tilts up when climbing
const double PITCH_LEVEL_THRESHOLD = 0.4;  // Degrees considered "flat"
const double CROSSING_TIMEOUT = 3000;      // MS to abort if stuck (Safety)
const double DRIVE_SPEED =  100;            // Motor voltage (0-127) for power
const double HEADING_KP = 2.0;             // Strength of heading correction
const int POST_LANDING_TIME = 300;         // MS to drive AFTER landing to clear rear wheels

void crossBarrier(int times = 2) {
    // 1. Lock in the target heading before we start movin
    double targetHeading = imu.get_heading();
    
    // State flags
    bool hasClimbed = false;
    bool hasDropped = false;
    
    uint32_t startTime = pros::millis();
    
    // Use this to track the loop cycle time for delay_until
    uint32_t previousTime = pros::millis();
    for(int i = 0; i < times; i++)
    {
        while (true) {
            // --- Safety Timeout ---
            if (pros::millis() - startTime > CROSSING_TIMEOUT) {
                // If we take too long, back up slightly and stop
                chassis.tank(-127, -127);
                pros::delay(300);
                chassis.tank(0, 0);
                std::cout << "Status: TIMEOUT - ABORTING" << std::endl;
                break; 
            }

            // --- Sensor Readings ---
            double currentPitch = imu.get_roll(); 
            double currentHeading = imu.get_heading();
            
            // --- Heading Correction (P-Controller) ---
            double error = targetHeading - currentHeading;
            
            // Optimize error to be within -180 to 180 degrees
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double turnOffset = error * HEADING_KP;

            // Apply power
            chassis.tank(DRIVE_SPEED + turnOffset, DRIVE_SPEED - turnOffset);

            // --- State Machine ---
            
            // Phase 1: Detect the climb (Front goes up)
            if (!hasClimbed && currentPitch > PITCH_CLIMB_THRESHOLD) {
                hasClimbed = true;
                std::cout << "Status: CLIMBING" << std::endl;
            }

            // Phase 2: Detect the drop (Robot starts leveling out or tilting down)
            // We check if we have climbed first to avoid false positives on flat ground
            if (hasClimbed && !hasDropped && currentPitch < PITCH_LEVEL_THRESHOLD) {
                hasDropped = true;
                std::cout << "Status: DROPPING" << std::endl;
            }

            // Phase 3: Landed & Clearing
            // If we have climbed, dropped, and are now flat...
            if (hasClimbed && hasDropped && std::abs(currentPitch) < PITCH_LEVEL_THRESHOLD) {
                std::cout << "Status: LANDED - CLEARING BARRIER" << std::endl;
                
                // CONTINUE DRIVING for a set time to clear back wheels
                // We use the same heading correction during this phase
                uint32_t clearStartTime = pros::millis();
                while (pros::millis() - clearStartTime < POST_LANDING_TIME) {
                    currentHeading = imu.get_heading();
                    error = targetHeading - currentHeading;
                    // Normalize error
                    while (error > 180) error -= 360;
                    while (error < -180) error += 360;
                    
                    turnOffset = error * HEADING_KP;
                    chassis.tank(DRIVE_SPEED + turnOffset, DRIVE_SPEED - turnOffset);
                    
                    pros::delay(10);
                }

                // Hard stop to prevent drift
                chassis.tank(0, 0);
                pros::delay(50); // Small settle time
                leftMotors.brake();
                rightMotors.brake(); // Optional: Hold position
                
                std::cout << "Status: COMPLETE" << std::endl;
                break; 
            }

            // Loop timing maintenance
            pros::Task::delay_until(&previousTime, 10);
        }
    }
}