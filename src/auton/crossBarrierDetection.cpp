#include "globals.h"
#include <sys/types.h>
#include <cmath>

const double PITCH_CLIMB_THRESHOLD = 9;
const double PITCH_LEVEL_THRESHOLD = 0.5;
const double CROSSING_TIMEOUT = 3000;
const double DRIVE_SPEED = 100;
const double HEADING_KP = 2.0;
const int POST_LANDING_TIME = 200;

void crossBarrier(int times = 2, bool reverse = false) {
    double targetHeading = imu.get_heading();
    int dir = reverse ? -1 : 1;

    for(int i = 0; i < times; i++) {
        bool hasClimbed = false;
        bool hasDropped = false;
        
        uint32_t startTime = pros::millis();
        uint32_t previousTime = pros::millis();

        std::cout << "Status: STARTING CROSS (Reverse: " << reverse << ")" << std::endl;

        while (true) {
            if (pros::millis() - startTime > CROSSING_TIMEOUT) {
                chassis.tank(-127 * dir, -127 * dir);
                pros::delay(300);
                chassis.tank(0, 0);
                std::cout << "Status: TIMEOUT - ABORTING" << std::endl;
                break; 
            }

            double currentPitch = imu.get_roll() * dir; 
            double currentHeading = imu.get_heading();
            
            double error = targetHeading - currentHeading;
            
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double turnOffset = error * HEADING_KP;

            if (!reverse) {
                chassis.tank(DRIVE_SPEED + turnOffset, DRIVE_SPEED - turnOffset);
            } else {
                chassis.tank((DRIVE_SPEED * dir) + turnOffset, (DRIVE_SPEED * dir) - turnOffset);
            }

            if (reverse ? : !hasClimbed && currentPitch > PITCH_CLIMB_THRESHOLD) {
                hasClimbed = true;
                std::cout << "Status: CLIMBING" << std::endl;
            }

            if (hasClimbed && !hasDropped && currentPitch < PITCH_LEVEL_THRESHOLD) {
                hasDropped = true;
                if (!reverse) {
                    matchload.extend();
                    std::cout << "Status: EXTENDING MATCHLOADER" << std::endl;
                }
                std::cout << "Status: DROPPING" << std::endl;
            }

            if (hasClimbed && hasDropped && std::abs(currentPitch) < PITCH_LEVEL_THRESHOLD) {
                std::cout << "Status: LANDED - CLEARING BARRIER" << std::endl;
                
                uint32_t clearStartTime = pros::millis();
                while (pros::millis() - clearStartTime < POST_LANDING_TIME) {
                    currentHeading = imu.get_heading();
                    error = targetHeading - currentHeading;
                    while (error > 180) error -= 360;
                    while (error < -180) error += 360;
                    
                    turnOffset = error * HEADING_KP;
                    
                    if (!reverse) {
                         chassis.tank(DRIVE_SPEED + turnOffset, DRIVE_SPEED - turnOffset);
                    } else {
                         chassis.tank((DRIVE_SPEED * dir) + turnOffset, (DRIVE_SPEED * dir) - turnOffset);
                    }
                    
                    pros::delay(10);
                }

                if (!reverse) {
                    matchload.retract();
                    std::cout << "Status: RETRACTING MATCHLOADER" << std::endl;
                }

                chassis.tank(0, 0);
                pros::delay(50);
                leftMotors.brake();
                rightMotors.brake();
                
                std::cout << "Status: COMPLETE" << std::endl;
                break; 
            }

            pros::Task::delay_until(&previousTime, 10);
        }
        if (i < times - 1) pros::delay(200);
    }
}
