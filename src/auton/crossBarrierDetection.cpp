#include "globals.h"
#include <sys/types.h>
#include <cmath>

const double PITCH_CLIMB_THRESHOLD = 9;
const double PITCH_LEVEL_THRESHOLD = 0.5;
const double CROSSING_TIMEOUT = 3000;
const double DRIVE_SPEED = 127;
const double HEADING_KP = 7.0;
const int POST_LANDING_TIME = 200;

void crossBarrier(int times = 2, bool reverse = false, bool fullyDrop = true) {
    double targetHeading = imu.get_heading();
    int dir = reverse ? -1 : 1;

    for(int i = 0; i < times; i++) {
        bool hasClimbed = false;
        bool hasDropped = false;
        
        uint32_t startTime = pros::millis();
        uint32_t previousTime = pros::millis();

        while (true) {

            if (pros::millis() - startTime > CROSSING_TIMEOUT) {
                chassis.tank(0,0);
                leftMotors.brake();
                rightMotors.brake();
                return;
            }

            double currentPitch = imu.get_roll() * dir; 
            double currentHeading = imu.get_heading();

            double error = targetHeading - currentHeading;
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double turnOffset = error * HEADING_KP;

            chassis.tank((DRIVE_SPEED * dir) + turnOffset,
                         (DRIVE_SPEED * dir) - turnOffset);

            // Detect climb
            if (!hasClimbed && currentPitch > PITCH_CLIMB_THRESHOLD) {
                hasClimbed = true;
            }

            // Detect crest
            if (hasClimbed && !hasDropped && currentPitch < PITCH_LEVEL_THRESHOLD) {
                hasDropped = true;
            }

            // --- LANDING LOGIC ---
            if (hasClimbed && hasDropped) {

                // FULLY DROP MODE
                if (fullyDrop && std::abs(currentPitch) < PITCH_LEVEL_THRESHOLD) {

                    uint32_t clearStartTime = pros::millis();
                    while (pros::millis() - clearStartTime < POST_LANDING_TIME) {

                        currentHeading = imu.get_heading();
                        error = targetHeading - currentHeading;
                        while (error > 180) error -= 360;
                        while (error < -180) error += 360;

                        turnOffset = error * HEADING_KP;

                        chassis.tank((DRIVE_SPEED * dir) + turnOffset,
                                     (DRIVE_SPEED * dir) - turnOffset);

                        pros::delay(10);
                    }

                    chassis.tank(0, 0);
                    leftMotors.brake();
                    rightMotors.brake();
                    break;
                }

                // ANGLED PARK MODE
                if (!fullyDrop && currentPitch < -PITCH_LEVEL_THRESHOLD) {

                    chassis.tank(0, 0);

                    // use hold instead of brake to prevent rocking forward
                    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

                    break;
                }
            }

            pros::Task::delay_until(&previousTime, 10);
        }

        if (i < times - 1)
            pros::delay(200);
    }
}
