#pragma once

#include "globals.h"

// Barrier crossing parameters
extern const double PITCH_CLIMB_THRESHOLD;   // Degrees robot tilts up when climbing
extern const double PITCH_LEVEL_THRESHOLD;   // Degrees considered "flat"
extern const double CROSSING_TIMEOUT;        // MS to abort if stuck
extern const double DRIVE_SPEED;             // Motor voltage (0-127)
extern const double HEADING_KP;               // Heading correction gain

// Cross-field barrier routine
void crossBarrier();
