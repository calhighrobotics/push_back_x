#pragma once

extern const double PITCH_CLIMB_THRESHOLD;
extern const double PITCH_LEVEL_THRESHOLD;
extern const double CROSSING_TIMEOUT;
extern const double DRIVE_SPEED;
extern const double HEADING_KP;

void crossBarrier(int times = 2, bool reverse = false, bool fullyDrop = true);
