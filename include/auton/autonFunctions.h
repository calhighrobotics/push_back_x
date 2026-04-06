#pragma once

#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "colorSort.h"



void intake(int power = 600);

void outtake(int power = 600);

void score_longgoal(int power = 600, Color allianceColor = Color::RED);

void score_midgoal(int power = 600);

void score_longgoal_auton(int power = 600, Color allianceColor = Color::RED, int time = -1);

void intake_stop(bool hood_state = false);

void matchload_state(bool state);

void relativeMotion(float expected_x, float expected_y, float expected_theta, float distance, int timeout_ms, bool forw = true, float EarlyExit = 0);

void score_midgoal_auton(int power = 600, Color allianceColor = Color::RED, int time = -1);
