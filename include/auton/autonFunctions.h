#pragma once

#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"



void intake(int power = 12000);

void outtake(int power = 12000);

void score_bottomgoal(int power = 12000);

void score_longgoal(int power = 12000);

void score_midgoal(int power = 12000);

void intake_to_basket();

void intake_stop();

void resting_state();

void matchload_state(bool state);

void longgoal_prep();

void reset_odometry();

void matchload_wiggle(int time = 1000, int speed = 100);

void MCL_reset(bool x = true, bool y = true);

void fusion_loop_fn(void* ignore);

void enable_fused_odometry(bool state);

void relativeMotion(float expected_x, float expected_y, float expected_theta, float distance, int timeout_ms, bool forw);

void matchload_counter(int balls, int time_ms);
