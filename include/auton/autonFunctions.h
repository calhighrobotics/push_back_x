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

void intake_stop();

void matchload_state(bool state);

void longgoal_prep();

void reset_odometry();

void match_load_wiggle(int time = 1000);

