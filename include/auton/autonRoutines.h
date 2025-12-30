#pragma once

#include "main.h"

extern const int longgoal_delay = 2000;
extern const int midgoal_delay = 3000;
extern const int matchload_delay = 1500;
extern const int mid_triball_delay = 500;
extern const int longgoal_offset = 12;
extern const int midgoal_offset = 12;
extern const int matchload_offset = 12.9;
extern const int triball_delay = 500;
extern const int dual_ball_delay = 500;

// Function declarations
void precompute_auton_paths();
void right_auton();
void left_auton();
void carry_auton();
void elim_auton(); 
void awp_auton();
void skills_auton();