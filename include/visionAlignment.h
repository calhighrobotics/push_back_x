#pragma once

#include "globals.h"
#include "pros/colors.h"

bool followMultipleObjectsPID(std::vector<int> signatures, float target_distance_in, int timeout_ms = 3000);
