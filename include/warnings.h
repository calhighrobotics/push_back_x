/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

// Spawns a background task that monitors motor temperatures
// and alerts the controller if overheating is detected.
void temp_warning();

// Spawns a background task that monitors all motors
// and alerts the controller if a motor becomes disconnected.
void motor_disconnect_warning();

// Spawns a background task that monitors distance sensors
// and alerts the controller if a sensor becomes disconnected.
void distance_sensor_disconnect_warning();
