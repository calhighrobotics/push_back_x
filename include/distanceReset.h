#pragma once

#ifndef DISTANCE_SENSORS_HPP
#define DISTANCE_SENSORS_HPP

#include "globals.h"
#include "lemlib/util.hpp"

constexpr double MM_TO_IN = 0.0393701;
constexpr double FIELD_WIDTH = 3657.6 * MM_TO_IN;
constexpr double FIELD_HEIGHT = 3657.6 * MM_TO_IN;
constexpr double HALF_WIDTH = FIELD_WIDTH / 2.0;
constexpr double HALF_HEIGHT = FIELD_HEIGHT / 2.0;
constexpr double MAX_SENSOR_RANGE = 1800 * MM_TO_IN;
constexpr double MIN_SENSOR_RANGE = 50 * MM_TO_IN;

extern bool controller_screen_avilable;

struct SensorConfig {
    double forward_offset;
    double strafe_offset;
    double mounting_angle;
};

extern const SensorConfig front_sensor_cfg;
extern const SensorConfig left_sensor_cfg;
extern const SensorConfig right_sensor_cfg;
extern const SensorConfig back_sensor_cfg;

struct SensorReadings {
    double dist_mm;
    int object_size;
    int confidence;
};

struct distancePose {
    double x;
    double y;
    bool using_odom_x;
    bool using_odom_y;
};

// Calculates global position based on sensor readings and robot heading
distancePose calculateGlobalPosition(
    const SensorReadings& front_data,
    const SensorReadings& left_data,
    const SensorReadings& right_data,
    const SensorReadings& back_data,
    double heading_deg
);

// Resets distance-based global pose using all sensors
distancePose distanceReset(bool setPose = true);

// Resets distance-based global pose using selected sensors
distancePose distanceReset(bool left_use, bool right_use, bool front_use, bool back_use, bool setPose);

#endif // DISTANCE_SENSORS_HPP
