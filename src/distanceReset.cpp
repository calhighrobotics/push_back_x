#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "globals.h"


const double FIELD_WIDTH_MM = 3657.6;
const double FIELD_HEIGHT_MM = 3657.6;


const double OFFSET_TO_LEFT_SENSOR = 150.0;
const double OFFSET_TO_RIGHT_SENSOR = 155.0;
const double OFFSET_TO_BACK_SENSOR = 170.0;
const double OFFSET_TO_FRONT_SENSOR = 165.0; 


std::pair<double, double> distanceReset()
{
    double dist_left = left.get_distance();
    double dist_right = right.get();
    double dist_back = back.get();
    double dist_front = front.get();
    double heading_rad = chassis.getPose(true).theta;
    double cos_h = std::cos(heading_rad);
    double sin_h = std::sin(heading_rad);

    // Determine nearest walls
    bool near_left, near_back;
    if ((dist_left + OFFSET_TO_LEFT_SENSOR) < (dist_right + OFFSET_TO_RIGHT_SENSOR)) {
        near_left = true;
    } else {
        near_left = false;
    }

    if ((dist_back + OFFSET_TO_BACK_SENSOR) < (dist_front + OFFSET_TO_FRONT_SENSOR)) {
        near_back = true;
    } else {
        near_back = false;
    }

    // Reference corner position in global frame
    double ref_x, ref_y;
    if (near_left) {
        ref_x = 0.0;
    } else {
        ref_x = FIELD_WIDTH_MM;
    }

    if (near_back) {
        ref_y = 0.0;
    } else {
        ref_y = FIELD_HEIGHT_MM;
    }

    // Local distances from nearest walls
    double x_local, y_local;
    if (near_left) {
        x_local = dist_left + OFFSET_TO_LEFT_SENSOR;
    } else {
        x_local = dist_right + OFFSET_TO_RIGHT_SENSOR;
    }

    if (near_back) {
        y_local = dist_back + OFFSET_TO_BACK_SENSOR;
    } else {
        y_local = dist_front + OFFSET_TO_FRONT_SENSOR;
    }

    // Rotate robot-local offset vector into global frame
    double global_x = ref_x + x_local * cos_h - y_local * sin_h;
    double global_y = ref_y + x_local * sin_h + y_local * cos_h;

    return {global_x, global_y};
}
