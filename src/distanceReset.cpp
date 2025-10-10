#include "main.h"
#include "globals.h"
#include "lemlib/api.hpp"
#include <cmath>
#include <vector>
#include <utility>
/*
const double FIELD_WIDTH_MM = 3657.6;
const double FIELD_HEIGHT_MM = 3657.6;

const double OFFSET_TO_LEFT_SENSOR = 150.0;
const double OFFSET_TO_RIGHT_SENSOR = 155.0;
const double OFFSET_TO_BACK_SENSOR = 170.0;
const double OFFSET_TO_FRONT_SENSOR = 165.0;

const double MAX_SENSOR_RANGE = 3000.0;

std::pair<double, double> calculateGlobalPosition(
    double dist_left, double dist_right,
    double dist_back, double dist_front,
    double heading_deg, lemlib::Pose current_pose)
{
    double heading_rad = lemlib::degToRad(heading_deg);
    double cos_h = std::cos(heading_rad);
    double sin_h = std::sin(heading_rad);

    struct Sensor {
        double sx, sy, angle_deg, dist;
    } sensors[] = {
        {-OFFSET_TO_LEFT_SENSOR, 0, -90, dist_left},
        {OFFSET_TO_RIGHT_SENSOR, 0, 90, dist_right},
        {0, -OFFSET_TO_BACK_SENSOR, 180, dist_back},
        {0, OFFSET_TO_FRONT_SENSOR, 0, dist_front}
    };

    std::vector<std::pair<char, double>> constraints;

    for (const auto& s : sensors) {
        if (s.dist > MAX_SENSOR_RANGE) continue;

        double sensor_angle = heading_rad + lemlib::degToRad(s.angle_deg);
        double dir_x = std::cos(sensor_angle);
        double dir_y = std::sin(sensor_angle);

        double offset_x = s.sx * cos_h - s.sy * sin_h;
        double offset_y = s.sx * sin_h + s.sy * cos_h;

        bool x_constraint_plausible = false;
        bool y_constraint_plausible = false;

        double wall_x;
        if (dir_x < 0) {
            wall_x = 0.0;
        } else {
            wall_x = FIELD_WIDTH_MM;
        }
        double robot_x_from_x_wall = wall_x - s.dist * dir_x - offset_x;
        double sensor_y_at_x_wall = (current_pose.y + offset_y) + (wall_x - (current_pose.x + offset_x)) * (dir_y / dir_x);
        if (sensor_y_at_x_wall >= 0 && sensor_y_at_x_wall <= FIELD_HEIGHT_MM) {
            x_constraint_plausible = true;
        }

        double wall_y;
        if (dir_y < 0) {
            wall_y = 0.0;
        } else {
            wall_y = FIELD_HEIGHT_MM;
        }
        double robot_y_from_y_wall = wall_y - s.dist * dir_y - offset_y;
        double sensor_x_at_y_wall = (current_pose.x + offset_x) + (wall_y - (current_pose.y + offset_y)) * (dir_x / dir_y);
        if (sensor_x_at_y_wall >= 0 && sensor_x_at_y_wall <= FIELD_WIDTH_MM) {
            y_constraint_plausible = true;
        }

        if (x_constraint_plausible && !y_constraint_plausible) {
            constraints.push_back({'x', robot_x_from_x_wall});
        } else if (!x_constraint_plausible && y_constraint_plausible) {
            constraints.push_back({'y', robot_y_from_y_wall});
        } else {
            if (std::fabs(dir_x) > std::fabs(dir_y)) {
                constraints.push_back({'x', robot_x_from_x_wall});
            } else {
                constraints.push_back({'y', robot_y_from_y_wall});
            }
        }
    }

    double sum_x = 0, sum_y = 0;
    int count_x = 0, count_y = 0;
    for (const auto& c : constraints) {
        if (c.first == 'x') { sum_x += c.second; count_x++; }
        else { sum_y += c.second; count_y++; }
    }
    
    double est_x;
    if (count_x > 0) {
        est_x = sum_x / count_x;
    } else {
        est_x = current_pose.x;
    }

    double est_y;
    if (count_y > 0) {
        est_y = sum_y / count_y;
    } else {
        est_y = current_pose.y;
    }

    return {est_x, est_y};
}

void distanceReset() {
    lemlib::Pose current_pose = chassis.getPose();
    double heading_deg = lemlib::radToDeg(current_pose.theta);

    double dist_left = left.get_distance();
    double dist_right = right.get_distance();
    double dist_front = front.get_distance();
    double dist_back = back.get_distance();

    std::pair<double, double> estimated_position = calculateGlobalPosition(
        dist_left, dist_right, dist_back, dist_front, heading_deg, current_pose
    );

    chassis.setPose(estimated_position.first, estimated_position.second, current_pose.theta);
}
*/