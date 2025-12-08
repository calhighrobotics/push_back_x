#include "globals.h"
#include "lemlib/util.hpp"
#include <numeric>

const double MM_TO_IN = 0.0393701;
const double FIELD_WIDTH = 3657.6 * MM_TO_IN;
const double FIELD_HEIGHT = 3657.6 * MM_TO_IN;
const double HALF_WIDTH = FIELD_WIDTH / 2.0;
const double HALF_HEIGHT = FIELD_HEIGHT / 2.0;
const double MAX_SENSOR_RANGE = 1800 * MM_TO_IN;
const double MIN_SENSOR_RANGE = 50 * MM_TO_IN;

bool controller_screen_avilable;

struct SensorConfig {
    double forward_offset;
    double strafe_offset;
    double mounting_angle;
};

const SensorConfig front_sensor_cfg = {9, 0.25, 0};   
const SensorConfig left_sensor_cfg  = {-1.25, -5.5, 90};   
const SensorConfig right_sensor_cfg = {-1.25, 5.5, -90};  
const SensorConfig back_sensor_cfg  = {-7.75, -0.25, 180}; 

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

distancePose calculateGlobalPosition(
    const SensorReadings& front_data,
    const SensorReadings& left_data,
    const SensorReadings& right_data,
    const SensorReadings& back_data,
    double heading_deg)
{
    lemlib::Pose current_pose = chassis.getPose();
    double est_x = current_pose.x;
    double est_y = current_pose.y;

    bool using_odom_x = true;
    bool using_odom_y = true;

    struct SensorData { SensorConfig cfg; double dist_in; int confidence;};
    const SensorData sensors[] = {
        {front_sensor_cfg, front_data.dist_mm * MM_TO_IN, front.get_confidence()},
        {left_sensor_cfg,  left_data.dist_mm * MM_TO_IN, left.get_confidence()},
        {right_sensor_cfg, right_data.dist_mm * MM_TO_IN, right.get_confidence()},
        {back_sensor_cfg,  back_data.dist_mm * MM_TO_IN, back.get_confidence()}
    };
    auto is_valid = [&](int i) {
        return (sensors[i].dist_in < MAX_SENSOR_RANGE && 
                sensors[i].dist_in >= MIN_SENSOR_RANGE &&
                sensors[i].confidence > 20);
    };

    double norm_heading = std::fmod(heading_deg, 360.0);
    if (norm_heading < 0) norm_heading += 360.0;
    const double TOLERANCE = 40.0; 
    
    std::vector<double> x_cands;
    std::vector<double> y_cands;

    auto get_global_offsets = [&](const SensorConfig& cfg, double heading_rad) {
        double cos_h = std::cos(heading_rad);
        double sin_h = std::sin(heading_rad);
        double global_offset_x = (cfg.forward_offset * sin_h) + (cfg.strafe_offset * cos_h);
        double global_offset_y = (cfg.forward_offset * cos_h) - (cfg.strafe_offset * sin_h);
        
        return std::make_pair(global_offset_x, global_offset_y);
    };

    if (norm_heading <= TOLERANCE || norm_heading >= 360.0 - TOLERANCE) {
        double angle_off_rad = (norm_heading <= TOLERANCE) ? 
            lemlib::degToRad(norm_heading) : lemlib::degToRad(norm_heading - 360.0);
        double heading_rad = angle_off_rad;

        double perp_dist_0 = sensors[0].dist_in * std::cos(angle_off_rad);
        double perp_dist_3 = sensors[3].dist_in * std::cos(angle_off_rad);
        double perp_dist_1 = sensors[1].dist_in * std::cos(angle_off_rad);
        double perp_dist_2 = sensors[2].dist_in * std::cos(angle_off_rad);

        auto offset_0 = get_global_offsets(sensors[0].cfg, heading_rad);
        auto offset_1 = get_global_offsets(sensors[1].cfg, heading_rad);
        auto offset_2 = get_global_offsets(sensors[2].cfg, heading_rad);
        auto offset_3 = get_global_offsets(sensors[3].cfg, heading_rad);

        if (is_valid(0)) { y_cands.push_back(HALF_HEIGHT - perp_dist_0 - offset_0.second); }
        if (is_valid(3)) { y_cands.push_back(-HALF_HEIGHT + perp_dist_3 - offset_3.second); }
        if (is_valid(1)) { x_cands.push_back(-HALF_WIDTH + perp_dist_1 - offset_1.first); }
        if (is_valid(2)) { x_cands.push_back(HALF_WIDTH - perp_dist_2 - offset_2.first); }
    }
    else if (std::fabs(norm_heading - 180.0) <= TOLERANCE) {
        double angle_off_rad = lemlib::degToRad(norm_heading - 180.0);
        double heading_rad = lemlib::degToRad(norm_heading);

        double perp_dist_0 = sensors[0].dist_in * std::cos(angle_off_rad);
        double perp_dist_3 = sensors[3].dist_in * std::cos(angle_off_rad);
        double perp_dist_1 = sensors[1].dist_in * std::cos(angle_off_rad);
        double perp_dist_2 = sensors[2].dist_in * std::cos(angle_off_rad);

        auto offset_0 = get_global_offsets(sensors[0].cfg, heading_rad);
        auto offset_1 = get_global_offsets(sensors[1].cfg, heading_rad);
        auto offset_2 = get_global_offsets(sensors[2].cfg, heading_rad);
        auto offset_3 = get_global_offsets(sensors[3].cfg, heading_rad);

        if (is_valid(0)) { y_cands.push_back(-HALF_HEIGHT + perp_dist_0 - offset_0.second); }
        if (is_valid(3)) { y_cands.push_back(HALF_HEIGHT - perp_dist_3 - offset_3.second); }
        if (is_valid(1)) { x_cands.push_back(HALF_WIDTH - perp_dist_1 - offset_1.first); }
        if (is_valid(2)) { x_cands.push_back(-HALF_WIDTH + perp_dist_2 - offset_2.first); }
    }
    else if (std::fabs(norm_heading - 90.0) <= TOLERANCE) {
        double angle_off_rad = lemlib::degToRad(norm_heading - 90.0);
        double heading_rad = lemlib::degToRad(norm_heading);
        
        double perp_dist_0 = sensors[0].dist_in * std::cos(angle_off_rad);
        double perp_dist_3 = sensors[3].dist_in * std::cos(angle_off_rad);
        double perp_dist_1 = sensors[1].dist_in * std::cos(angle_off_rad);
        double perp_dist_2 = sensors[2].dist_in * std::cos(angle_off_rad);

        auto offset_0 = get_global_offsets(sensors[0].cfg, heading_rad);
        auto offset_1 = get_global_offsets(sensors[1].cfg, heading_rad);
        auto offset_2 = get_global_offsets(sensors[2].cfg, heading_rad);
        auto offset_3 = get_global_offsets(sensors[3].cfg, heading_rad);

        if (is_valid(0)) { x_cands.push_back(HALF_WIDTH - perp_dist_0 - offset_0.first); }
        if (is_valid(3)) { x_cands.push_back(-HALF_WIDTH + perp_dist_3 - offset_3.first); }
        if (is_valid(1)) { y_cands.push_back(HALF_HEIGHT - perp_dist_1 - offset_1.second); }
        if (is_valid(2)) { y_cands.push_back(-HALF_HEIGHT + perp_dist_2 - offset_2.second); }
    }
    else if (std::fabs(norm_heading - 270.0) <= TOLERANCE) {
        double angle_off_rad = lemlib::degToRad(norm_heading - 270.0);
        double heading_rad = lemlib::degToRad(norm_heading);

        double perp_dist_0 = sensors[0].dist_in * std::cos(angle_off_rad);
        double perp_dist_3 = sensors[3].dist_in * std::cos(angle_off_rad);
        double perp_dist_1 = sensors[1].dist_in * std::cos(angle_off_rad);
        double perp_dist_2 = sensors[2].dist_in * std::cos(angle_off_rad);
        
        auto offset_0 = get_global_offsets(sensors[0].cfg, heading_rad);
        auto offset_1 = get_global_offsets(sensors[1].cfg, heading_rad);
        auto offset_2 = get_global_offsets(sensors[2].cfg, heading_rad);
        auto offset_3 = get_global_offsets(sensors[3].cfg, heading_rad);
        
        if (is_valid(0)) { x_cands.push_back(-HALF_WIDTH + perp_dist_0 - offset_0.first); }
        if (is_valid(3)) { x_cands.push_back(HALF_WIDTH - perp_dist_3 - offset_3.first); }
        if (is_valid(1)) { y_cands.push_back(-HALF_HEIGHT + perp_dist_1 - offset_1.second); }
        if (is_valid(2)) { y_cands.push_back(HALF_HEIGHT - perp_dist_2 - offset_2.second); }
    }

    if (!x_cands.empty()) {
        est_x = std::accumulate(x_cands.begin(), x_cands.end(), 0.0) / x_cands.size();
        using_odom_x = false;
    }

    if (!y_cands.empty()) {
        est_y = std::accumulate(y_cands.begin(), y_cands.end(), 0.0) / y_cands.size();
        using_odom_y = false;
    }

    distancePose pose;
    pose.x = est_x;
    pose.y = est_y;
    pose.using_odom_x = using_odom_x;
    pose.using_odom_y = using_odom_y;
    return pose;
}

distancePose distanceReset() {
    double heading_deg = chassis.getPose().theta;

    SensorReadings front_data = {(double)front.get_distance(), front.get_object_size(), front.get_confidence()};
    SensorReadings left_data  = {(double)left.get_distance(),  left.get_object_size(),  left.get_confidence()};
    SensorReadings right_data = {(double)right.get_distance(), right.get_object_size(), right.get_confidence()};
    SensorReadings back_data  = {(double)back.get_distance(),  back.get_object_size(),  back.get_confidence()};

    return calculateGlobalPosition(front_data, left_data, right_data, back_data, heading_deg);
}

// Overloaded function
distancePose distanceReset(bool left_use, bool right_use, bool front_use, bool back_use) {
    double heading_deg = chassis.getPose().theta; 

    const int invalid_dist_mm = 10000; 
    const int invalid_confidence = 0; 

    SensorReadings front_data = front_use
        ? SensorReadings{(double)front.get_distance(), front.get_object_size(), front.get_confidence()}
        : SensorReadings{invalid_dist_mm, 0, invalid_confidence};
    
    SensorReadings left_data = left_use
        ? SensorReadings{(double)left.get_distance(), left.get_object_size(), left.get_confidence()}
        : SensorReadings{invalid_dist_mm, 0, invalid_confidence};

    SensorReadings right_data = right_use
        ? SensorReadings{(double)right.get_distance(), right.get_object_size(), right.get_confidence()}
        : SensorReadings{invalid_dist_mm, 0, invalid_confidence};

    SensorReadings back_data = back_use
        ? SensorReadings{(double)back.get_distance() ,back.get_object_size(), back.get_confidence()}
        : SensorReadings{invalid_dist_mm, 0, invalid_confidence};

    return calculateGlobalPosition(front_data, left_data, right_data, back_data, heading_deg);
}
