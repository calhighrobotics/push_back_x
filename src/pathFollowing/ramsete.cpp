#include "ramsete.h" 
#include "lemlib/util.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm> 

RamsetePathFollower::Vector2::Vector2(float x, float y) : x(x), y(y) {}

std::string RamsetePathFollower::Vector2::latex() const {
    std::ostringstream oss;
    oss << "\\left(" << std::fixed << this->x << "," << std::fixed << this->y << "\\right)";
    return oss.str();
}

double RamsetePathFollower::sinc(double x) {
    if (std::abs(x) < 1e-6) {
        return 1.0 - (x * x) / 6.0 + (x * x * x * x) / 120.0;
    }
    return std::sin(x) / x;
}

double RamsetePathFollower::angleError(double robotAngle, double targetAngle) {
    constexpr double TWO_PI = 2.0 * M_PI;
    double diff = std::fmod(targetAngle - robotAngle, TWO_PI);
    if (diff < -M_PI) diff += TWO_PI;
    else if (diff >= M_PI) diff -= TWO_PI;
    return diff;
}

RamsetePathFollower::RamsetePathFollower(const VelocityControllerConfig& config, float b_, float zeta_)
    : controller(
          config.kV, config.KA_straight, config.KA_turn,
          config.KS_straight, config.KS_turn,
          config.KP_straight, config.KI_straight,
          99999.0, TRACK_WIDTH * INCH_TO_METER
      ), b(b_), zeta(zeta_) {}

void RamsetePathFollower::followPath(const std::string& path_name, const ramseteConfig& r_config) {
    if (is_running) {
        cancel();
        waitUntilDone();
    }

    is_running = true;
    cancel_request = false;
    distance_traveled = 0.0f;

    TaskParams* params = new TaskParams{this, path_name, r_config};

    task = new pros::Task(task_trampoline, params, "RamseteTask");
    
    pros::delay(10);
}

void RamsetePathFollower::task_trampoline(void* params) {
    TaskParams* p = static_cast<TaskParams*>(params);
    p->instance->followPathImpl(p->path_name, p->config);
    delete p;
}

void RamsetePathFollower::waitUntilDone() {
    while (is_running) {
        pros::delay(10);
    }
}

void RamsetePathFollower::waitUntil(float dist_inches) {
    while (is_running && distance_traveled < dist_inches) {
        pros::delay(10);
    }
}

void RamsetePathFollower::cancel() {
    cancel_request = true;
}

bool RamsetePathFollower::isRunning() {
    return is_running;
}

void RamsetePathFollower::followPathImpl(const std::string& path_name, const ramseteConfig& r_config) {
    std::vector<State> trajectory;
    
    if(r_config.path_index >= 0 && (size_t)(r_config.path_index) < precomputed_paths.size()) {
        if (!precomputed_paths.at(r_config.path_index).empty()) {
            trajectory = precomputed_paths.at(r_config.path_index);
        }
    }
    if (trajectory.empty()) {
        trajectory = prepare_trajectory(path_name);
    }
    if (trajectory.empty()) {
        is_running = false;
        return;
    }

    if(r_config.test) {
        chassis.setPose(trajectory[0].x / INCH_TO_METER, trajectory[0].y / INCH_TO_METER, M_PI_2 - trajectory[0].heading, true);
    } else if(r_config.turnFirst) {
        double targetH = lemlib::radToDeg(M_PI_2 - trajectory[0].heading);
        chassis.turnToHeading(r_config.backwards ? targetH + 180 : targetH, 1000);
    }

    std::vector<std::string> logs;
    int trajectory_size = trajectory.size();
    
    const float min_k_sq_vel = 0.5f;
    const int path_dt_ms = 10;
    
    const double success_tolerance_inches = 0.5;
    const int max_settle_time_ms = 1500;
    
    lemlib::Pose start_pose = chassis.getPose();
    uint32_t global_start_time = pros::millis();
    
    bool is_settling = false;
    uint32_t settle_start_time = 0;

    while (!cancel_request) {
        uint32_t now = pros::millis();
        
        float t_elapsed_sec = (now - global_start_time) / 1000.0f;
        float exact_index = t_elapsed_sec / (path_dt_ms / 1000.0f);
        int idx = static_cast<int>(exact_index);

        State target_state;
        float current_b = r_config.b; 

        if (idx < trajectory_size - 1) {
            float alpha = exact_index - idx;
            const State& s0 = trajectory[idx];
            const State& s1 = trajectory[idx+1];

            target_state.x = s0.x + alpha * (s1.x - s0.x);
            target_state.y = s0.y + alpha * (s1.y - s0.y);
            target_state.linear_vel = s0.linear_vel + alpha * (s1.linear_vel - s0.linear_vel);
            target_state.angular_vel = s0.angular_vel + alpha * (s1.angular_vel - s0.angular_vel);
            
            double dh = s1.heading - s0.heading;
            while (dh > M_PI) dh -= 2 * M_PI;
            while (dh < -M_PI) dh += 2 * M_PI;
            target_state.heading = s0.heading + alpha * dh; 
        } 
        else {
            if (!is_settling) {
                is_settling = true;
                settle_start_time = now;
            }

            lemlib::Pose p = chassis.getPose();
            double p_x_m = p.x * INCH_TO_METER;
            double p_y_m = p.y * INCH_TO_METER;

            double dist_to_end_m = std::hypot(
                trajectory.back().x - p_x_m,
                trajectory.back().y - p_y_m
            );
            
            double dist_to_end_in = dist_to_end_m / INCH_TO_METER;

            if (dist_to_end_in < success_tolerance_inches) {
                break;
            }
            if (now - settle_start_time > max_settle_time_ms) {
                break;
            }

            target_state = trajectory.back();
            target_state.linear_vel = 0;
            target_state.angular_vel = 0;

            current_b = r_config.b * 4.0; 
        }

        lemlib::Pose current_pose = chassis.getPose(true);
        distance_traveled = start_pose.distance(current_pose);

        current_pose.x *= INCH_TO_METER;
        current_pose.y *= INCH_TO_METER;
        current_pose.theta = M_PI_2 - current_pose.theta; 

        double targetHeadingAdjusted = target_state.heading + (r_config.backwards ? M_PI : 0);
        double errorTheta = angleError(current_pose.theta, targetHeadingAdjusted);

        Eigen::Vector3d global_error;
        global_error << target_state.x - current_pose.x, target_state.y - current_pose.y, errorTheta;

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix <<  std::cos(current_pose.theta), std::sin(current_pose.theta), 0, 
                           -std::sin(current_pose.theta), std::cos(current_pose.theta), 0, 
                            0, 0, 1;

        Eigen::Vector3d local_error = rotation_matrix * global_error;
        float e_x = local_error(0);
        float e_y = local_error(1);
        float e_t = local_error(2);

        float vd = target_state.linear_vel * (r_config.backwards ? -1.0 : 1.0);
        float wd = target_state.angular_vel;

        float k = 2.0 * r_config.zeta * std::sqrt(wd * wd + current_b * std::max((float)(vd * vd), min_k_sq_vel));

        float v_desired_ramsete = vd * std::cos(e_t) + k * e_x;
        float w_desired_ramsete = wd + k * e_t + (current_b * vd * sinc(e_t) * e_y);

        float left_actual_mps = leftMotors.get_actual_velocity() * rpm_to_mps_factor;
        float right_actual_mps = rightMotors.get_actual_velocity() * rpm_to_mps_factor;

        DrivetrainVoltages output_voltages = controller.update(
            v_desired_ramsete, 
            w_desired_ramsete, 
            left_actual_mps, 
            right_actual_mps
        );
    
        rightMotors.move_voltage(output_voltages.rightVoltage * 1000.0);
        leftMotors.move_voltage(output_voltages.leftVoltage * 1000.0);

        if(r_config.log) {
            std::ostringstream ss;
            ss << Vector2(current_pose.x, current_pose.y).latex() << ",";
            logs.push_back(ss.str());
        }

        pros::delay(10);
    }

    rightMotors.brake();
    leftMotors.brake();
    
    if(!r_config.test && r_config.end_correction && !cancel_request) {
        chassis.moveToPose(
            trajectory.back().x / INCH_TO_METER, 
            trajectory.back().y / INCH_TO_METER, 
            lemlib::radToDeg(M_PI_2 - trajectory.back().heading), 
            1000,
            {.lead = r_config.mpose_lead}
        );
        chassis.waitUntilDone();
    }
    
    if(r_config.log) {
        for (const auto& line : logs) {
            std::cout << line;
            pros::delay(50);
        }
    }

    is_running = false;
}

void RamsetePathFollower::precompute_paths(const std::vector<std::string>& path_names) {
    auto* stored = new std::vector<std::string>(path_names);
    pros::Task t(precompute_paths_task, stored);
}

void RamsetePathFollower::precompute_paths_task(void* param) {
    auto* path_names = static_cast<std::vector<std::string>*>(param);

    precomputed_paths.clear();
    precomputed_paths.reserve(path_names->size());

    for (const auto& name : *path_names) {
        precomputed_paths.push_back(prepare_trajectory(name));
        pros::delay(10); 
    }
    delete path_names;
}

std::vector<std::pair<double,double>> RamsetePathFollower::parse_pairs(const std::string& line) {
    std::vector<std::pair<double,double>> result;
    std::string temp;
    bool inside_parens = false;
    for (char c : line) {
        if (c == '(') {
            temp.clear();
            inside_parens = true;
        } else if (c == ')') { 
            std::replace(temp.begin(), temp.end(), ',', ' ');
            std::istringstream ss(temp);
            double first, second;
            ss >> first >> second;
            result.emplace_back(first, second);
            inside_parens = false;
        } else if (inside_parens) {
            temp += c;
        }
    }
    return result;
}

std::vector<State> RamsetePathFollower::prepare_trajectory(const std::string& data) {
    std::istringstream ss(data);
    std::vector<std::pair<double,double>> X, L, A;
    std::string line;
    X.reserve(1000); L.reserve(1000); A.reserve(1000);
    
    while (std::getline(ss, line)) {
        if (line.find("X =") != std::string::npos) X = parse_pairs(line.substr(line.find('[')));
        else if (line.find("L =") != std::string::npos) L = parse_pairs(line.substr(line.find('[')));
        else if (line.find("A =") != std::string::npos) A = parse_pairs(line.substr(line.find('[')));
    }

    size_t n = X.size();
    if (n == 0) return {};

    std::vector<State> states(n);

    for (size_t i = 0; i < n; i++) {
        states[i].x = X[i].first;
        states[i].y = X[i].second;
        states[i].linear_vel = L[i].second;
        states[i].angular_vel = A[i].second;
    }

    if (n > 1) {
        states[0].heading = atan2(states[1].y - states[0].y, states[1].x - states[0].x);
        
        for (size_t i = 1; i < n - 1; i++) {
            double dy = states[i + 1].y - states[i - 1].y;
            double dx = states[i + 1].x - states[i - 1].x;
            states[i].heading = atan2(dy, dx);
        }
        
        states[n - 1].heading = atan2(states[n - 1].y - states[n - 2].y, states[n - 1].x - states[n - 2].x);
    }

    for(size_t i = 1; i < n; ++i) {
        double diff = states[i].heading - states[i-1].heading;
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        states[i].heading = states[i-1].heading + diff;
    }

    return states;
}
