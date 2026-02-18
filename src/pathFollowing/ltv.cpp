#include "ltv.h"
#include "lemlib/util.hpp"
#include "pros/motors.h"
#include <cmath>
#include <sys/types.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>

// --- Helper Class Implementations ---

LTVPathFollower::Vector2::Vector2(float x, float y) : x(x), y(y) {}

std::string LTVPathFollower::Vector2::latex() const {
    std::ostringstream oss;
    oss << "\\left(" << std::fixed << std::setprecision(3) << this->x << "," << this->y << "\\right)";
    return oss.str();
}

double LTVPathFollower::angleError(double robotAngle, double targetAngle) {
    return std::remainder(targetAngle - robotAngle, 2.0 * M_PI);
}

double LTVPathFollower::clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

LTVPathFollower::LTVPathFollower(const VelocityControllerConfig& config)
    : controller(
          config.kV,
          config.KA_straight,
          config.KA_turn,
          config.KS_straight,
          config.KS_turn,
          11.4953431776,
          54.5797495382,
          99999.0, 
          11.55f * INCH_TO_METER 
      ) {}


void LTVPathFollower::followPath(const std::string& path_name, const ltvConfig& l_config) {
    if (is_running) {
        cancel();
        waitUntilDone();
    }

    is_running = true;
    cancel_request = false;
    distance_traveled_inches = 0.0f;

    TaskParams* params = new TaskParams{this, path_name, l_config, {}};
    task = new pros::Task(task_trampoline, params, "LTVTask");
    
    if (task == nullptr) {
        delete params;
        is_running = false;
        std::cout << "[LTV] Failed to start task!" << std::endl;
        return;
    }
    pros::delay(10);
}

void LTVPathFollower::moveTo(float x, float y, float theta_deg, float timeout_ms, float max_speed, bool backwards) {
    // 1. Safety: Stop existing tasks
    if (is_running) {
        cancel();
        waitUntilDone();
    }

    // 2. Setup: Get current state
    lemlib::Pose start = chassis.getPose();
    
    // 3. Math: Calculate Target Heading correctly
    // LemLib uses "Degrees Clockwise from North" (0 = North, 90 = East)
    // Math uses "Radians Counter-Clockwise from East" (0 = East, pi/2 = North)
    float target_heading_lemlib = 0.0f;

    if (theta_deg <= -990.0f) {
        // Option A: Point towards the target
        float dx = x - start.x;
        float dy = y - start.y;
        
        // atan2 gives Math Angle (Rad CCW from East)
        float math_angle_rad = std::atan2(dy, dx);
        float math_angle_deg = lemlib::radToDeg(math_angle_rad);
        
        // Convert: LemLib = 90 - Math
        target_heading_lemlib = 90.0f - math_angle_deg;
    } else {
        // Option B: User specified absolute orientation
        target_heading_lemlib = theta_deg;
    }

    // 4. Create End Pose with CORRECT coordinate system
    lemlib::Pose end(x, y, target_heading_lemlib); 

    // 5. Config: Initialize properly to avoid garbage data
    ltvConfig config; // Relies on defaults set in header now
    config.backwards = backwards;
    config.log = false;
    // IMPORTANT: Explicitly disable turnFirst for moveTo, as we want spline movement
    config.turnFirst = false; 
    
    if(max_speed > 0) config.max_velocity = max_speed * INCH_TO_METER; 
    
    // 6. Generate Path
    std::vector<State> path = generateCurvedPath(start, end, config.max_velocity, config.max_acceleration, backwards);

    if (path.empty()) {
        std::cout << "[LTV] Error: Generated path is empty (start/end too close?)" << std::endl;
        return;
    }

    // 7. Execute
    is_running = true;
    cancel_request = false;
    distance_traveled_inches = 0.0f;

    TaskParams* params = new TaskParams{this, "", config, path};
    task = new pros::Task(task_trampoline, params, "LTVMoveTo");
    
    if (task == nullptr) {
        delete params;
        is_running = false;
        std::cout << "[LTV] Failed to create task" << std::endl;
        return;
    }

    // Give the task time to spin up
    pros::delay(20);
}
double LTVPathFollower::getPathLength(const std::string& path_name) {
    std::vector<State> trajectory = prepare_trajectory(path_name);
    if (trajectory.empty()) return 0.0;
    double length_meters = 0.0;
    for (size_t i = 1; i < trajectory.size(); ++i) {
        double dx = trajectory[i].x - trajectory[i-1].x;
        double dy = trajectory[i].y - trajectory[i-1].y;
        length_meters += std::sqrt(dx*dx + dy*dy);
    }
    return length_meters * METER_TO_INCH;
}

void LTVPathFollower::task_trampoline(void* params) {
    TaskParams* p = static_cast<TaskParams*>(params);
    if (p && p->instance) {
        p->instance->followPathImpl(p->path_name, p->config, p->dynamic_path);
    }
    delete p;
}

void LTVPathFollower::waitUntilDone() {
    while (is_running) {
        pros::delay(10);
    }
}

void LTVPathFollower::waitUntil(float dist_inches) {
    while (is_running && distance_traveled_inches < dist_inches) {
        pros::delay(10);
    }
}

void LTVPathFollower::waitUntil(float x_inch, float y_inch, float radius_inch) {
    while (is_running) {
        lemlib::Pose p = chassis.getPose();
        float dist = std::sqrt(std::pow(p.x - x_inch, 2) + std::pow(p.y - y_inch, 2));
        if (dist < radius_inch) {
            break;
        }
        pros::delay(10);
    }
}

void LTVPathFollower::cancel() {
    cancel_request = true;
}

bool LTVPathFollower::isRunning() {
    return is_running;
}

void LTVPathFollower::followPathImpl(const std::string& path_name, const ltvConfig& l_config, const std::vector<State>& dynamic_path) {
    std::vector<State> trajectory;

    if (!dynamic_path.empty()) {
        trajectory = dynamic_path;
    } else if(l_config.path_index >= 0 && (size_t)(l_config.path_index) < precomputed_paths.size()) {
        if (!precomputed_paths.at(l_config.path_index).empty()) {
            trajectory = precomputed_paths.at(l_config.path_index);
        }
    }
    if (trajectory.empty() && !path_name.empty()) {
        trajectory = prepare_trajectory(path_name);
    }

    if (trajectory.empty()) {
        std::cout << "[LTV] Error: Empty trajectory." << std::endl;
        is_running = false;
        return;
    }

    // --- Initial Pose Setup ---
    if(l_config.test) {
        // If testing, snap robot to start. If backwards, face the opposite way of path start.
        double start_theta = l_config.backwards ? trajectory[0].heading + M_PI : trajectory[0].heading;
        // Convert math angle back to GPS angle (0 is North, CW)
        double gps_start_theta = M_PI_2 - start_theta;
        chassis.setPose(trajectory[0].x / INCH_TO_METER, trajectory[0].y / INCH_TO_METER, lemlib::radToDeg(gps_start_theta));
    } else if(l_config.turnFirst) {
        double start_theta = l_config.backwards ? trajectory[0].heading + M_PI : trajectory[0].heading;
        double gps_target = lemlib::radToDeg(M_PI_2 - start_theta);
        chassis.turnToHeading(gps_target, 1000);
    }

    std::vector<std::string> logs;
    int trajectory_size = trajectory.size();
    u_int32_t start_time = pros::millis();
    uint32_t prev_time = pros::millis();
    
    double sum_lat_error = 0;
    double sum_head_error = 0;
    double sum_oscillation = 0; 
    float prev_w_cmd = 0;
    int steps = 0;

    lemlib::Pose start_pose = chassis.getPose();

    // Cache for DARE solution
    Eigen::MatrixXf cached_K(2, 3);
    cached_K.setZero();
    float last_solve_v = -9999.0f;
    float last_solve_w = -9999.0f;
    bool dare_solved_once = false;

    constexpr double FIXED_DT = 0.01; 

    for (int i = 0; i < trajectory_size; ++i) {
        if (cancel_request) break;

        u_int32_t current_time = pros::millis();
        
        // Use a minimum dt to prevent division by zero errors in logging
        double measured_dt = (current_time - prev_time) / 1000.0;
        if (measured_dt <= 0.002) measured_dt = 0.01;
        prev_time = current_time;

        const auto &target_state = trajectory[i];
        lemlib::Pose current_pose = chassis.getPose(true);
        
        distance_traveled_inches = start_pose.distance(current_pose);

        current_pose.x *= INCH_TO_METER;
        current_pose.y *= INCH_TO_METER;
        
        // --- Tuning Factors ---
        double progress = (double)i / trajectory_size;
        double velocity_scale = 1.0;
        double q_gain_mult = 1.0;
        double r_vel_mult = 1.0;
        double q_x_boost = 1.0;
        // Ramp down velocity at the end
        if (progress > 0.85) {
            // 0.0 at 0.85, 1.0 at 1.0
            double end_phase = (progress - 0.85) / 0.15;
            
            // Clamp to ensure we don't go out of bounds
            if (end_phase > 1.0) end_phase = 1.0; 
            
            // Calculate the ramp down (1.0 to 0.0)
            double ramp = 1.0 - end_phase;
            velocity_scale *= ramp;

            // Smoothly blend gains from 1.0 (normal) to target (stiff)
            // Formula: current = start + (end - start) * percent
            r_vel_mult = 1.0 + (1.0 * end_phase); // Blends 1.0 -> 2.0
            q_gain_mult = 1.0 + (2.0 * end_phase); // Blends 1.0 -> 3.0
        }
        if (std::abs(target_state.angular_vel) > 0.5) { 
            q_x_boost = 2; 
        }

        Eigen::Matrix3f Q_mat; 
        Q_mat << l_config.q_x * q_gain_mult * q_x_boost, 0, 0,
                 0, l_config.q_y * q_gain_mult, 0,
                 0, 0, l_config.q_theta * q_gain_mult;
        
        Eigen::Matrix2f R_mat;
        R_mat << l_config.r_vel * r_vel_mult, 0,
                 0, l_config.r_ang;

        // --- Coordinate Transformation Fix ---
        
        // 1. Get Robot Heading in Math Frame (Counter-Clockwise, 0 is East)
        double math_theta = M_PI_2 - current_pose.theta; 
        
        // 2. Define "Virtual" Robot Heading
        // If backing up, the "front" of our virtual robot is the physical rear.
        // We add PI to the angle.
        double effective_theta = l_config.backwards ? math_theta + M_PI : math_theta;
        
        // 3. Define Target Heading
        // We do NOT flip the target heading. The path tangent points in the direction of motion.
        // Our "Virtual Front" (physical rear) should align with this tangent.
        double target_heading = target_state.heading;
        
        double errorTheta = angleError(effective_theta, target_heading);
        
        Eigen::Vector3d global_error;
        global_error << target_state.x - current_pose.x, target_state.y - current_pose.y, errorTheta;
        
        // 4. Rotation Matrix using EFFECTIVE theta
        // This ensures X error is "distance along track" and Y error is "cross track"
        // relative to the direction of motion.
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix <<  std::cos(effective_theta), std::sin(effective_theta), 0, 
                           -std::sin(effective_theta), std::cos(effective_theta), 0, 
                            0, 0, 1;
        Eigen::Vector3d error = rotation_matrix * global_error;

        // --- Solver Inputs ---
        // We feed the solver Positive velocity. It calculates how to drive a robot "Forward" to fix errors.
        float v_ref = std::abs(target_state.linear_vel) * velocity_scale;
        
        // Angular velocity direction is invariant to gear. A left turn is a left turn.
        float w_ref = target_state.angular_vel * velocity_scale;

        float lateral_coupling = std::clamp(v_ref, -1.0f, 1.0f);

        constexpr float eps = -1e-3f;
        Eigen::Matrix3f A;
        A << eps, w_ref, 0,
             -w_ref, eps, lateral_coupling, 
             0, 0, eps;
             
        Eigen::Matrix<float, 3, 2> B;
        B << 1, 0,
             0, 0,
             0, 1;
        
        if (!dare_solved_once || 
            std::abs(v_ref - last_solve_v) > 0.15f || 
            std::abs(w_ref - last_solve_w) > 0.25f) {
            
            auto discAB = discretizeAB(A, B, measured_dt); //May need to change back to FIXED
            Eigen::MatrixXf X = dareSolver(discAB.first, discAB.second, Q_mat, R_mat);
            
            cached_K = (R_mat + discAB.second.transpose() * X * discAB.second).inverse() * discAB.second.transpose() * X * discAB.first;
            
            last_solve_v = v_ref;
            last_solve_w = w_ref;
            dare_solved_once = true;
        }

        Eigen::Vector2f u = cached_K * error.cast<float>();
        
        float u_v = clamp(u(0), -l_config.max_lin_correction, l_config.max_lin_correction);
        float u_w = clamp(u(1), -l_config.max_ang_correction, l_config.max_ang_correction);
        
        // Solver thinks we are driving forward. 
        float v_cmd = v_ref + u_v;
        float w_cmd = w_ref + u_w;

        // --- Output Inversion ---
        // If we are actually backing up, we must invert the Linear command.
        // Angular command stays same (To move rear Left, we steer Left/CCW).
        if (l_config.backwards) {
            v_cmd = -v_cmd;
        }

        double lat_err = std::abs(error(1));
        double head_err = std::abs(error(2));
        double instant_oscillation = std::abs(w_cmd - prev_w_cmd);
        prev_w_cmd = w_cmd;

        sum_lat_error += lat_err;
        sum_head_error += head_err;
        sum_oscillation += instant_oscillation;
        steps++;

        float left_actual_mps = leftMotors.get_actual_velocity() * rpm_to_mps_factor;
        float right_actual_mps = rightMotors.get_actual_velocity() * rpm_to_mps_factor;
        
        DrivetrainVoltages output_voltages = controller.update(
            v_cmd, w_cmd, left_actual_mps, right_actual_mps
        );

        output_voltages.rightVoltage = clamp(output_voltages.rightVoltage, -12.0, 12.0);
        output_voltages.leftVoltage = clamp(output_voltages.leftVoltage, -12.0, 12.0);

        rightMotors.move_voltage(output_voltages.rightVoltage * 1000.0);
        leftMotors.move_voltage(output_voltages.leftVoltage * 1000.0);
        
        if(l_config.log) {
            std::ostringstream ss;
            ss << Vector2(current_pose.x, current_pose.y).latex() << ",";
            logs.push_back(ss.str());
        }
        
        pros::Task::delay_until(&current_time, 10);
    }

    rightMotors.brake();
    leftMotors.brake();

    if (steps == 0) steps = 1; 
    double avg_lat_error = sum_lat_error / steps;
    double avg_head_error = sum_head_error / steps;
    double avg_jerk = sum_oscillation / steps;

    if(l_config.log) {
        std::cout << "\n--- LTV PERFORMANCE SUMMARY ---" << std::endl;
        std::cout << "Steps Completed: " << steps << " / " << trajectory_size << std::endl;
        std::cout << "Avg Lateral Error: " << (avg_lat_error / INCH_TO_METER) << " in" << std::endl;
        std::cout << "Avg Heading Error: " << lemlib::radToDeg(avg_head_error) << " deg" << std::endl;
        std::cout << "Avg Control Jerk:  " << avg_jerk << std::endl;
        
        std::cout << "\n--- COORDINATE LOG START ---" << std::endl;
        for (const auto& line : logs) {
            std::cout << line;
            pros::delay(10); 
        }
        std::cout << "\n--- LOG END ---" << std::endl;
    }

    is_running = false;
}

std::vector<State> LTVPathFollower::generateCurvedPath(lemlib::Pose start, lemlib::Pose end, float max_v, float max_a, bool backwards) {
    float x0 = start.x * INCH_TO_METER;
    float y0 = start.y * INCH_TO_METER;
    float x3 = end.x * INCH_TO_METER;
    float y3 = end.y * INCH_TO_METER;

    float dx = x3 - x0;
    float dy = y3 - y0;
    float dist_euclidean = std::sqrt(dx*dx + dy*dy);

    if (dist_euclidean < 0.01) return {};

    float theta_start = M_PI_2 - lemlib::degToRad(start.theta);
    float theta_end = M_PI_2 - lemlib::degToRad(end.theta);

    if (backwards) {
        theta_start += M_PI;
        theta_end += M_PI;
    }

    float d = std::min(dist_euclidean * 0.35f, 0.75f); 

    float P0x = x0;
    float P0y = y0;
    float P1x = P0x + d * std::cos(theta_start);
    float P1y = P0y + d * std::sin(theta_start);
    float P3x = x3;
    float P3y = y3;
    float P2x = P3x - d * std::cos(theta_end);
    float P2y = P3y - d * std::sin(theta_end);

    int samples = 100;
    std::vector<State> geometry_points;
    geometry_points.reserve(samples + 1);

    double total_arc_length = 0;
    float prev_x = P0x, prev_y = P0y;

    for (int i = 0; i <= samples; ++i) {
        float t = (float)i / samples;
        float u = 1 - t;
        float tt = t * t;
        float uu = u * u;
        float uuu = uu * u;
        float ttt = tt * t;

        float p_x = uuu * P0x + 3 * uu * t * P1x + 3 * u * tt * P2x + ttt * P3x;
        float p_y = uuu * P0y + 3 * uu * t * P1y + 3 * u * tt * P2y + ttt * P3y;

        float d_x = 3 * uu * (P1x - P0x) + 6 * u * t * (P2x - P1x) + 3 * tt * (P3x - P2x);
        float d_y = 3 * uu * (P1y - P0y) + 6 * u * t * (P2y - P1y) + 3 * tt * (P3y - P2y);

        float heading = std::atan2(d_y, d_x);

        if (i > 0) {
            total_arc_length += std::sqrt(std::pow(p_x - prev_x, 2) + std::pow(p_y - prev_y, 2));
        }

        State s;
        s.x = p_x;
        s.y = p_y;
        s.heading = heading;
        s.linear_vel = total_arc_length; 
        geometry_points.push_back(s);

        prev_x = p_x;
        prev_y = p_y;
    }

    float t_accel = max_v / max_a;
    float d_accel = 0.5f * max_a * t_accel * t_accel;
    float t_cruise = 0.0f;
    float d_cruise = 0.0f;

    if (2 * d_accel > total_arc_length) {
        d_accel = total_arc_length / 2.0f;
        t_accel = std::sqrt(2.0f * d_accel / max_a);
        max_v = max_a * t_accel; 
    } else {
        d_cruise = total_arc_length - 2 * d_accel;
        t_cruise = d_cruise / max_v;
    }

    float t_total = 2 * t_accel + t_cruise;
    float dt = 0.02f;
    int time_steps = std::ceil(t_total / dt);
    
    std::vector<State> path;
    path.reserve(time_steps + 5);

    for (int i = 0; i <= time_steps; ++i) {
        float t = i * dt;
        float current_dist_on_arc = 0.0f;
        float current_lin_vel = 0.0f;

        if (t < t_accel) {
            current_dist_on_arc = 0.5f * max_a * t * t;
            current_lin_vel = max_a * t;
        } else if (t < t_accel + t_cruise) {
            float t_c = t - t_accel;
            current_dist_on_arc = d_accel + max_v * t_c;
            current_lin_vel = max_v;
        } else if (t <= t_total) {
            float t_d = t - (t_accel + t_cruise);
            float dist_in_decel = max_v * t_d - 0.5f * max_a * t_d * t_d;
            current_dist_on_arc = d_accel + d_cruise + dist_in_decel;
            current_lin_vel = max_v - max_a * t_d;
        } else {
            current_dist_on_arc = total_arc_length;
            current_lin_vel = 0;
        }

        State interpolated;
        bool found = false;
        
        for(size_t j = 0; j < geometry_points.size() - 1; j++) {
            float d1 = geometry_points[j].linear_vel; 
            float d2 = geometry_points[j+1].linear_vel;
            
            if (current_dist_on_arc >= d1 && current_dist_on_arc <= d2) {
                float ratio = (current_dist_on_arc - d1) / (d2 - d1);
                interpolated.x = geometry_points[j].x + ratio * (geometry_points[j+1].x - geometry_points[j].x);
                interpolated.y = geometry_points[j].y + ratio * (geometry_points[j+1].y - geometry_points[j].y);
                
                float h1 = geometry_points[j].heading;
                float h2 = geometry_points[j+1].heading;
                float dh = h2 - h1;
                while (dh > M_PI) dh -= 2*M_PI;
                while (dh < -M_PI) dh += 2*M_PI;
                interpolated.heading = h1 + ratio * dh;

                found = true;
                break;
            }
        }
        if (!found) {
             interpolated.x = P3x;
             interpolated.y = P3y;
             // Fix #5: Use discretized path tangent, not control point tangent
             interpolated.heading = geometry_points.back().heading;
        }

        interpolated.linear_vel = current_lin_vel;
        
        // Fix #4: Prevent angular velocity spike at step 1
        if (i > 1) {
            float dtheta = interpolated.heading - path.back().heading;
            while (dtheta > M_PI) dtheta -= 2*M_PI;
            while (dtheta < -M_PI) dtheta += 2*M_PI;
            interpolated.angular_vel = dtheta / dt;
        } else {
            interpolated.angular_vel = 0;
        }

        path.push_back(interpolated);
    }
    
    return path;
}

Eigen::MatrixXf LTVPathFollower::dareSolver(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R) {
    Eigen::MatrixXf X = Q; 
    Eigen::MatrixXf X_prev;
    Eigen::MatrixXf K;

    for (int i = 0; i < 80; ++i) {
        X_prev = X;
        Eigen::MatrixXf R_BXB = R + B.transpose() * X * B;
        K = R_BXB.inverse() * B.transpose() * X * A;
        X = A.transpose() * X * (A - B * K) + Q;
        if ((X - X_prev).norm() < 1e-4) {
            break;
        }
    }
    return X;
}

std::pair<Eigen::MatrixXf, Eigen::MatrixXf> LTVPathFollower::discretizeAB(
    const Eigen::MatrixXf& contA, const Eigen::MatrixXf& contB, double dtSeconds) {
    if(dtSeconds <= 0.001) dtSeconds = 0.01;
    int states = contA.rows();
    int inputs = contB.cols();
    Eigen::MatrixXf M(states + inputs, states + inputs);
    M.setZero();
    M.topLeftCorner(states, states) = contA;
    M.topRightCorner(states, inputs) = contB;
    Eigen::MatrixXf Mdt = M * dtSeconds;
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(M.rows(), M.cols());
    Eigen::MatrixXf M2 = Mdt * Mdt;
    Eigen::MatrixXf phi = I + Mdt + (M2 * 0.5f); 
    Eigen::MatrixXf discA = phi.topLeftCorner(states, states);
    Eigen::MatrixXf discB = phi.topRightCorner(states, inputs);
    return {discA, discB};
}

void LTVPathFollower::precompute_paths(const std::vector<std::string>& path_names) {
    auto* stored = new std::vector<std::string>(path_names);
    pros::Task t(precompute_paths_task, stored, "PathCompute");
}

void LTVPathFollower::precompute_paths_task(void* param) {
    auto* path_names = static_cast<std::vector<std::string>*>(param);
    precomputed_paths.clear();
    precomputed_paths.reserve(path_names->size());
    for (const auto& name : *path_names) {
        precomputed_paths.push_back(prepare_trajectory(name));
        pros::delay(10); 
    }
    delete path_names;
}

std::vector<std::pair<double,double>> LTVPathFollower::parse_pairs(const std::string& line) {
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

std::vector<State> LTVPathFollower::prepare_trajectory(const std::string& data) {
    std::istringstream ss(data);
    std::vector<std::pair<double,double>> X, L, A;
    std::string line;
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
    if (n > 1) states[0].heading = atan2(states[1].y - states[0].y, states[1].x - states[0].x);
    for (size_t i = 1; i < n - 1; i++) {
        states[i].heading = atan2(states[i + 1].y - states[i - 1].y, states[i + 1].x - states[i - 1].x);
    }
    if (n > 1) states[n - 1].heading = atan2(states[n - 1].y - states[n - 2].y, states[n - 1].x - states[n - 2].x);
    return states;
}