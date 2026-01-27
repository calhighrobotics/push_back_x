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

Eigen::MatrixXf LTVPathFollower::dareSolver(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R) {
    Eigen::MatrixXf X = Q; 
    Eigen::MatrixXf X_prev;
    Eigen::MatrixXf K;

    for (int i = 0; i < 25; ++i) {
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

LTVPathFollower::PathScore LTVPathFollower::followPath(const std::string& path_name, const ltvConfig& l_config) {
    // [Setup Code Remains the Same...]
    std::vector<State> trajectory;
    if(l_config.path_index >= 0 && (size_t)(l_config.path_index) < precomputed_paths.size()) {
        if (!precomputed_paths.at(l_config.path_index).empty()) {
            trajectory = precomputed_paths.at(l_config.path_index);
        }
    }
    if (trajectory.empty()) trajectory = prepare_trajectory(path_name);
    if (trajectory.empty()) {
        std::cout << "[LTV] Error: Empty trajectory." << std::endl;
        return {0,0,0,1000000.0};
    }

    // Tuning Matrices
    Eigen::Matrix3f Q_mat; 
    Q_mat << l_config.q_x, 0, 0,
             0, l_config.q_y, 0,
             0, 0, l_config.q_theta;
    Eigen::Matrix2f R_mat;
    R_mat << l_config.r_vel, 0,
             0, l_config.r_ang;

    // Initial Pose Setup
    if(l_config.test) {
        chassis.setPose(trajectory[0].x / INCH_TO_METER, trajectory[0].y / INCH_TO_METER, l_config.backwards ? M_PI_2 - trajectory[0].heading + M_PI : M_PI_2 - trajectory[0].heading, true);
    } else if(l_config.turnFirst) {
        double targetH = lemlib::radToDeg(M_PI_2 - trajectory[0].heading);
        chassis.turnToHeading(l_config.backwards ? targetH + 180 : targetH, 1000);
    }

    std::vector<std::string> logs;
    int trajectory_size = trajectory.size();
    
    // --- FIX 1: CONSTANT CONTROL DT ---
    // We trust the loop delay more than the system clock for derivative math stability
    const double CONTROL_DT = 0.01; 

    // -- Scoring Variables --
    double sum_lat_error = 0;
    double sum_head_error = 0;
    double sum_jerk = 0;
    float prev_w_cmd = 0;
    int steps = 0;

    // --- FIX 2: GAIN CACHING SETUP ---
    // Initialize K with zeros. We will compute it on the first tick.
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(2, 3);
    int compute_counter = 0; 
    const int RECOMPUTE_INTERVAL = 5; // Re-solve Riccati every 50ms

    uint32_t start_time = pros::millis();
    uint32_t next_wake_time = start_time;

    for (int i = 0; i < trajectory_size; ++i) {
        // We use the loop structure to enforce timing, but math uses CONTROL_DT
        
        const auto &target_state = trajectory[i];
        lemlib::Pose current_pose = chassis.getPose(true);
        
        // Convert LemLib pose to standard math frame (Meters & Radians)
        current_pose.x *= INCH_TO_METER;
        current_pose.y *= INCH_TO_METER;
        double math_theta = M_PI_2 - current_pose.theta; 
        
        double targetHeadingAdjusted = target_state.heading + (l_config.backwards ? M_PI : 0);
        double errorTheta = angleError(math_theta, targetHeadingAdjusted);
        
        // Calculate Error Vector (Global -> Local)
        Eigen::Vector3d global_error;
        global_error << target_state.x - current_pose.x, target_state.y - current_pose.y, errorTheta;
        
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix <<  std::cos(math_theta), std::sin(math_theta), 0, 
                           -std::sin(math_theta), std::cos(math_theta), 0, 
                            0, 0, 1;
        Eigen::Vector3d error = rotation_matrix * global_error;

        float v_ref = target_state.linear_vel * (l_config.backwards ? -1.0 : 1.0);
        float w_ref = target_state.angular_vel;

        // --- SINGULARITY HANDLING ---
        float lateral_coupling = v_ref;
        float v_model = v_ref;
        if (std::abs(v_model) < 0.05) {
            v_model = (v_model >= 0) ? 0.05 : -0.05; 
            lateral_coupling = 0.0; 
        }

        // --- FIX 2: CONDITIONAL DARE SOLVE ---
        // Only run the heavy matrix math every N ticks. 
        // This removes structural jitter while keeping gains accurate enough.
        if (compute_counter % RECOMPUTE_INTERVAL == 0) {
            Eigen::Matrix3f A;
            A << 0, w_ref, 0,
                 -w_ref, 0, lateral_coupling, 
                 0, 0, 0;
            
            Eigen::Matrix<float, 3, 2> B;
            B << 1, 0,
                 0, 0,
                 0, 1;
            
            // Use stable CONTROL_DT for discretization
            auto discAB = discretizeAB(A, B, CONTROL_DT);
            Eigen::MatrixXf X = dareSolver(discAB.first, discAB.second, Q_mat, R_mat);
            
            // Update K
            K = (R_mat + discAB.second.transpose() * X * discAB.second).inverse() * discAB.second.transpose() * X * discAB.first;
        }
        compute_counter++;

        // --- FIX 3: ERROR CLAMPING ---
        // Prevents gain spikes from reacting to massive instantaneous errors (e.g. vision jumps)
        Eigen::Vector3f e_clamped = error.cast<float>();
        
        // Clamp Lateral Error (Y) to +/- 0.3 meters (approx 1 foot)
        // Clamp Heading Error to +/- 0.5 radians (approx 30 degrees)
        // We leave X (Longitudinal) unclamped usually, or looser.
        e_clamped(1) = clamp(e_clamped(1), -0.3f, 0.3f);
        e_clamped(2) = clamp(e_clamped(2), -0.5f, 0.5f);

        // Apply Control Law
        Eigen::Vector2f u = K * e_clamped;
        
        float u_v = clamp(u(0), -l_config.max_lin_correction, l_config.max_lin_correction);
        float u_w = clamp(u(1), -l_config.max_ang_correction, l_config.max_ang_correction);
        
        float v_cmd = v_ref + u_v;
        float w_cmd = w_ref + u_w;

        // --- SCORING (Corrected for Local Frame) ---
        // error(0) = X error (Longitudinal - timing/speed error)
        // error(1) = Y error (Lateral - deviation from path)
        
        // Use error(1) for lateral scoring, not vector norm of X+Y
        double lat_err = std::abs(error(1)); 
        double head_err = std::abs(error(2));
        double current_jerk = std::abs(w_cmd - prev_w_cmd);
        prev_w_cmd = w_cmd;

        sum_lat_error += lat_err;
        sum_head_error += head_err;
        sum_jerk += current_jerk;
        steps++;

        // --- MOTOR OUTPUT ---
        float left_actual_mps = leftMotors.get_actual_velocity() * rpm_to_mps_factor;
        float right_actual_mps = rightMotors.get_actual_velocity() * rpm_to_mps_factor;
        
        DrivetrainVoltages output_voltages = controller.update(
            v_cmd, 
            w_cmd, 
            left_actual_mps, 
            right_actual_mps
        );

        rightMotors.move_voltage(output_voltages.rightVoltage * 1000.0);
        leftMotors.move_voltage(output_voltages.leftVoltage * 1000.0);
        
        if(l_config.log) {
            std::ostringstream ss;
            ss << Vector2(current_pose.x, current_pose.y).latex() << ",";
            logs.push_back(ss.str());
        }

        // --- PRECISE LOOP TIMING ---
        // Ensures the loop actually takes 10ms, matching CONTROL_DT
        next_wake_time += 10;
        pros::Task::delay_until(&next_wake_time, 10);
    }

    // Stop Motors
    rightMotors.brake();
    leftMotors.brake();
    
    // Logging Output
    if(l_config.log) {
        std::cout << "--- PATH LOG START ---" << std::endl;
        for (const auto& line : logs) {
            std::cout << line;
            pros::delay(10);
        }
        std::cout << std::endl << "--- PATH LOG END ---" << std::endl;
    }

    // --- FINAL SCORE ---
    PathScore score;
    if (steps == 0) steps = 1;

    score.total_lateral_error = sum_lat_error / steps;
    score.total_heading_error = sum_head_error / steps;
    score.total_jerk = sum_jerk / steps;

    double performance_cost = (score.total_lateral_error * 1000.0) + 
                              (score.total_heading_error * 2000.0) + 
                              (score.total_jerk * 50.0);

    if (steps > trajectory_size * 1.5) {
        performance_cost += 5000.0;
    }

    score.final_score = performance_cost;
    return score;
}