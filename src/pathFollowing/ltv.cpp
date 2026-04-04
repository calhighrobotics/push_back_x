#include "ltv.h"
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

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
          config.KP_straight,
          0,
          5000.0, 
          11.45f * INCH_TO_METER 
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

    if(l_config.test) {
        double start_theta = l_config.backwards ? trajectory[0].heading + M_PI : trajectory[0].heading;
        double gps_start_theta = M_PI_2 - start_theta;
        chassis.setPose(trajectory[0].x / INCH_TO_METER, trajectory[0].y / INCH_TO_METER, lemlib::radToDeg(gps_start_theta));
    } else if(l_config.turnFirst) {
        double start_theta = l_config.backwards ? trajectory[0].heading + M_PI : trajectory[0].heading;
        double gps_target = lemlib::radToDeg(M_PI_2 - start_theta);
        chassis.turnToHeading(gps_target, 1000);
    }

    std::vector<std::string> logs;
    std::vector<std::string> logs_2;
    int trajectory_size = trajectory.size();
    u_int32_t start_time = pros::millis();
    uint32_t prev_time = pros::millis();
    
    double sum_lat_error = 0;
    double sum_head_error = 0;
    double sum_oscillation = 0; 
    float prev_w_cmd = 0;
    int steps = 0;

    lemlib::Pose start_pose = chassis.getPose();

    Eigen::MatrixXf cached_K(2, 3);
    cached_K.setZero();
    float last_solve_v = -9999.0f;
    float last_solve_w = -9999.0f;
    bool dare_solved_once = false;

    constexpr double FIXED_DT = 0.01; 
    
    for (int i = 0; i < trajectory_size; ++i) {
        if (cancel_request) break;

        u_int32_t current_time = pros::millis();
        
        double measured_dt = (current_time - prev_time) / 1000.0;
        if (measured_dt <= 0.002) measured_dt = 0.01;
        prev_time = current_time;

        const auto &target_state = trajectory[i];
        lemlib::Pose current_pose = chassis.getPose(true);
        
        distance_traveled_inches = start_pose.distance(current_pose);

        current_pose.x *= INCH_TO_METER;
        current_pose.y *= INCH_TO_METER;
        
        double progress = (double)i / trajectory_size;
        double velocity_scale = 1.0;
        double q_gain_mult = 1.0;
        double r_vel_mult = 1.0;
        double q_x_boost = 1.0;
        
        float q_x_effective = (l_config.backwards) ? l_config.q_x_b : l_config.q_x;
        float q_y_effective = (l_config.backwards) ? l_config.q_y_b : l_config.q_y;
        float q_theta_effective = (l_config.backwards) ? l_config.q_theta_b : l_config.q_theta;
        float r_ang_effective = (l_config.backwards) ? l_config.r_ang_b : l_config.r_ang;
        float r_vel_effective = (l_config.backwards) ? l_config.r_vel_b : l_config.r_vel;
        


        Eigen::Matrix3f Q_mat; 
        Q_mat << q_x_effective * q_gain_mult * q_x_boost * l_config.q_scalar, 0, 0,
                 0, q_y_effective * q_gain_mult * l_config.q_scalar, 0,
                 0, 0, q_theta_effective * q_gain_mult * l_config.q_scalar;
        
        Eigen::Matrix2f R_mat;
        R_mat << r_vel_effective * r_vel_mult, 0,
                 0, r_ang_effective;

        double math_theta = M_PI_2 - current_pose.theta; 
        
        double effective_theta = l_config.backwards ? math_theta + M_PI : math_theta;
        
        double target_heading = target_state.heading;
        
        double errorTheta = angleError(effective_theta, target_heading);
        
        Eigen::Vector3d global_error;
        global_error << target_state.x - current_pose.x, target_state.y - current_pose.y, errorTheta;
        
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix <<  std::cos(effective_theta), std::sin(effective_theta), 0, 
                           -std::sin(effective_theta), std::cos(effective_theta), 0, 
                            0, 0, 1;
        Eigen::Vector3d error = rotation_matrix * global_error;


        //float lateral_coupling = std::clamp(v_ref, -1.0f, 1.0f);

       float v_ref = std::abs(target_state.linear_vel) * velocity_scale;
        float w_ref = target_state.angular_vel * velocity_scale;

        // --- THE FIX ---
        // 1. Prevent loss of controllability at low speeds. 
        // We clamp the velocity used in the A matrix so the LQR never "forgets" how to steer.
        float a_v_ref = (v_ref < 0.15f) ? 0.15f : v_ref;

        // 2. The CORRECT unicycle A matrix. You must include w_ref!
        // Without w_ref, the controller doesn't know the error frame is rotating during curves.
        constexpr float eps = -1e-3f;
        Eigen::Matrix3f A;
        A << eps, w_ref, 0,
            -w_ref, eps, a_v_ref, 
             0, 0, eps;
             
        Eigen::Matrix<float, 3, 2> B;
        B << 1, 0,
             0, 0,
             0, 1;
            
        auto discAB = discretizeAB(A, B, measured_dt);
        Eigen::MatrixXf X = dareSolver(discAB.first, discAB.second, Q_mat, R_mat);
        
        cached_K = (R_mat + discAB.second.transpose() * X * discAB.second).inverse() * discAB.second.transpose() * X * discAB.first;
        
        last_solve_v = v_ref;
        last_solve_w = w_ref;
        
        Eigen::Vector2f u = cached_K * error.cast<float>();
        
        float u_v = clamp(u(0), -l_config.max_lin_correction, l_config.max_lin_correction);
        float u_w = clamp(u(1), -l_config.max_ang_correction, l_config.max_ang_correction);

        
        float v_cmd = v_ref + u_v;
        float w_cmd = w_ref + u_w;


        if(l_config.backwards) {
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
            //ss << Vector2((current_time - start_time) / 1000.0, rightMotors.get_actual_velocity() * rpm_to_mps_factor).latex() << ",";
            //ss << Vector2((current_time - start_time) / 1000.0, u_w).latex() << ",";
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
        std::cout << "\n--- COORDINATE LOG END ---" << std::endl;
    }

    is_running = false;
}

Eigen::MatrixXf LTVPathFollower::dareSolver(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R) {
    int states = A.rows();
    
    // Initial conditions for SDA
    Eigen::MatrixXf A_k = A;
    // G_0 = B * R^-1 * B^T
    // Using LLT decomposition since R is symmetric positive definite
    Eigen::MatrixXf G_k = B * R.llt().solve(B.transpose()); 
    Eigen::MatrixXf H_k;
    Eigen::MatrixXf H_k1 = Q;
    
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(states, states);

    for (int i = 0; i < 80; ++i) {
        H_k = H_k1;
        Eigen::MatrixXf W = I + G_k * H_k;
        auto W_solver = W.partialPivLu();
        Eigen::MatrixXf V_1 = W_solver.solve(A_k);
        Eigen::MatrixXf V_2 = W_solver.solve(G_k);

        G_k += A_k * V_2 * A_k.transpose();
        H_k1 = H_k + V_1.transpose() * H_k * A_k;
        A_k *= V_1;
        if ((H_k1 - H_k).norm() <= 1e-10f * H_k1.norm()) {
            break;
        }
    }

    return H_k1;
}

std::pair<Eigen::MatrixXf, Eigen::MatrixXf> LTVPathFollower::discretizeAB(
    const Eigen::MatrixXf& contA, const Eigen::MatrixXf& contB, double dtSeconds) {
    dtSeconds = 0.01;
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

std::vector<std::vector<double>> LTVPathFollower::parse_tuples(const std::string& line) {
    std::vector<std::vector<double>> result;
    std::string temp;
    bool inside_parens = false;
    
    for (char c : line) {
        if (c == '(') {
            temp.clear();
            inside_parens = true;
        } else if (c == ')') { 
            std::replace(temp.begin(), temp.end(), ',', ' ');
            std::istringstream ss(temp);
            std::vector<double> tuple;
            double val;
            
            while (ss >> val) {
                tuple.push_back(val);
            }
            
            result.push_back(tuple);
            inside_parens = false;
        } else if (inside_parens) {
            temp += c;
        }
    }
    return result;
}

std::vector<State> LTVPathFollower::prepare_trajectory(const std::string& data) {
    std::istringstream ss(data);
    std::vector<std::vector<double>> P, V;
    std::string line;
    
    while (std::getline(ss, line)) {
        if (line.find("P =") != std::string::npos) {
            P = parse_tuples(line.substr(line.find('{')));
        } else if (line.find("V =") != std::string::npos) {
            V = parse_tuples(line.substr(line.find('{')));
        }
    }
    
    size_t n = std::min(P.size(), V.size());
    if (n == 0) return {};
    
    std::vector<State> states(n);
    for (size_t i = 0; i < n; i++) {
        if (P[i].size() >= 3) {
            states[i].x = P[i][0];
            states[i].y = P[i][1];
            states[i].heading = P[i][2];
        }
        
        if (V[i].size() >= 2) {
            states[i].linear_vel = V[i][0];
            states[i].angular_vel = V[i][1];
        }
    }
    
    return states;
}