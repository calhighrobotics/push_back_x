#include "ltv.h"
#include "lemlib/util.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

LTVPathFollower::Vector2::Vector2(float x, float y) : x(x), y(y) {}

std::string LTVPathFollower::Vector2::latex() const {
    std::ostringstream oss;
    oss << "\\left(" << std::fixed << this->x << "," << std::fixed << this->y << "\\right)";
    return oss.str();
}

double LTVPathFollower::angleError(double robotAngle, double targetAngle) {
    constexpr double TWO_PI = 2.0 * M_PI;
    double diff = std::fmod(targetAngle - robotAngle, TWO_PI);
    if (diff < -M_PI) diff += TWO_PI;
    else if (diff >= M_PI) diff -= TWO_PI;
    return diff;
}

LTVPathFollower::LTVPathFollower(const VelocityControllerConfig& config)
    : controller(
          config.kV,
          config.KA_straight,
          config.KA_turn,
          config.KS_straight,
          config.KS_turn,
          config.KP_straight,
          config.KI_straight,
          99999.0, 
          12.8f * INCH_TO_METER
      ) {}

Eigen::MatrixXf LTVPathFollower::dareSolver(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R) {
    Eigen::MatrixXf X = Q; 
    Eigen::MatrixXf X_prev;
    Eigen::MatrixXf K;

    for (int i = 0; i < 1000; ++i) {
        X_prev = X;
        K = (R + B.transpose() * X * B).inverse() * B.transpose() * X * A;
        X = A.transpose() * X * (A - B * K) + Q;

        if ((X - X_prev).norm() < 1e-4) {
            break;
        }
    }
    return X;
}

std::pair<Eigen::MatrixXf, Eigen::MatrixXf> LTVPathFollower::discretizeAB(
    const Eigen::MatrixXf& contA, const Eigen::MatrixXf& contB, double dtSeconds) {

    int states = contA.rows();
    int inputs = contB.cols();

    Eigen::MatrixXf M(states + inputs, states + inputs);
    M.setZero();
    M.topLeftCorner(states, states) = contA;
    M.topRightCorner(states, inputs) = contB;

    Eigen::MatrixXf Mdt = M * dtSeconds;
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(M.rows(), M.cols());
    Eigen::MatrixXf M2 = Mdt * Mdt;
    
    Eigen::MatrixXf phi = I + Mdt + (M2 / 2.0); 

    Eigen::MatrixXf discA = phi.topLeftCorner(states, states);
    Eigen::MatrixXf discB = phi.topRightCorner(states, inputs);

    return {discA, discB};
}

void LTVPathFollower::precompute_paths(const std::vector<std::string>& path_names) {
    auto* stored = new std::vector<std::string>(path_names);
    pros::Task t(precompute_paths_task, stored);
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

void LTVPathFollower::followPath(const std::string& path_name, const ltvConfig& l_config) {
    std::vector<State> trajectory;
    
    if(l_config.path_index >= 0 && (size_t)(l_config.path_index) < precomputed_paths.size()) {
        if (!precomputed_paths.at(l_config.path_index).empty()) {
            trajectory = precomputed_paths.at(l_config.path_index);
        }
    }
    if (trajectory.empty()) trajectory = prepare_trajectory(path_name);
    if (trajectory.empty()) return;

    Eigen::Matrix3f Q_mat; 
    Q_mat << l_config.q_x, 0, 0,
             0, l_config.q_y, 0,
             0, 0, l_config.q_theta;

    Eigen::Matrix2f R_mat;
    R_mat << l_config.r_vel, 0,
             0, l_config.r_ang;

    if(l_config.test) {
        chassis.setPose(trajectory[0].x / INCH_TO_METER, trajectory[0].y / INCH_TO_METER, M_PI_2 - trajectory[0].heading, true);
    } else if(l_config.turnFirst) {
        double targetH = lemlib::radToDeg(M_PI_2 - trajectory[0].heading);
        chassis.turnToHeading(l_config.backwards ? targetH + 180 : targetH, 1000);
    }

    std::vector<std::string> logs;
    int trajectory_size = trajectory.size();
    
    double dt = 0.01; 

    for (int i = 0; i < trajectory_size; ++i) {
        uint32_t start_time_ms = pros::millis();
        const auto &target_state = trajectory[i];

        lemlib::Pose current_pose = chassis.getPose(true);
        current_pose.x *= INCH_TO_METER;
        current_pose.y *= INCH_TO_METER;
        current_pose.theta = M_PI_2 - current_pose.theta; 

        double targetHeadingAdjusted = target_state.heading + (l_config.backwards ? M_PI : 0);
        double errorTheta = angleError(current_pose.theta, targetHeadingAdjusted);
        
        Eigen::Vector3d global_error;
        global_error << target_state.x - current_pose.x, target_state.y - current_pose.y, errorTheta;

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix <<  std::cos(current_pose.theta), std::sin(current_pose.theta), 0, 
                           -std::sin(current_pose.theta), std::cos(current_pose.theta), 0, 
                            0, 0, 1;

        Eigen::Vector3d error = rotation_matrix * global_error;

        float v_ref = target_state.linear_vel * (l_config.backwards ? -1.0 : 1.0);
        float w_ref = target_state.angular_vel;

        float v_calc = v_ref;
        if (std::abs(v_calc) < 0.25) {
            v_calc = (v_calc >= 0) ? 0.25 : -0.25;
        }

        Eigen::Matrix3f A;
        A << 0, 0, 0,
             0, 0, v_calc,
             0, 0, 0;

        Eigen::Matrix<float, 3, 2> B;
        B << 1, 0,
             0, 0,
             0, 1;

        auto discAB = discretizeAB(A, B, dt);

        Eigen::MatrixXf X = dareSolver(discAB.first, discAB.second, Q_mat, R_mat);
        
        Eigen::MatrixXf K = (R_mat + discAB.second.transpose() * X * discAB.second).inverse() * discAB.second.transpose() * X * discAB.first;

        Eigen::Vector2f u = K * error.cast<float>();

        float v_cmd = v_ref + u(0);
        float w_cmd = w_ref + u(1);

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

        pros::Task::delay_until(&start_time_ms, 10);
    }

    if(!l_config.test && l_config.end_correction) {
        pros::delay(100); 
        chassis.moveToPose(
            trajectory.back().x / INCH_TO_METER, 
            trajectory.back().y / INCH_TO_METER, 
            lemlib::radToDeg(M_PI_2 - trajectory.back().heading), 
            2000, 
            {.lead = l_config.mpose_lead}
        );
        chassis.waitUntilDone();
    }
    
    rightMotors.brake();
    leftMotors.brake();

    if(l_config.log) {
        for (const auto& line : logs) {
            std::cout << line;
            pros::delay(50);
        }
    }
}
