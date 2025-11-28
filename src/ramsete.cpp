#pragma once

#include "Eigen/Eigen/Core"
#include "globals.h"
#include "paths.cpp"
#include <cmath>

int sign(double x) {
        return (x > 0) - (x < 0); // returns 1, 0, or -1
}

struct DrivetrainVoltages {
    double leftVoltage;
    double rightVoltage;
};

class VoltageController {
private:
    // Gains
    double kV;         // velocity feedforward
    double kaStraight; // acceleration feedforward
    double kaTurn;
    double ksStraight; // static friction compensation
    double ksTurn;
    double kP; // proportional gain
    double kI; // integral gain

    double trackWidth;
    double prevLinearVelocity{0.0};
    double prevAngularVelocity{0.0};
    double prevLeftError{0.0};
    double prevRightError{0.0};
    double leftIntegral{0.0};
    double rightIntegral{0.0};
    double integralThreshold{99999.0};
    double lastTargetVelocity{0.0};

    int sign(double x) {
        return (x > 0) - (x < 0); // returns 1, 0, or -1
    }

public:
    VoltageController(double kv, double kaStraight, double kaTurn, double ksStraight, double ksTurn, double kp,
                      double ki, double integralThreshold, double trackWidth)
        : kV(kv), kaStraight(kaStraight), kaTurn(kaTurn), ksStraight(ksStraight), ksTurn(ksTurn), kP(kp), kI(ki),
          integralThreshold(integralThreshold), trackWidth(trackWidth) {}

    DrivetrainVoltages update(double targetLinearVelocity, double targetAngularVelocity, double measuredLeftVelocity,
                              double measuredRightVelocity) {

        // Change in target angular velocity (angular acceleration)
        double deltaW = (targetAngularVelocity - prevAngularVelocity) / 0.01;
        // Change in target linear velocity (linear acceleration)
        double deltaV = (targetLinearVelocity - prevLinearVelocity) / 0.01;

        prevAngularVelocity = targetAngularVelocity;
        prevLinearVelocity = targetLinearVelocity;

        // Differential drive kinematics
        double leftVelocity = targetLinearVelocity - targetAngularVelocity * (trackWidth / 2.0);
        double rightVelocity = targetLinearVelocity + targetAngularVelocity * (trackWidth / 2.0);

        // Velocity errors
        double leftError = leftVelocity - measuredLeftVelocity;
        double rightError = rightVelocity - measuredRightVelocity;

        // Integrals
        if ((leftError < 0) != (prevLeftError < 0)) {
            leftIntegral = 0;
        }
        if (std::abs(leftError) < integralThreshold) {
            leftIntegral += leftError * 0.01;
        }
        if ((rightError < 0) != (prevRightError < 0)) {
            rightIntegral = 0;
        }
        if (std::abs(rightError) < integralThreshold) {
            rightIntegral += rightError * 0.01;
        }

        // Feedforward Terms
        // Acceleration term when going straight (a = F/m); kaTurn from moment of inertia (α = τ/I).
        double kaLeft = (kaStraight * deltaV) - (kaTurn * deltaW);
        double kaRight = (kaStraight * deltaV) + (kaTurn * deltaW);

        // Static friction component
        double ksLeft = (ksStraight * sign(leftVelocity)) - (ksTurn * sign(targetAngularVelocity));
        double ksRight = (ksStraight * sign(rightVelocity)) + (ksTurn * sign(targetAngularVelocity));

        double leftVoltage =
            std::clamp((kV * leftVelocity) + (kaLeft) + (ksLeft) + (kP * leftError) + (kI * leftIntegral), -12.0, 12.0);
        double rightVoltage =
            std::clamp((kV * rightVelocity) + (kaRight) + (ksRight) + (kP * rightError) + (kI * rightIntegral), -12.0, 12.0);

        return {leftVoltage, rightVoltage};
    }
};

struct VelocityControllerConfig {
    
    // --- Feedforward ---
    float kV {12.4370890785};
    float KA_turn {0.803031225567};
    float KA_straight {0.664537661342};
    float KS_turn {0.472796490892};
    float KS_straight {0.236548087393};

    // --- Feedback Gains ---
    float KP_straight {25.2621164319};
    float KI_straight {524.703492373};
};

VelocityControllerConfig test_config{
    12.4370890785,
    0.803031225567,
    0.664537661342,
    0.472796490892,
    0.236548087393,
    25.2621164319,
    524.703492373,
};


struct State {
    float x, y, heading, linear_vel, angular_vel;
};

std::vector<std::pair<double,double>> parse_pairs(const std::string& line) {
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

std::vector<State> prepare_trajectory(const std::string& data) {
    std::istringstream ss(data);

    std::vector<std::pair<double,double>> X, L, A;
    std::string line;
    
    while (std::getline(ss, line)) {
        if (line.find("X =") != std::string::npos) {
            X = parse_pairs(line.substr(line.find('[')));
        } else if (line.find("L =") != std::string::npos) {
            L = parse_pairs(line.substr(line.find('[')));
        } else if (line.find("A =") != std::string::npos) {
            A = parse_pairs(line.substr(line.find('[')));
        }
    }

    size_t n = X.size();
    std::vector<State> states(n);

    for (size_t i = 0; i < n; i++) {
        states[i].x = X[i].first;
        states[i].y = X[i].second;
        states[i].linear_vel = L[i].second;
        states[i].angular_vel = A[i].second;
    }

    for (size_t i = 0; i < n - 1; i++) {
        states[i].heading = atan2(states[i + 1].y - states[i].y, states[i + 1].x - states[i].x);
    }

    if (!states.empty()) {
        states.back().heading = states[n - 2].heading;
    }

    return states;
}




// Wrap angle difference into range [-pi, pi)
double angleError(double robotAngle, double targetAngle) {
    constexpr double TWO_PI = 2.0 * M_PI;

    double diff = std::fmod(targetAngle - robotAngle, TWO_PI);

    if (diff < -M_PI) {
        diff += TWO_PI;
    } else if (diff >= M_PI) {
        diff -= TWO_PI;
    }

    return diff;
}



inline double sinc(double x) {
    const double eps = 1e-9;
    if (std::abs(x) < eps) {
        // 1 - x^2/6 + O(x^4)
        return 1.0 - (x * x) / 6.0;
    }
    return std::sin(x) / x;
}

const double wheel_circumference = 4 * M_PI * INCH_TO_METER; // meters
const double gear_ratio = 1.25;
const double rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60;

void ramsete_auton(VelocityControllerConfig &config, std::string path_name) {
    
    VoltageController controller(
        config.kV,
        config.KA_straight,
        config.KA_turn,
        config.KS_straight,
        config.KS_turn,
        config.KP_straight,
        config.KI_straight,
        99999.0,
        10.0 * INCH_TO_METER
    );
    //4 and 0.2 current best
    const double b = 4;     
    const double zeta = 0.2; 
    const double track_width = 10.0 * INCH_TO_METER;

    std::vector<State> trajectory;
    trajectory.reserve(2000);
    trajectory = prepare_trajectory(path_name);
    if (trajectory.empty())
        return;

    int trajectory_size = trajectory.size();
    chassis.setPose(trajectory[0].x / INCH_TO_METER, trajectory[0].y / INCH_TO_METER, M_PI_2 - trajectory[0].heading, true);
    std::vector<std::string> logs;
    
    double time = 0.01;
    int counter = 0;
    for (const auto &target_state : trajectory) {
        // Record the start time of this specific loop iteration
        uint32_t start_time_ms = pros::millis();

        lemlib::Pose current_pose = chassis.getPose(true);
        Eigen::Matrix3d rotation_matrix;
        Eigen::Vector3d global_error;
        Eigen::Vector3d local_error;

        current_pose.x *= INCH_TO_METER;
        current_pose.y *= INCH_TO_METER;
        current_pose.theta = M_PI_2 - current_pose.theta;
        double AngleError = angleError(current_pose.theta, target_state.heading); 

        rotation_matrix << std::cos(current_pose.theta), std::sin(current_pose.theta), 0, 
        -std::sin(current_pose.theta), std::cos(current_pose.theta), 0, 
        0, 0, 1;

        global_error << target_state.x - current_pose.x, target_state.y - current_pose.y,
            AngleError;

        local_error = rotation_matrix * global_error;

        double vd = target_state.linear_vel;
        double wd = target_state.angular_vel;
        double e_x = local_error(0);
        double e_y = local_error(1);
        double e_t = local_error(2);

        double k = 2.0 * zeta * std::sqrt(wd * wd + b * vd * vd);
        double v_desired_ramsete = vd * std::cos(e_t) + k * e_x;
        double w_desired_ramsete = wd + k * e_t + (b * vd * sinc(e_t) * e_y);

        DrivetrainVoltages output_voltages = controller.update(v_desired_ramsete, w_desired_ramsete, leftMotors.get_actual_velocity() * rpm_to_mps_factor, rightMotors.get_actual_velocity() * rpm_to_mps_factor);
       
        rightMotors.move_voltage(output_voltages.rightVoltage * 1000.0);
        leftMotors.move_voltage(output_voltages.leftVoltage * 1000.0);


        std::ostringstream ss;
        ss << Vector2(current_pose.x, current_pose.y).latex() << ",";
        logs.push_back(ss.str());

        if(counter + 4 >= trajectory_size) {
            break;
        }
        counter++;
        time += 0.01;
        pros::Task::delay_until(&start_time_ms, 10);
    }
    chassis.moveToPose(trajectory[trajectory_size - 1].x / INCH_TO_METER,trajectory[trajectory_size - 1].y / INCH_TO_METER, lemlib::radToDeg(M_PI_2 - trajectory[trajectory.size()-1].heading),1000);
    chassis.waitUntilDone();

    rightMotors.brake();
    leftMotors.brake();

    for (const auto& line : logs) {
        std::cout << line;
        pros::delay(50);
    }
}

class RamsetePathFollower {
    private:
    VoltageController controller;
    const double b;
    const double zeta;
    const double track_width = 10;
};