#include "Eigen/Dense"
#include "globals.h"
#include "lemlib/util.hpp"
#include "paths.cpp"
#include "pathFollowing/velocityController.h"
#include <cmath>


class RamsetePathFollower {

    public:
        RamsetePathFollower(const VelocityControllerConfig& config, double b_, double zeta_)
            : controller(
                  config.kV,
                  config.KA_straight,
                  config.KA_turn,
                  config.KS_straight,
                  config.KS_turn,
                  config.KP_straight,
                  config.KI_straight,
                  99999.0,
                  10.0 * INCH_TO_METER
              ), b(b_), zeta(zeta_) {}

        void followPath(const std::string path_name) {

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


    private:
        const double wheel_circumference = 4 * M_PI * INCH_TO_METER; // meters
        const double gear_ratio = 1.25;
        const double rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60;
        VoltageController controller;
        const double b;
        const double zeta;
        const double track_width = 10;

        int sign(double x) {
            return (x > 0) - (x < 0); // returns 1, 0, or -1
        }

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

};