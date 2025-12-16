#include "Eigen/Dense"
#include "globals.h"
#include "lemlib/util.hpp"
#include "velocityController.h"
#include <cmath>
#include <vector>
#include <string>
#include <sstream>

class RamsetePathFollower {

    private:

        static constexpr float INCH_TO_METER = 0.0254f;
        static constexpr float TRACK_WIDTH = 11.5f;

        const float wheel_circumference = (float)lemlib::Omniwheel::NEW_325 * M_PI * INCH_TO_METER;
        const float gear_ratio = 4.0f / 3.0f;

        const float rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60.0f;


        VoltageController controller;
        
        const float b;
        const float zeta;

        inline static std::vector<std::vector<State>> precomputed_paths;

    public:
        RamsetePathFollower(const VelocityControllerConfig& config, float b_, float zeta_)
            : controller(
                  config.kV,
                  config.KA_straight,
                  config.KA_turn,
                  config.KS_straight,
                  config.KS_turn,
                  config.KP_straight,
                  config.KI_straight,
                  99999.0,
                  TRACK_WIDTH * INCH_TO_METER
              ), b(b_), zeta(zeta_) {}

        struct ramseteConfig
        {
            bool backwards = false;
            bool log = false;
            int path_index  = -1;
            float b = this->b;
            float zeta = this->zeta;
            int exit_points = 10;
            bool test = false;
        };

        void followPath(const std::string path_name, const ramseteConfig r_config) {
            std::vector<State> trajectory;

            if(r_config.path_index - 1 >= 0 && r_config.path_index < precomputed_paths.size() && !precomputed_paths.at(r_config.path_index-1).empty()) {
                trajectory = precomputed_paths.at(r_config.path_index-1);
            }
            else {
                trajectory = prepare_trajectory(path_name);
            }

            if (trajectory.empty()) return;
            
            const int trajectory_size = trajectory.size();
            if(r_config.test)
                chassis.setPose(trajectory[0].x / INCH_TO_METER, trajectory[0].y / INCH_TO_METER, M_PI_2 - trajectory[0].heading, true);
            std::vector<std::string> logs;
            
            double time = 0.01;
            int counter = 0;
            for (const auto &target_state : trajectory) {
                uint32_t start_time_ms = pros::millis();

                lemlib::Pose current_pose = chassis.getPose(true);
                Eigen::Matrix3d rotation_matrix;
                Eigen::Vector3d global_error;
                Eigen::Vector3d local_error;

                current_pose.x *= INCH_TO_METER;
                current_pose.y *= INCH_TO_METER;
                current_pose.theta = M_PI_2 - current_pose.theta;

                double AngleError = angleError(current_pose.theta, target_state.heading + (r_config.backwards ? M_PI : 0)); 

                rotation_matrix << std::cos(current_pose.theta), std::sin(current_pose.theta), 0, 
                -std::sin(current_pose.theta), std::cos(current_pose.theta), 0, 
                0, 0, 1;

                global_error << target_state.x - current_pose.x, target_state.y - current_pose.y, AngleError;

                local_error = rotation_matrix * global_error;

                double vd = target_state.linear_vel * (r_config.backwards ? -1.0 : 1.0);
                double wd = target_state.angular_vel;
                double e_x = local_error(0);
                double e_y = local_error(1);
                double e_t = local_error(2);

                double k = 2.0 * r_config.zeta * std::sqrt(wd * wd + r_config.b * vd * vd);
                double v_desired_ramsete = vd * std::cos(e_t) + k * e_x;
                double w_desired_ramsete = wd + k * e_t + (r_config.b * vd * sinc(e_t) * e_y);

                DrivetrainVoltages output_voltages = controller.update(v_desired_ramsete, w_desired_ramsete, (leftMotors.get_actual_velocity() * rpm_to_mps_factor), (rightMotors.get_actual_velocity() * rpm_to_mps_factor));
            
                rightMotors.move_voltage(output_voltages.rightVoltage * 1000.0);
                leftMotors.move_voltage(output_voltages.leftVoltage * 1000.0);

                if(r_config.log) {
                    std::ostringstream ss;
                    ss << Vector2(current_pose.x, current_pose.y).latex() << ",";
                    logs.push_back(ss.str());
                }

                if(counter + r_config.exit_points >= trajectory_size) break;
                
                counter++;
                time += 0.01;
                pros::Task::delay_until(&start_time_ms, 10);
            }
            
            chassis.moveToPose(trajectory.back().x / INCH_TO_METER, trajectory.back().y / INCH_TO_METER, lemlib::radToDeg(M_PI_2 - trajectory.back().heading), 1000);
            chassis.waitUntilDone();
            rightMotors.brake();
            leftMotors.brake();

            if(r_config.log) {
                for (const auto& line : logs) {
                    std::cout << line;
                    pros::delay(50);
                }
            }
        }

        void precompute_paths(const std::vector<std::string>& path_names) {
            auto* stored = new std::vector<std::string>(path_names);
            pros::Task t(precompute_paths_task, stored);
        }

    private:
        static void precompute_paths_task(void* param) {
            auto* path_names = static_cast<std::vector<std::string>*>(param);

            precomputed_paths.clear();
            precomputed_paths.reserve(path_names->size());

            for (const auto& name : *path_names) {
                precomputed_paths.push_back(prepare_trajectory(name));
                pros::delay(10); 
            }
            delete path_names;
        }

        static std::vector<std::pair<double,double>> parse_pairs(const std::string& line) {
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

        static std::vector<State> prepare_trajectory(const std::string& data) {
            std::istringstream ss(data);
            std::vector<std::pair<double,double>> X, L, A;
            std::string line;
            
            while (std::getline(ss, line)) {
                if (line.find("X =") != std::string::npos) X = parse_pairs(line.substr(line.find('[')));
                else if (line.find("L =") != std::string::npos) L = parse_pairs(line.substr(line.find('[')));
                else if (line.find("A =") != std::string::npos) A = parse_pairs(line.substr(line.find('[')));
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

            if (!states.empty()) states.back().heading = states[n - 2].heading;

            return states;
        }

        double angleError(double robotAngle, double targetAngle) {
            constexpr double TWO_PI = 2.0 * M_PI;
            double diff = std::fmod(targetAngle - robotAngle, TWO_PI);
            if (diff < -M_PI) diff += TWO_PI;
            else if (diff >= M_PI) diff -= TWO_PI;
            return diff;
        }

        inline double sinc(double x) {
            const double eps = 1e-9;
            if (std::abs(x) < eps) return 1.0 - (x * x) / 6.0;
            return std::sin(x) / x;
        }

        class Vector2 {
            public:
                Vector2(float x, float y) : x(x), y(y) {}
                std::string latex() const {
                    std::ostringstream oss;
                    oss << "\\left(" << std::fixed << this->x << "," << std::fixed << this->y << "\\right)";
                    return oss.str();
                }
                float x, y;
         };
};