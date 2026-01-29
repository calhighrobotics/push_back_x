#pragma once

#include "Eigen/Dense"
#include "velocityController.h" 
#include "lemlib/api.hpp"
#include <vector>
#include <string>
#include <cmath>
#include "globals.h" 

class LTVPathFollower {
public:
    // Struct to report performance back to the Tuner
    struct PathScore {
        double total_lateral_error = 0;
        double total_heading_error = 0;
        double total_jerk = 0;
        double final_score = 0; // The number the Tuner tries to minimize
    };

    struct ltvConfig {
        bool backwards = false;
        bool log = false;
        int path_index = -1;
        bool test = false;
        bool turnFirst = false;
        bool end_correction = false;
        float mpose_lead = 0.6f;
        float track_width = 11.55f;
        float max_lin_correction = 10000.0f;
        float max_ang_correction = 10000.0f;
        int exit_points = 5;

        // --- STABILIZED DEFAULT WEIGHTS ---
        // These are the values that fixed the "Sawtooth" behavior.
        // 1. Position Weights (High Accuracy)
/*
float q_x = 44148.2;
float q_y = 751291.2;
float q_theta = 275330;
float r_ang = 100.531;
float r_vel = 195.0193;
*/

float q_x = 44148.2;
float q_y = 751291.2;
float q_theta = 275330;
float r_ang = 100.531;
float r_vel = 195.0193;

    };

    LTVPathFollower(const VelocityControllerConfig& config);

    // Now returns PathScore instead of void
    PathScore followPath(const std::string& path_name, const ltvConfig& l_config);
    
    void precompute_paths(const std::vector<std::string>& path_names);

private:
    static constexpr float INCH_TO_METER = 0.0254f;
    
    static constexpr float wheel_circumference = (float)lemlib::Omniwheel::NEW_325 * M_PI * INCH_TO_METER;
    static constexpr float gear_ratio = 4.0f / 3.0f;
    static constexpr float rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60.0f;

    VoltageController controller;

    static inline std::vector<std::vector<State>> precomputed_paths;

    Eigen::MatrixXf dareSolver(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R);
    std::pair<Eigen::MatrixXf, Eigen::MatrixXf> discretizeAB(const Eigen::MatrixXf& contA, const Eigen::MatrixXf& contB, double dtSeconds);
    
    static void precompute_paths_task(void* param);
    static std::vector<std::pair<double,double>> parse_pairs(const std::string& line);
    static std::vector<State> prepare_trajectory(const std::string& data);

    double angleError(double robotAngle, double targetAngle);
    double clamp(double value, double min, double max);

    class Vector2 {
    public:
        Vector2(float x, float y);
        std::string latex() const;
        float x, y;
    };
};