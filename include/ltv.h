#pragma once

#include "Eigen/Dense"
#include "velocityController.h" // Assumes you have this from your Ramsete setup
#include "lemlib/api.hpp"
#include <vector>
#include <string>
#include "globals.h" // Assumes your motors/chassis are defined here

class LTVPathFollower {
public:
    // Config struct for LTV tuning
    struct ltvConfig {
        bool backwards = false;
        bool log = false;
        int path_index = -1;

        // LTV Cost Matrices diagonal elements
        // Q: State cost [x_error, y_error, theta_error]
        float q_x = 1.0f;
        float q_y = 8.0f;
        float q_theta = 6.0f;

        // R: Control cost [velocity_correction, angular_correction]
        // Higher R = smoother, less aggressive corrections
        float r_vel = 0.5f;
        float r_ang = 0.8f;

        bool test = false;
        bool turnFirst = false;
        bool end_correction = false;
        float mpose_lead = 0.6f;
        float track_width = 12.8f; // inches
    };

    // Constructor
    LTVPathFollower(const VelocityControllerConfig& config);

    // Method Declarations
    void followPath(const std::string& path_name, const ltvConfig& l_config);
    void precompute_paths(const std::vector<std::string>& path_names);

private:
    // Constants
    static constexpr float INCH_TO_METER = 0.0254f;
    
    // Physical constants (calculated same as Ramsete)
    static constexpr float wheel_circumference = (float)lemlib::Omniwheel::NEW_325 * M_PI * INCH_TO_METER;
    static constexpr float gear_ratio = 4.0f / 3.0f;
    static constexpr float rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60.0f;

    // Member Variables
    VoltageController controller;

    // Static member for path storage
    static inline std::vector<std::vector<State>> precomputed_paths;

    // --- Core LTV Math Helpers ---
    Eigen::MatrixXf dareSolver(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R);
    std::pair<Eigen::MatrixXf, Eigen::MatrixXf> discretizeAB(const Eigen::MatrixXf& contA, const Eigen::MatrixXf& contB, double dtSeconds);
    
    // --- General Helpers ---
    static void precompute_paths_task(void* param);
    static std::vector<std::pair<double,double>> parse_pairs(const std::string& line);
    static std::vector<State> prepare_trajectory(const std::string& data);

    double angleError(double robotAngle, double targetAngle);

    // Nested Helper Class for Logging
    class Vector2 {
    public:
        Vector2(float x, float y);
        std::string latex() const;
        float x, y;
    };
};