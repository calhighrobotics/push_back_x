#pragma once

#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "Eigen/Dense"
#include "velocityController.h"
#include <vector>
#include <string>
#include "globals.h"

class RamsetePathFollower {
public:
    // Config struct defined inside the class
    struct ramseteConfig {
        bool backwards = false;
        bool log = false;
        int path_index  = -1;
        float b = 2.0f;
        float zeta = 0.7f;
        int exit_points = 5;
        bool test = false;
    };

    // Constructor
    RamsetePathFollower(const VelocityControllerConfig& config, float b_, float zeta_);

    // Method Declarations
    // Note: 'const std::string&' and 'const ramseteConfig&' (Pass by Reference)
    void followPath(const std::string& path_name, const ramseteConfig& r_config);
    void precompute_paths(const std::vector<std::string>& path_names);

private:
    // Constants
    static constexpr float INCH_TO_METER = 0.0254f;
    static constexpr float TRACK_WIDTH = 11.5f;
    static constexpr float wheel_circumference = (float)lemlib::Omniwheel::NEW_325 * M_PI * INCH_TO_METER;
    static constexpr float gear_ratio = 4.0f / 3.0f;
    static constexpr float rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60.0f;

    // Member Variables
    VoltageController controller;
    const float b;
    const float zeta;

    // Static member for path storage
    static inline std::vector<std::vector<State>> precomputed_paths;

    // Helper functions
    static void precompute_paths_task(void* param);
    static std::vector<std::pair<double,double>> parse_pairs(const std::string& line);
    static std::vector<State> prepare_trajectory(const std::string& data);

    double angleError(double robotAngle, double targetAngle);
    double sinc(double x);

    // Nested Helper Class
    class Vector2 {
    public:
        Vector2(float x, float y);
        std::string latex() const;
        float x, y;
    };
};