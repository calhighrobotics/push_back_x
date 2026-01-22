#pragma once

#include "Eigen/Dense"
#include "velocityController.h"
#include "lemlib/api.hpp" 
#include <vector>
#include <string>
#include <atomic>
#include "globals.h"

class RamsetePathFollower {
public:
    // Config struct
    struct ramseteConfig {
        bool backwards = false;
        bool log = false;
        int path_index  = -1;
        float b = 2.0f;
        float zeta = 0.7f;
        bool test = false;
        bool turnFirst = false;
        bool end_correction = false;
        float mpose_lead = 0.6f;
    };

    RamsetePathFollower(const VelocityControllerConfig& config, float b_, float zeta_);

    // --- Main Methods ---
    // Now launches a task and returns immediately
    void followPath(const std::string& path_name, const ramseteConfig& r_config);
    
    // Blocks until the movement is complete
    void waitUntilDone();

    // Blocks until the robot has traveled 'dist_inches' along the path
    void waitUntil(float dist_inches);
    
    // Stops the current movement immediately
    void cancel();

    // Returns true if the robot is currently following a path
    bool isRunning();

    void precompute_paths(const std::vector<std::string>& path_names);

private:
    static constexpr float INCH_TO_METER = 0.0254f;
    static constexpr float TRACK_WIDTH = 12.8f;
    static constexpr float wheel_circumference = (float)lemlib::Omniwheel::NEW_325 * M_PI * INCH_TO_METER;
    static constexpr float gear_ratio = 4.0f / 3.0f;
    static constexpr float rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60.0f;

    VoltageController controller;
    const float b;
    const float zeta;

    // --- Task & State Management ---
    pros::Task* task = nullptr;
    std::atomic<bool> is_running {false};
    std::atomic<float> distance_traveled {0.0f}; // in inches
    std::atomic<bool> cancel_request {false};

    // Internal struct to pass arguments to the task
    struct TaskParams {
        RamsetePathFollower* instance;
        std::string path_name;
        ramseteConfig config;
    };

    static inline std::vector<std::vector<State>> precomputed_paths;

    // --- Helpers ---
    static void task_trampoline(void* params);
    void followPathImpl(const std::string& path_name, const ramseteConfig& r_config);
    
    static void precompute_paths_task(void* param);
    static std::vector<std::pair<double,double>> parse_pairs(const std::string& line);
    static std::vector<State> prepare_trajectory(const std::string& data);

    double angleError(double robotAngle, double targetAngle);
    double sinc(double x);

    class Vector2 {
    public:
        Vector2(float x, float y);
        std::string latex() const;
        float x, y;
    };
};