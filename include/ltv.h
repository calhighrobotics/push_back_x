#pragma once

#include "Eigen/Dense"
#include "velocityController.h" 
#include "lemlib/api.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <atomic>
#include <memory>
#include <algorithm>
#include "globals.h" 
#include "pros/rtos.hpp"

class LTVPathFollower {
public:

    struct ltvConfig {
        bool backwards = false;
        bool log = false;
        int path_index = -1;
        bool test = false;
        bool turnFirst = false;
        bool end_correction = false;
        float mpose_lead = 0.6f;
        float track_width = 11.55f;
        float max_lin_correction = 2.0f;
        float max_ang_correction = 10.0f;
        int exit_points = 5;

        float max_velocity = 1.5f; 
        float max_acceleration = 2.0f;
        
        float q_x_forward = 400.0f;
        float q_y_forward = 256.0f * 3;
        float q_theta_forward = 32.82f;
        float r_ang_forward = 0.25f;
        float r_vel_forward = 1.0f;
        float q_x_backward = 1300.0f; 
        float q_y_backward = 90000.0f; 
        float q_theta_backward = 10000.0f; 
        float r_ang_backward = 135.0f;
        float r_vel_backward = 190.0f;
    };

    LTVPathFollower(const VelocityControllerConfig& config);
    void followPath(const std::string& path_name, const ltvConfig& l_config);

    double getPathLength(const std::string& path_name);
    void waitUntilDone();
    void waitUntil(float dist_inches);
    void waitUntil(float x_inch, float y_inch, float radius_inch = 2.0f);
    void cancel();
    bool isRunning();
    void precompute_paths(const std::vector<std::string>& path_names);

private:
    static constexpr float INCH_TO_METER = 0.0254f;
    static constexpr float METER_TO_INCH = 39.3700787f;
    
    static constexpr float wheel_circumference = (float)lemlib::Omniwheel::NEW_325 * M_PI * INCH_TO_METER;
    static constexpr float gear_ratio = 4.0f / 3.0f;
    static constexpr float rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60.0f;

    VoltageController controller;

    // Task management
    pros::Task* task = nullptr;
    std::atomic<bool> is_running {false};
    std::atomic<bool> cancel_request {false};
    std::atomic<float> distance_traveled_inches {0.0f};

    struct TaskParams {
        LTVPathFollower* instance;
        std::string path_name;
        ltvConfig config;
        std::vector<State> dynamic_path; 
    };

    static void task_trampoline(void* params);
    void followPathImpl(const std::string& path_name, const ltvConfig& l_config, const std::vector<State>& dynamic_path = {});
    static inline std::vector<std::vector<State>> precomputed_paths;
    Eigen::MatrixXf dareSolver(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R);
    std::pair<Eigen::MatrixXf, Eigen::MatrixXf> discretizeAB(const Eigen::MatrixXf& contA, const Eigen::MatrixXf& contB, double dtSeconds);
    static void precompute_paths_task(void* param);
    static std::vector<std::pair<double,double>> parse_pairs(const std::string& line);
    static std::vector<State> prepare_trajectory(const std::string& data);
    static double angleError(double robotAngle, double targetAngle);
    static double clamp(double value, double min, double max);

    class Vector2 {
    public:
        Vector2(float x, float y);
        std::string latex() const;
        float x, y;
    };
};