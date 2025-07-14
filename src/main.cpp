#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <numeric>
#include "Eigen/Core"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Motors
pros::MotorGroup rightMotors({11,12,13});
pros::MotorGroup leftMotors({-18,-19,-20});

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10.1, 4, 120, 2);

//Sensors
pros::IMU imu(15);
pros::Rotation horizontal_tracking_sensor(16);
pros::Rotation vertical_tracking_sensor(6);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracking_sensor, 2, 6.5, 1);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracking_sensor, 2, -0.1, 1);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

lemlib::ControllerSettings lateral_controller(30, 0, 100, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(4, 0, 24.5, 3, 1, 100, 3, 500, 0);

//Needs Tuning
lemlib::PID left_vel_pid(1, 0, 0);
lemlib::PID right_vel_pid(1, 0, 0);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

struct Point {
    double x;
    double y;
    double velocity;
    double angular_velocity = 0;
    double heading = 0;
    double curvature = 0;
};

double calculate_heading(double x1, double y1, double x2, double y2) {
    return atan2(y2 - y1, x2 - x1) * (180.0 / M_PI);
}

double distance(const Point& p1, const Point& p2) {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

void add_curvature_to_path(std::vector<Point>& path) {
    if (path.size() < 3) return;

    for (size_t i = 1; i < path.size() - 1; ++i) {
        Point& p0 = path[i - 1];
        Point& p1 = path[i];
        Point& p2 = path[i + 1];

        constexpr double epsilon = 1e-6;
        double area = 0.5 * std::abs(p0.x * (p1.y - p2.y) + p1.x * (p2.y - p0.y) + p2.x * (p0.y - p1.y));
        double a = distance(p0, p1);
        double b = distance(p1, p2);
        double c = distance(p2, p0);

        double denominator = 4 * area;
        if (std::abs(denominator) < epsilon) {
            p1.curvature = 0;
        } else {
            double radius = (a * b * c) / denominator;
            p1.curvature = 1.0 / radius;
        }
    }

    path[0].curvature = 0;
    path.back().curvature = 0;
}

std::vector<double> parse(const std::string& line) {
    std::stringstream ss(line);
    std::string token;
    std::vector<double> tokens;
    while (std::getline(ss, token, ',')) {
        try {
            tokens.push_back(std::stod(token));
        } catch (const std::invalid_argument& e) {
            tokens.push_back(0.0);
        }
    }
    return tokens;
}

std::vector<Point> load_path_file() {
    const double MAX_ROBOT_SPEED_IN_INCHES_PER_SEC = 25.13;

    const std::string file_content = R"(#PATH-POINTS-START Path
                                        0,0,120,0
                                        1.498,-1.326,112.02
                                        3.017,-2.627,104.041
                                        4.558,-3.9,96.061
                                        6.124,-5.145,88.081
                                        7.714,-6.357,89.701
                                        9.331,-7.535,91.32
                                        10.974,-8.674,92.939
                                        12.646,-9.772,94.558
                                        14.346,-10.825,96.177
                                        16.075,-11.83,97.796
                                        17.833,-12.783,99.415
                                        19.622,-13.678,101.034
                                        21.442,-14.505,102.654
                                        23.292,-15.266,104.273
                                        25.168,-15.958,105.892
                                        27.071,-16.572,107.511
                                        29.001,-17.098,108.472
                                        30.95,-17.543,109.432
                                        32.919,-17.893,110.393
                                        34.902,-18.151,111.354
                                        36.895,-18.313,112.314
                                        38.894,-18.379,113.275
                                        40.893,-18.349,114.236
                                        42.889,-18.228,115.197
                                        44.878,-18.014,116.157
                                        46.856,-17.722,117.118
                                        48.82,-17.343,118.079
                                        50.769,-16.896,119.039
                                        52.701,-16.381,120
                                        54.617,-15.806,120
                                        56.517,-15.182,120
                                        58.4,-14.509,120
                                        60.27,-13.8,120
                                        62.127,-13.057,120
                                        63.974,-12.29,120
                                        65.813,-11.504,120
                                        67.646,-10.704,120
                                        69.476,-9.898,120
                                        71.306,-9.09,120
                                        73.138,-8.287,120
                                        74.974,-7.495,120
                                        76.818,-6.721,120
                                        78.671,-5.967,120
                                        80.536,-5.246,120
                                        82.415,-4.559,120
                                        84.308,-3.915,120
                                        86.218,-3.322,120
                                        88.143,-2.781,120
                                        90.087,-2.309,120
                                        92.045,-1.902,120
                                        94.017,-1.574,120
                                        96.002,-1.327,120
                                        97.995,-1.165,120
                                        99.994,-1.097,120
                                        101.993,-1.118,120
                                        103.989,-1.24,120
                                        105.978,-1.451,120
                                        107.953,-1.76,120
                                        109.913,-2.159,120
                                        111.853,-2.643,120
                                        113.769,-3.217,120
                                        115.66,-3.865,120
                                        117.526,-4.585,120
                                        119.36,-5.383,120
                                        121.166,-6.242,120
                                        122.943,-7.16,120
                                        124.69,-8.132,120
                                        126.409,-9.155,120
                                        128.099,-10.224,120
                                        129.761,-11.337,120
                                        131.395,-12.489,120
                                        133.003,-13.678,120
                                        135.089,-15.294,120,0
                                        135.089,-15.294,0,0
                                        )";

    std::stringstream ss(file_content);
    std::vector<Point> points;
    std::string line;
    
    std::getline(ss, line);

    while (std::getline(ss, line)) {
        if (line.rfind("#", 0) == 0 || line.find_first_not_of(" \t\n\v\f\r") == std::string::npos) continue;
        
        auto tokens = parse(line);
        if (tokens.size() >= 3) {
            Point p;
            p.x = tokens[0];
            p.y = tokens[1];
            p.velocity = (tokens[2] / 127.0) * MAX_ROBOT_SPEED_IN_INCHES_PER_SEC;
            points.push_back(p);
        }
    }
    
    add_curvature_to_path(points);

    for (int i = 0; i < points.size(); ++i) {
        if (i + 1 < points.size()) {
            points[i].heading = calculate_heading(points[i].x, points[i].y, points[i+1].x, points[i+1].y);
        } else if (i > 0) {
            points[i].heading = points[i-1].heading;
        }
        points[i].angular_velocity = points[i].velocity * points[i].curvature;
    }

    return points;
}

int find_closest_point(const lemlib::Pose& robot_pose, const std::vector<Point>& path, int last_closest_index) {
    int closest_index = last_closest_index;
    double min_dist = std::hypot(robot_pose.x - path[last_closest_index].x, robot_pose.y - path[last_closest_index].y);

    for (int i = last_closest_index + 1; i < path.size(); ++i) {
        double dist = std::hypot(robot_pose.x - path[i].x, robot_pose.y - path[i].y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_index = i;
        }
    }
    return closest_index;
}


std::array<double,2> ramsete_controller(const std::array<double,3> &desired, const std::array<double,3> &actual,
                                        double v_d, double w_d, double zeta, double b) {
    Eigen::Matrix<double,3,1> error;
    double dx = desired[0]-actual[0], dy=desired[1]-actual[1], dtheta=desired[2]-actual[2];
    while (dtheta > M_PI) 
    {
        dtheta -= 2*M_PI;
    }
    while (dtheta < -M_PI){
        dtheta += 2*M_PI;
    }
    Eigen::Matrix<double,3,3> T {
        { cos(actual[2]), sin(actual[2]), 0 },
        {-sin(actual[2]), cos(actual[2]), 0 },
        {0,0,1}
    };
    error = T * Eigen::Matrix<double,3,1>{dx, dy, dtheta};
    double k = 2 * zeta * std::sqrt(w_d*w_d + b*v_d*v_d);
    double v = v_d * cos(error(2)) + k*error(0);
    double w;
    if (std::abs(error(2)) < 1e-9) {
        w = w_d + k*error(2);
    } else {
        w = w_d + k*error(2) + (b*v_d * std::sin(error(2))*error(1))/error(2);
    }
    return {v,w};
}

double get_drive_velocity(pros::MotorGroup& motors) {
    double avg_rpm = motors.get_actual_velocity();
    const double WHEEL_DIAMETER = 4;
    const double INCHES_PER_ROTATION = WHEEL_DIAMETER * M_PI;
    return (avg_rpm/60.0) * INCHES_PER_ROTATION;
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    pros::Task([](){
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            pros::lcd::print(0,"X: %.2f",pose.x);
            pros::lcd::print(1,"Y: %.2f",pose.y);
            pros::lcd::print(2,"Theta: %.2f",pose.theta);
            pros::delay(20);
        }
    });
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    std::vector<Point> points = load_path_file();
    if (points.empty()) {
        controller.print(0,0,"Asset empty!");
        return;
    }

    chassis.setPose(points[0].x, points[0].y, points[0].heading);
    
    // Needs tuning
    const double RAMSETE_B = 2.0;
    const double RAMSETE_ZETA = 0.7;
    const double TRACK_WIDTH = 10.1;
    const double LOOKAHEAD_DISTANCE = 6.0;

    int last_closest_point_index = 0;

    while(last_closest_point_index < points.size() - 1) {
        lemlib::Pose pose = chassis.getPose();
        std::array<double, 3> actual_state = {pose.x, pose.y, pose.theta * M_PI / 180.0};

        last_closest_point_index = find_closest_point(pose, points, last_closest_point_index);

        int lookahead_point_index = last_closest_point_index;
        while (lookahead_point_index < points.size() - 1 && distance(points[last_closest_point_index], points[lookahead_point_index]) < LOOKAHEAD_DISTANCE) {
            lookahead_point_index++;
        }

        Point& target = points[lookahead_point_index];
        std::array<double, 3> desired_state = {target.x, target.y, target.heading * M_PI / 180.0};

        auto [target_v, target_w] = ramsete_controller(desired_state, actual_state, target.velocity, target.angular_velocity, RAMSETE_ZETA, RAMSETE_B);

        double left_v = target_v - target_w * (TRACK_WIDTH / 2.0);
        double right_v = target_v + target_w * (TRACK_WIDTH / 2.0);

        double actual_left_v = get_drive_velocity(leftMotors);
        double actual_right_v = get_drive_velocity(rightMotors);
        double left_power = left_vel_pid.update(left_v - actual_left_v);
        double right_power = right_vel_pid.update(right_v - actual_right_v);

        leftMotors.move(std::clamp(left_power, -127.0, 127.0));
        rightMotors.move(std::clamp(right_power, -127.0, 127.0));

        pros::delay(10);
    }

    leftMotors.move(0);
    rightMotors.move(0);
}

void opcontrol() {
    /**
    while (true) {
        int right=controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int left=controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        chassis.arcade(left,right,false);
        pros::delay(20);
    }
    */
     const double TARGET_VELOCITY = 15.0;

    while (true) {
        double actual_left_v = get_drive_velocity(leftMotors);
        double actual_right_v = get_drive_velocity(rightMotors);

        double left_power = left_vel_pid.update(TARGET_VELOCITY - actual_left_v);
        double right_power = right_vel_pid.update(TARGET_VELOCITY - actual_right_v);

        leftMotors.move(std::clamp(left_power, -127.0, 127.0));
        rightMotors.move(std::clamp(right_power, -127.0, 127.0));

        controller.print(0, 0, "Target: %.2f", TARGET_VELOCITY);
        controller.print(1, 0, "Actual L: %.2f", actual_left_v);
        controller.print(2, 0, "Power L: %.2f", left_power);

        pros::delay(10);
    }
}
