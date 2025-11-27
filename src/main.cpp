#include "main.h"
#include "Eigen/Core"
#include "Eigen/src/Core/Matrix.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include <cmath>
#include <cstdint>
#include <sstream>
#include <sys/types.h>
#include "paths.cpp"

const double INCH_TO_METER = 0.0254;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup rightMotors({11, 12, 13}, pros::v5::MotorGears::green);
pros::MotorGroup leftMotors({-18, -19, -20}, pros::v5::MotorGears::green);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors,
                              10.0, // 10 inches -> meters
                              4,     // LemLib handles this enum, but it's 4 inches
                              160, 2);

pros::IMU imu(15);

pros::Rotation horizontal_tracking_sensor(6);
pros::Rotation vertical_tracking_sensor(-16);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracking_sensor,
                                                2,   // 2 inch wheel diameter -> meters
                                                -5.03088913472, // 7.1 inch offset -> meters
                                                1);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracking_sensor,
                                              2,   // 2 inch wheel diameter -> meters
                                              0.306116918816, // 0.1 inch offset -> meters
                                              1);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// FIX: Convert lateral controller error ranges to meters
lemlib::ControllerSettings lateral_controller(10, // kP
                                              0,  // kI
                                              3,  // kD
                                              3,  // windup
                                              1,  // small error range in meters
                                              100,
                                              3, // large error range in meters
                                              500,
                                              20 // slew rate in meters/sec^2
);

// Angular controller error ranges are in degrees, so they don't need conversion
lemlib::ControllerSettings angular_controller(4,    // kP
                                              0.01, // kI
                                              20,   // kD
                                              3,    // windup
                                              1,    // small error range in degrees
                                              100,
                                              3, // large error range in degrees
                                              500, 0);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

pros::Distance right(3);
pros::Distance left(4);
pros::Distance back(2);
pros::Distance front(17);
pros::Distance front_2(9);

pros::Optical color_sensor(10);
pros::Vision vision_sensor(14);
pros::adi::AnalogIn line_sensor_left('H');
pros::adi::AnalogIn line_sensor_right('G');
pros::adi::DigitalIn bumper_sensor('A');

class Vector2 {
public:
    Vector2(float x, float y) : x(x), y(y) {}
    std::string latex() const {
        std::ostringstream oss;
        oss << "\\left(" << std::fixed << this->x << "," << std::fixed << this->y << "\\right)";
        return oss.str();
    }

    float x;
    float y;
};

int sign(double x) {
        return (x > 0) - (x < 0); // returns 1, 0, or -1
}

/** Velocity Controller Functions
*
*/
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


void initialize() {
    chassis.calibrate();
    pros::lcd::initialize();
    chassis.setPose(0,0,0);
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose(true).theta);
            /*
            Eigen::Matrix3d rotation_matrix;
            Eigen::Vector3d global_error;
            Eigen::Vector3d local_error;

            lemlib::Pose current_pose = chassis.getPose(true);
            current_pose.x *= INCH_TO_METER;
            current_pose.y *= INCH_TO_METER;
            current_pose.theta = M_PI_2 - current_pose.theta;
            double AngleError = angleError(current_pose.theta, -M_PI_2); 

            rotation_matrix << std::cos(current_pose.theta), std::sin(current_pose.theta), 0, -std::sin(current_pose.theta),
                std::cos(current_pose.theta), 0, 0, 0, 1;

            global_error << 1 - current_pose.x, 0 - current_pose.y, AngleError;
            
            local_error = rotation_matrix * global_error;
            pros::lcd::print(0, "X Error: %f", local_error(0));
            pros::lcd::print(1, "Y Error: %f", local_error(1));
            pros::lcd::print(2, "T Error: %f", local_error(2));
            */
            pros::delay(20);
        }
    });
}

inline float clamp(float val, float min_val, float max_val) {
    return std::max(min_val, std::min(max_val, val));
}


void pid_ramsete(float x_desired, float y_desired, VelocityControllerConfig &config, float time_ms)
{
    const float POS_TOLERANCE = 0.10f;   
    const float HEADING_TOLERANCE = 0.05f;  

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


    lemlib::PID pid_v(10, 0.0, 3);     
    lemlib::PID pid_w(4, 0.01, 20);      
    float k_lat = 0.5f;                

    double current_time = 0.0;
    const double dt = 10.0; 

    while (current_time < time_ms)
    {
        u_int32_t start_time_ms = pros::millis();
        lemlib::Pose current_pose = chassis.getPose(true);
        current_pose.theta = M_PI_2 - current_pose.theta;

        double cur_x = current_pose.x * INCH_TO_METER;
        double cur_y = current_pose.y * INCH_TO_METER;

        Eigen::Vector2d global_error(
            x_desired * INCH_TO_METER - cur_x,
            y_desired * INCH_TO_METER - cur_y
        );

        Eigen::Matrix2d R;
        R << cos(current_pose.theta),  sin(current_pose.theta),
            -sin(current_pose.theta),  cos(current_pose.theta);

        Eigen::Vector2d local_error = R * global_error;
        float e_x = local_error(0);
        float e_y = local_error(1);


        float e_t = atan2(e_y, e_x);  
        float vd = pid_v.update(local_error.norm() * sign(cos(e_t)));
        float w = pid_w.update(e_t) + k_lat * vd * e_y * sinc(e_t);

        float v = fabs(cos(e_t)) * vd;

        DrivetrainVoltages out = controller.update(
            v, w,
            leftMotors.get_actual_velocity() * rpm_to_mps_factor,
            rightMotors.get_actual_velocity() * rpm_to_mps_factor
        );

        // ---- SAFE VOLTAGE OUTPUT ----
        leftMotors.move_voltage(clamp(out.leftVoltage  * 1000.0f, -12000.0f, 12000.0f));
        rightMotors.move_voltage(clamp(out.rightVoltage * 1000.0f, -12000.0f, 12000.0f));

        // ---- Stop condition ----
        if (local_error.norm() < POS_TOLERANCE && fabs(e_t) < HEADING_TOLERANCE)
            break;

        current_time += dt;
        pros::Task::delay_until(&start_time_ms, dt);
    }

    leftMotors.brake();
    rightMotors.brake();
}




void autonomous() {
    
    ramsete_auton(test_config, test_path);
    //auto_tune_pid(1, 2000, test_config);
    //find_tracking_center(6, 5000);
    //chassis.setPose(0,0,0);
    //chassis.turnToPoint(24,24,10000);
    //pid_ramsete(24,24, test_config, 1000);
}

void disabled() {}

void competition_initialize() {}

void opcontrol() {
    bool brake_mode = false;
    int sig_num = 1;
    while (true) {
        double left = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double right = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        
        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        //     bool aligned = alignToGoal(sig_num, 82);
        //     if(aligned)
        //     {
        //         controller.rumble(".");
        //     }
        // }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            sig_num++;
            if(sig_num > 3)
                sig_num = 1;;
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            brake_mode = !brake_mode;
            controller.rumble(".");
        }

        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        // {
        //     detect_line();
        // }

        if(brake_mode)
        {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            chassis.tank(0,0);
            leftMotors.brake();
            rightMotors.brake();
        }
        else
        {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            chassis.tank(left, right, false);
        }
        controller.print(0, 0, "Signature#: %i", sig_num);
        pros::delay(20);
    }
}
