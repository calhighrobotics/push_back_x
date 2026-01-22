#include "globals.h"
#include "pros/colors.h"
#include "globals.h"
#include "velocityController.h"
#include <algorithm>
#include <cmath>

void calibrate_vision() {
    pros::Task calibrate_vision([]() {
        pros::vision_signature_s_t SIG_1 = pros::Vision::signature_from_utility(1, 2283, 6317, 4300, -5329, -4553, -4941, 5.3, 0);
        vision_sensor.set_signature(1, &SIG_1);
        vision_sensor.set_auto_white_balance(true);
        pros::delay(1500); 
        int wb_value = vision_sensor.get_white_balance();
        vision_sensor.set_auto_white_balance(false);
        vision_sensor.set_white_balance(wb_value);
        pros::Task::current().remove();
    });
}

static constexpr float INCH_TO_METER = 0.0254f;
static constexpr float TRACK_WIDTH_M = 12.8f * INCH_TO_METER; 
static constexpr float wheel_circumference = (float)3.25 * M_PI * INCH_TO_METER;
static constexpr float gear_ratio = 4.0f / 3.0f;
static constexpr float rpm_to_mps_factor = (wheel_circumference / gear_ratio) / 60.0f;

VoltageController driveController(
    5.8432642308, 0.213526937516, 1.14429410811, 0.906356177095, 
    0.347072436421, 11.4953431776, 54.5797495382, 0.05, TRACK_WIDTH_M
);

bool alignToGoal(int SIG_NUM) {
    const float VISION_kP = 0.012f;
    const float MAX_W = 3.0f;
    const int CENTER_X = 158;
    const int ERROR_THRESHOLD = 5;
    const int FRAMES_TO_SETTLE = 12;
    
    int aligned_counter = 0;
    int lost_frame_counter = 0;
    float time_elapsed = 0;
    bool is_aligned = false;

    vision_sensor.clear_led();

    while (time_elapsed < 3000 && !controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        
        pros::vision_object_s_t goal = vision_sensor.get_by_sig(0, SIG_NUM);
        double mLeft = leftMotors.get_actual_velocity() * rpm_to_mps_factor;
        double mRight = rightMotors.get_actual_velocity() * rpm_to_mps_factor;

        double targetW = 0;

        if (goal.width > 15) {
            lost_frame_counter = 0;
            
            int error = -CENTER_X + goal.x_middle_coord;

            if (std::abs(error) <= ERROR_THRESHOLD) {
                aligned_counter++;
                vision_sensor.set_led(pros::c::COLOR_GREEN);
                targetW = 0;
                
                if (aligned_counter > FRAMES_TO_SETTLE) {
                    is_aligned = true;
                    break;
                }
            } else {
                aligned_counter = 0;
                vision_sensor.set_led(pros::c::COLOR_BLUE);
                targetW = error * VISION_kP;
                targetW = std::clamp((float)targetW, -MAX_W, MAX_W);
            }
        } else {
            lost_frame_counter++;
            if (lost_frame_counter > 3) {
                vision_sensor.set_led(pros::c::COLOR_RED);
                targetW = 0; 
                aligned_counter = 0;
            }
        }

        DrivetrainVoltages volts = driveController.update(0.0, targetW, mLeft, mRight);

        leftMotors.move_voltage(volts.leftVoltage);
        rightMotors.move_voltage(volts.rightVoltage);

        pros::delay(10); 
        time_elapsed += 10;
    }

    leftMotors.brake();
    rightMotors.brake();
    return is_aligned;
}
