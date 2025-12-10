#include "globals.h"
#include "pros/colors.h"

void calibrate_vision() {
    pros::Task calibrate_vision([]() {
        pros::vision_signature_s_t SIG_1 = pros::Vision::signature_from_utility(1, 2283, 6317, 4300, -5329, -4553, -4941, 5.3, 0);
        pros::vision_signature_s_t SIG_2 = pros::Vision::signature_from_utility(2, -4793, -4177, -4485, 1449, 5079, 3264, 5.0, 0);
        pros::vision_signature_s_t SIG_3 = pros::Vision::signature_from_utility(3, 4349, 11213, 7781, 261, 1081, 671, 1.7, 0);
        vision_sensor.set_signature(1, &SIG_1);
        vision_sensor.set_signature(2, &SIG_2);
        vision_sensor.set_signature(3, &SIG_3);
        vision_sensor.set_auto_white_balance(true);
        pros::delay(1500); 
        int wb_value = vision_sensor.get_white_balance();
        vision_sensor.set_auto_white_balance(false);
        vision_sensor.set_white_balance(wb_value);
        pros::Task::current().remove();
    });
}

bool alignToGoal(int SIG_NUM, int exposure) {
    lemlib::PID aligner_pid(0.2, 0, 0);
    const int CENTER_X = 158;
    
    const int requiredFrames = 10;
    int alignedFrames = 0;
    bool aligned = false;
    double time = 0;

    vision_sensor.clear_led();
    while (time < 1000 && !controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        pros::vision_object_s_t goal = vision_sensor.get_by_sig(0, SIG_NUM);

        if (goal.width > 10) { 
            int error = -CENTER_X + goal.x_middle_coord;
            float output = aligner_pid.update((float)error);

            if (std::abs(error) < requiredFrames) {
                alignedFrames++;
                leftMotors.move_velocity(0);
                rightMotors.move_velocity(0);
                vision_sensor.set_led(pros::c::COLOR_GREEN);

                if (alignedFrames > requiredFrames) {
                    aligned = true;
                    break;
                }
            } else {
                alignedFrames = 0;
                leftMotors.move_velocity(output);
                rightMotors.move_velocity(-output);
                vision_sensor.set_led(pros::c::COLOR_BLUE);
            }
        } else {
            alignedFrames = 0;
            vision_sensor.set_led(pros::c::COLOR_RED);
            break;
        }
        time += 20;
        pros::delay(20);
    }

    aligner_pid.reset();
    leftMotors.brake();
    rightMotors.brake();
    return aligned;
}
