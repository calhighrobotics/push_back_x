#include "pros/rtos.hpp"
#include "globals.h"

void temp_warning() {
    pros::Task temp_screening([]() {
        while (true) {
            std::vector<double> left_temps = leftMotors.get_temperature_all();
            std::vector<double> right_temps = rightMotors.get_temperature_all();

            bool is_overheating = false; 

            const double TEMP_LIMIT = 55.0; 
            for (int i = 0; i < left_temps.size(); i++) {
                if (left_temps[i] > TEMP_LIMIT) { 
                    controller.rumble(".-.");
                    controller.clear_line(0);
                    controller.print(0, 0, "OVERHEAT! L%i", i + 1);
                    pros::delay(1000); 
                    controller.clear_line(0);
                    is_overheating = true;
                    break; 
                }
            }

            if (is_overheating) {
                pros::delay(10000); 
                return; 
            }

            for (int i = 0; i < right_temps.size(); i++) {
                if (right_temps[i] > TEMP_LIMIT) { 
                    controller.rumble(".-.");
                    controller.clear_line(0);
                    controller.print(0, 0, "OVERHEAT! R%i", i + 1);
                    pros::delay(1000); 
                    controller.clear_line(0);
                    is_overheating = true;
                    break; 
                }
            }
            pros::delay(200);
            if(is_overheating) {
                pros::delay(10000);
                return;
            }
        }
    });
}

void motor_disconnect_warning()
{
    pros::Task disconnect_screening([]() {
        std::vector<pros::Motor> all_motors = pros::Motor::get_all_devices();
        std::vector<unsigned char> disconnected;
        std::vector<unsigned char> last_disconnected;
        bool is_notified = false;
        uint32_t now = pros::millis();

        constexpr int TASK_DELAY_MILLIS = 1000;
        constexpr int CONTROLLER_DELAY_MILLIS = 50;
        while (true) {
            std::string disc_motors = "MD: ";
            bool are_motors_disconnected = false;

            for (pros::Motor i : all_motors) {
                if (!i.is_installed()) {
                    unsigned char port = i.get_port();
                    disconnected.push_back(port);
                    disc_motors = disc_motors + " " + std::to_string(port);
                    are_motors_disconnected = true;
                }
            }
            if (are_motors_disconnected) {
                controller.clear_line(0);
                pros::delay(CONTROLLER_DELAY_MILLIS);
                controller.set_text(0, 0, disc_motors);
                is_notified = true;
                pros::delay(CONTROLLER_DELAY_MILLIS);
            }
            if (disconnected.size() > last_disconnected.size()) {
                controller.rumble(". . .");
            } else if (disconnected.size() < last_disconnected.size()) {
                controller.rumble("-");
            } else if (!are_motors_disconnected && is_notified) {
                controller.clear_line(0);
                is_notified = false;
            }
            last_disconnected = disconnected;
            disconnected.clear();
            pros::delay(300);
        }
    });
}

void distance_sensor_disconnect_warning()
{
    pros::Task disconnect_screening([]() {
        std::vector<pros::Distance> all_distance_sensors = {frontDistance, backDistance, leftDistance, rightDistance};
        std::vector<unsigned char> disconnected;
        std::vector<unsigned char> last_disconnected;
        bool is_notified = false;

        constexpr int TASK_DELAY_MILLIS = 1000;
        constexpr int CONTROLLER_DELAY_MILLIS = 50;
        while (true) {
            std::string disc_d_sensors = "DSD: ";
            bool are_distance_sensors_disconnected = false;

            for (pros::Distance i : all_distance_sensors) {
                if (!i.is_installed()) {
                    unsigned char port = i.get_port();
                    disconnected.push_back(port);
                    disc_d_sensors = disc_d_sensors + " " + std::to_string(port);
                    are_distance_sensors_disconnected = true;
                }
            }
            if (are_distance_sensors_disconnected) {
                controller.clear_line(0);
                pros::delay(CONTROLLER_DELAY_MILLIS);
                controller.set_text(0, 0, disc_d_sensors);
                is_notified = true;
                pros::delay(CONTROLLER_DELAY_MILLIS);
            }
            if (disconnected.size() > last_disconnected.size()) {
                controller.rumble(". . .");
            } else if (disconnected.size() < last_disconnected.size()) {
                controller.rumble("-");
            } else if (!are_distance_sensors_disconnected && is_notified) {
                controller.clear_line(0);
                is_notified = false;
            }
            last_disconnected = disconnected;
            disconnected.clear();
            pros::delay(300);
        }
    });
}
