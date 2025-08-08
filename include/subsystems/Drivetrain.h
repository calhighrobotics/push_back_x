#include "globals.h"
#include "pros/misc.h"

class Drivetrain {
    private:
        lemlib::Chassis* chassis;
        pros::Task* auton_task = nullptr;
        bool running = false;

    public:
        explicit Drivetrain(lemlib::Chassis* chassis) {
            this->chassis = chassis;
        }

        void run() {
            chassis->arcade(
                controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)
            );
        }

        void drive_to(const float x = 0, const float y = 0, const float timeout = 0) {
            chassis->moveToPoint(x, y, timeout);
        }

        void drive_to_async(float x = 0, float y = 0, float timeout = 0) {
            end(); 
            auton_task = new pros::Task([this, x, y, timeout]() {
                chassis->moveToPoint(x, y, timeout);
            });
        }

        lemlib::Pose getPose()
        {
            return chassis->getPose();
        }    

        void drive_voltage(double left_pct, double right_pct, int duration_ms = 1000) {
            end(); 
            running = true;
            auton_task = new pros::Task([this, left_pct, right_pct, duration_ms]() {
                leftMotors.move_voltage(left_pct * 12000);
                rightMotors.move_voltage(right_pct * 12000);
                pros::delay(duration_ms);
                leftMotors.move_voltage(0);
                rightMotors.move_voltage(0);
            });
        }

        void end() {
            running = false;
            if (auton_task != nullptr) {
                auton_task->remove();
                delete auton_task;
                auton_task = nullptr;
            }
            leftMotors.move_voltage(0);
            rightMotors.move_voltage(0);
        }

        void wait_for_completion() {
            if (auton_task != nullptr) {
                auton_task->join(); 
                delete auton_task;
                auton_task = nullptr;
            }
}
};


