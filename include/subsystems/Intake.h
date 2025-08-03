#include "globals.h"
#include "pros/misc.h"

class Intake {
    private:
        pros::Motor* intake_motor;
        pros::Task* auton_task = nullptr;
        bool running = false;

    public:
        explicit Intake(pros::Motor* intakeMotor)
        {
            this->intake_motor = intakeMotor;                
        }
    
        void run() {
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            {
                intake_motor->move_voltage(-12000);
            }
            else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
            {
                intake_motor->move_voltage(12000);
            }
            else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
            {
                intake_motor->move_voltage(12000);
                pros::delay(300);
                intake_motor->move_voltage(-12000);
                pros::delay(300);
            }
            else
            {
                intake_motor->move_voltage(0);
            }
        }
        void auton_run_async(const double pct = 0) {
            running = true;
            if (auton_task != nullptr) {
                delete auton_task;  // Clean up old task
            }
            auton_task = new pros::Task([this, pct]() {
                while(running)
                {
                    intake_motor->move_voltage(pct * 12000);
                    pros::delay(10);
                }
                intake_motor->move_voltage(0);
            });
        }

        void end() {
            running = false;
            if (auton_task != nullptr) {
                auton_task->remove();
                delete auton_task;
                auton_task = nullptr;
            }
            intake_motor->move_voltage(0);
        }

        
        
};

