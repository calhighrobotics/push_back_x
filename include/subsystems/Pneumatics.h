#include "globals.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"

class Pneumatics {
    private:
        pros::adi::DigitalOut* piston;
        pros::Task* auton_task;
        bool state = false;
        bool running = false;

    public:
        explicit Pneumatics(char port)
        {
            this->piston = new pros::adi::DigitalOut(port);
        }

        void run()
        {
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            {
                this->retract();
            }
            else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            {
                this->extend();
            }
        }
        
        void extend()
        {
            if(!state)
                piston->set_value(true);
            state = true;
        }

        void retract()
        {
            if(state)
                piston->set_value(false);
            state = false;
        }

        bool getState()
        {
            return state;
        }

       void auton_run_async(bool desired_state = false) {
            running = true;
            if (auton_task != nullptr) {
                delete auton_task;  // Clean up old task
            }
            auton_task = new pros::Task([this, desired_state]() {
                while(running)
                {
                    if(desired_state)
                    {
                        if(!state)
                        {
                            this->extend();
                        }
                    }
                    else { if(state) { this->retract(); } }
                    pros::delay(10);
                }
            });
        }

        void end() {
            running = false;
            if (auton_task != nullptr) {
                auton_task->remove();
                delete auton_task;
                auton_task = nullptr;
            }
        }
     
};

