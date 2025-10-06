#include "main.h"
#include "globals.h" 
#include "pros/misc.h"

// Global Objects
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(20);
        }
    });
}

void disabled() {

}

void competition_initialize() {

}

void autonomous() {

}

void opcontrol() {
    basketExtension.extend();
    while(true)
    {
        double y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(y, x, false);

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intakeMotor.move_voltage(12000);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intakeMotor.move_voltage(-12000);
        }
        else {
            intakeMotor.brake();
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            topRoller.toggle();
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            midRollerDirection.toggle();
            midRollerHeight.toggle();
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            agitator.move_voltage(-12000);
        }
        else {
            agitator.brake();
        }


        pros::delay(10);
    }
}

//Deactivated: Long goal
//Extended: Score top goal
//Deactivate top roller for storage
//Activate storage to extend
