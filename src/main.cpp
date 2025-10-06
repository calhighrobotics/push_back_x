#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "autonRoutines.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"



void auton2()
{
    chassis.turnToHeading(90, 1000);
}

void auton3()
{
    chassis.moveToPoint(0, -12, 1000);
}


rd::Selector selector({
    {"Red Right", red_right},
    {"Test auton 2", auton2},
    {"Test auton 3", auton3},
});

rd::Console console;

// Global Objects
void initialize() {
    chassis.calibrate();
    agitator.set_voltage_limit(6500);
    pros::Task screen_task([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            console.printf("X: %f\n", pose.x);
            console.printf("Y: %f\n", pose.y);
            console.printf("Theta: %f\n", pose.theta);
            pros::delay(20);
        }
    });
}

void disabled() {
    selector.focus();
}

void competition_initialize() {

}

void autonomous() {
    selector.run_auton();
}

void opcontrol() {
    while(true)
    {
        double y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.curvature(y, x, false);

        /*
        double right = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        double left = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        chassis.tank(left, right, false);
        */

        //R1 - outake long goal
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            midMotor.move_voltage(-12000);
            agitator.move_voltage(-6500);
            intakeMotor.move_voltage(12000);
 
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            //R2 - Bottom goal
            midMotor.move_voltage(12000);
            agitator.move_voltage(-6500);
            intakeMotor.move_voltage(-12000);

        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            //Intake
            if(midRollerHeight.is_extended())
            {
                midRollerHeight.retract();
            }
            agitator.brake();
            intakeMotor.move_voltage(12000);
            topRoller.retract();
            midMotor.move_voltage(-12000);

        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            agitator.move_voltage(-6500);
            intakeMotor.move_voltage(12000);
            midMotor.move_voltage(12000);
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        {   
            aligner.toggle();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {   
            matchload_mech.toggle();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {   
            topRoller.toggle();
        }
        else {
            intakeMotor.brake();
            agitator.brake();
            midMotor.brake();
            if(!midRollerHeight.is_extended())
            {
                midRollerHeight.extend();
            }
        }
        pros::delay(20);
    }
}

//Deactivated: Long goal
//Extended: Score top goal
//Deactivate top roller for storage
//Activate storage to extend

//R1 - outake long goal
//R2 - Bottom goal
//L1 - Intake into hopper
//l2 - outtake onto mid goal