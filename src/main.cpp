#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "autonRoutines.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "autonFunctions.h"


rd::Selector selector({
    {"Right", right_auton},
    {"Left", left_auton},
    {"Carry", carry_auton},
    {"Elim", elim_auton},
    {"AWP", awp_auton}
});


rd::Console console;

// Global Objects
void initialize() {
    chassis.calibrate();
    pros::Task screen_task([&]() {
        while (true) {
            console.clear();
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
    //chassis.setPose(0,0,0);
    //chassis.turnToHeading(90, 1000);
    //chassis.moveToPoint(0,48,10000);
}

void opcontrol() {
    while(true)
    {

    double throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // Port 7
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        intakeMotor.move_voltage(12000);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intakeMotor.move_voltage(-12000);
    }
    else {
        intakeMotor.move_voltage(0);
    }
    
    // Port 18
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        topMotor.move_voltage(12000);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        topMotor.move_voltage(-12000);
    }
    else
    {
        topMotor.move_voltage(0);
    }

    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    {
        A.toggle();
    }
    else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        B.toggle();
    }
    else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
    {
        C.toggle();
    }


    chassis.curvature(throttle, steer, false);
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


