#include "main.h"
#include "globals.h" 
#include "pros/misc.h"


void auton1()
{
    chassis.moveToPoint(0, 12, 1000);
}

void auton2()
{
    chassis.turnToHeading(90, 1000);
}

void auton3()
{
    chassis.moveToPoint(0, -12, 1000);
}


rd::Selector selector({
    {"Test auton 1", auton1},
    {"Test auton 2", auton2},
    {"Test auton 3", auton3},
});

rd::Console console;

// Global Objects
void initialize() {
    chassis.calibrate();
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
    basketExtension.extend();
    while(true)
    {
        double y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(y, x, false);

        //R1 - outake long goal
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            agitator.move_voltage(-12000);
            intakeMotor.move_voltage(12000);
            if(midRollerDirection.is_extended())
            {
                midRollerDirection.retract();
            }
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            //R2 - Bottom goal
            agitator.move_voltage(-12000);
            intakeMotor.move_voltage(-12000);
            if(midRollerDirection.is_extended())
            {
                midRollerDirection.retract();
            }
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            //R2 - Bottom goal
            agitator.brake();
            intakeMotor.move_voltage(-12000);
            topRoller.retract();
            if(midRollerDirection.is_extended())
            {
                midRollerDirection.retract();
            }
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            agitator.move_voltage(-12000);
            intakeMotor.move_voltage(12000);
            if(!midRollerDirection.is_extended())
            {
                midRollerDirection.extend();
            }
        }
        else {
            intakeMotor.brake();
            agitator.brake();
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