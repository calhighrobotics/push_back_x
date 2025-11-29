#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "auton/autonRoutines.h"
#include "auton/autonFunctions.h"
#include "pathFollowing/velocityController.h"
#include "pathFollowing/ramsete.cpp"
#include "pathFollowing/paths.cpp"

VelocityControllerConfig config{
    12.4370890785,
    0.803031225567,
    0.664537661342,
    0.472796490892,
    0.236548087393,
    25.2621164319,
    524.703492373,
};


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
    RamsetePathFollower ramsete(config, 4.0, 0.2);
    ramsete.followPath(test_path);
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

    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        intake();
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        outtake();
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        score_longgoal();
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        score_midgoal();
    }
    else
    {
        intake_stop();
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


