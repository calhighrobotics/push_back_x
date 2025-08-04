#include "main.h"
#include "globals.h" 
#include "subsystems/Intake.h"
#include "subsystems/Pneumatics.h"
#include "subsystems/Drivetrain.h"

Intake* intake = new Intake(&intakeMotor);
Pneumatics* piston = new Pneumatics('A');
Drivetrain* drivetrain = new Drivetrain(&chassis);


// Global Objects
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "Piston State: %s", piston->getState() ? "Extended" : "Retracted"); 
            pros::delay(20);
        }
    });
}

void disabled() {

}

void competition_initialize() {

}

void autonomous() {
    intake->auton_run_async(1.0);
    piston->auton_run_async(true);
    drivetrain->drive_to_async(0,24,1000);
    drivetrain->wait_for_completion();
    intake->end();
    piston->end();
    drivetrain->end();
}

void opcontrol() {
    while(true)
    {
        drivetrain->run();
        intake->run();
        piston->run();
        pros::delay(10);
    }

}