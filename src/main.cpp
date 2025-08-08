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
    while(true)
    {
        drivetrain->run();
        intake->run();
        piston->run();
        pros::delay(10);
    }

}