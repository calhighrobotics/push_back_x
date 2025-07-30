#include "main.h"
#include "command/commandScheduler.h"
#include "command/commandController.h"
#include "globals.h" // Assuming 'chassis' is declared in here
#include "Intake.h"  // Your intake subsystem
#include "pros/misc.h"
#include "units/units.hpp"
#include "Drivetrain.h"

using namespace units;


// Global Objects
CommandController primary(pros::E_CONTROLLER_MASTER);
Intake *intake;
Drivetrain *drivetrain_control;

// Command scheduler loop
[[noreturn]] void update_loop() {
    while (true) {
        auto start_time = pros::millis();
        CommandScheduler::run();
        pros::c::task_delay_until(&start_time, 10);
    }
}


void initialize() {

    chassis.calibrate(); // calibrate sensors
    pros::Task commandSchedulerTask(update_loop);
    intake = new Intake(pros::Motor(20));
    CommandScheduler::registerSubsystem(intake, intake->pctCommand(0.0));
    CommandScheduler::registerSubsystem(drivetrain_control, drivetrain_control->driverControl(0, 0));
    primary.getTrigger(DIGITAL_L1)->whileTrue(intake->pctCommand(-1.0));
    primary.getTrigger(DIGITAL_L2)->whileTrue(intake->pctCommand(1.0));
    primary.getTrigger(DIGITAL_A)->whileTrue(intake->pctCommand(-1.0)
                                                    ->withTimeout(300_ms)
                                                    ->andThen(intake->pctCommand(1.0)
                                                        ->withTimeout(300_ms))
                                                    ->repeatedly());
    // print position to brain screen

}


void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.setPose(0,0,0);
    drivetrain_control->moveToPoint(0, 24, 1000);
    drivetrain_control->turnToPoint(90, 1000);
}

void opcontrol() {
    while(true)
    {
        drivetrain_control->Tank(primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        pros::delay(20);
    }
}