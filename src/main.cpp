#include "main.h"
#include "command/commandScheduler.h"
#include "command/commandController.h"
#include "globals.h" // Assuming 'chassis' is declared in here
#include "Intake.h"  // Your intake subsystem
#include "units/units.hpp"

using namespace units;


// Global Objects
CommandController primary(pros::E_CONTROLLER_MASTER);
Intake *intake;

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
    // Your autonomous code will go here
}

void opcontrol() {

}