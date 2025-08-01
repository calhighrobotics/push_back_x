#include "main.h"
#include "command/commandScheduler.h"
#include "command/commandController.h"
#include "command/parallelCommandGroup.h"
#include "globals.h" // Assuming 'chassis' is declared in here
#include "Intake.h"  // Your intake subsystem
#include "pros/misc.h"
#include "units/units.hpp"
#include "Drivetrain.h"
#include <vector>

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
    intake = new Intake(pros::Motor(6));
    drivetrain_control = new Drivetrain(chassis);
    CommandScheduler::registerSubsystem(intake, intake->pctCommand(0.0));
    CommandScheduler::registerSubsystem(drivetrain_control, drivetrain_control->driverControl(0, 0));
    primary.getTrigger(DIGITAL_L1)->whileTrue(intake->pctCommand(-1.0));
    primary.getTrigger(DIGITAL_L2)->whileTrue(intake->pctCommand(1.0));
    primary.getTrigger(DIGITAL_A)->whileTrue(intake->pctCommand(-1.0)
                                                    ->withTimeout(300_ms)
                                                    ->andThen(intake->pctCommand(1.0)
                                                        ->withTimeout(300_ms))
                                                    ->repeatedly());
    pros::delay(20);

}


void disabled() {}

void competition_initialize() {}

void autonomous() {
}

void opcontrol() {
        while(true)
        {
            drivetrain_control->driverControl(primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        }
}