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
    pros::Task commandSchedulerTask(update_loop);
    intake = new Intake(pros::Motor(1));
    CommandScheduler::registerSubsystem(intake, intake->pctCommand(0.0));
    primary.getTrigger(DIGITAL_R1)->whileTrue(intake->pctCommand(-1.0));
    primary.getTrigger(DIGITAL_R2)->toggleOnTrue(intake->pctCommand(1.0));
    primary.getTrigger(DIGITAL_A)->whileTrue(intake->pctCommand(-1.0)
                                                    ->withTimeout(300_ms)
                                                    ->andThen(intake->pctCommand(1.0)
                                                        ->withTimeout(300_ms))
                                                    ->repeatedly());
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(20);
        }
    });
}


void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Your autonomous code will go here
}

void opcontrol() {
        while (true) {
            int leftControl = primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightControl = primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

            // The 'chassis' object must be defined elsewhere
            chassis.arcade(leftControl, rightControl);

            pros::delay(20);
        }
}