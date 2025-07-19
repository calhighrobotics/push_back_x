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
    // Start the command scheduler task
    pros::Task commandSchedulerTask(update_loop);

    // Initialize the intake subsystem on port 5
    intake = new Intake(pros::Motor(5));
    // Register the intake and set its default command to stop
    CommandScheduler::registerSubsystem(intake, intake->pctCommand(0.0));

    // --- Define Controller Triggers ---
        // Set pctCommand to run while R1 is true
    primary.getTrigger(DIGITAL_R1)->whileTrue(intake->pctCommand(-1.0));

    // Toggle pctCommand to run while R1 turns to ture
    primary.getTrigger(DIGITAL_R2)->toggleOnTrue(intake->pctCommand(1.0));

    // Dejam mode, causes the intake to move back and forth quickly
    primary.getTrigger(DIGITAL_A)->whileTrue(intake->pctCommand(-1.0)
                                                ->withTimeout(300.0_ms)
                                                ->andThen(intake->pctCommand(1.0)
                                                    ->withTimeout(300.0_ms))
                                                ->repeatedly());

    // --- Initialize Chassis and Screen Task ---
    pros::lcd::initialize();
    // The 'chassis' object must be defined in another file like globals.cpp
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