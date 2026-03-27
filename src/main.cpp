#include "main.h"
#include "globals.h" 
#include "liblvgl/core/lv_obj_pos.h"
#include "pros/ai_vision.hpp"
#include "pros/misc.h"
#include "robodash/views/selector.hpp"
#include <string>
#include "visionAlignment.h"

rd::Console console;

void initialize() {
    chassis.calibrate();
    /*
   chassis.setPose(-63.5, -18.5, 180);
   MCL::StartMCL(-63.5, -18.5, 180);
   pros::Task mclTask(MCL::MonteCarlo);
    */

    /*
    chassis.setPose(-48.5, -54.56, 270);
    console.focus();
    
    */
    pros::AIVision ai_vision(6);
	ai_vision.enable_detection_types(pros::AivisionModeType::colors);
    ai_vision.start_awb();
    
    pros::Task screen_task([&]() {
        lemlib::Pose pose{0,0,0};
        while (true) {
            console.clear();
            pose = chassis.getPose();

            //distancePose dpose = distanceReset(false);
            console.printf("X: %f\n", pose.x);
            console.printf("Y: %f\n", pose.y);
            console.printf("Theta: %f\n", pose.theta);
            console.printf("Object Count: %d\n", ai_vision.get_object_count());
            int32_t object_count = ai_vision.get_object_count();
            for(int i = 0; i < object_count; i++)
            {
                pros::AIVision::Object object = ai_vision.get_object(1);
                if(pros::AIVision::is_type(object, pros::AivisionDetectType::color))
                {
                    std::cout << object.id << " ," << object.object.color.width << std::endl;
                }
            }
            
            /*
            console.printf("D X: %f\n", dpose.x);
            console.printf("D Y: %f\n", dpose.y);
            console.printf("Using X: %d\n", dpose.using_odom_x);
            console.printf("Using Y: %d\n", dpose.using_odom_y);
            */
            
            
            //console.printf("X MCL: %f\n", MCL::X);
            //console.printf("Y MCL: %f\n", MCL::Y);
            pros::delay(50);
        }
    });
}

void disabled() {

}

void competition_initialize() {
}

void autonomous() {
    //selector.run_auton();
    //skills_auton();
    //awp_auton();
	followMultipleObjectsPID({1}, 5, 10000);
}

/*

Straight:
KV = 5.19338427813
1.26552223944
0.67257433253
11.6978629947
46.9504105993

Turn:
7.22576300573
1.36207946996
1.54163120695
19.8980417469
106.56124453

*/

void opcontrol() {
    bool mloader_state = false;
	bool chicken_wing_state = false;
	bool indexer_state = true;


    mloader.set_value(false);
	chicken_wing.set_value(false);
	indexer.set_value(true);
	extender.set_value(true);
	odom_lifter.set_value(true);
    while(true)
    {
    chassis.arcade(controller.get_analog(ANALOG_LEFT_Y), controller.get_analog(ANALOG_RIGHT_X), false);
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
				{
					chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
				}
				else
				{
					chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
				}

				if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
				{
					intake_motor.move_voltage(-12000);
					hood_motor.move_voltage(-12000);
				}
				else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
				{
					intake_motor.move_voltage(-12000);
					hood_motor.move_voltage(1000);
				}

				else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
				{
					intake_motor.move_voltage(12000);
					hood_motor.move_voltage(12000);
				}
				else
				{
					intake_motor.move_voltage(0);
					hood_motor.move_voltage(0);
                }

                if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
                {
                    mloader_state = !mloader_state;
                    mloader.set_value(mloader_state);
                }

                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
				{
					chicken_wing.set_value(false);
				}
				else{
					chicken_wing.set_value(true);
				}

                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
				{
					indexer.set_value(!indexer_state);
					indexer_state = !indexer_state;
				}

				if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
				{
					chassis.turnToHeading(45, 2000);
					while (chassis.isInMotion()) {
						pros::delay(20);
					}
				}
				else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
				{
					chassis.turnToHeading(135, 2000);
					while (chassis.isInMotion()) {
						pros::delay(20);
					}
				}
				else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
				{
					chassis.turnToHeading(315, 2000);
					while (chassis.isInMotion()) {
						pros::delay(20);
					}
				}
				else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
				{
					chassis.turnToHeading(225, 750);
					while (chassis.isInMotion()) {
						pros::delay(20);
					}
				}
				else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
				{
					chassis.turnToHeading(90, 750);
					while (chassis.isInMotion()) {
						pros::delay(20);
					}
				}
				else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
				{
					chassis.turnToHeading(180, 750);
					while (chassis.isInMotion()) {
						pros::delay(20);
					}
				}
				else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
				{
					chassis.turnToHeading(0, 750);
					while (chassis.isInMotion()) {
						pros::delay(20);
					}
				}
				else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
				{
					chassis.turnToHeading(270, 750);
					while (chassis.isInMotion()) {
						pros::delay(20);
					}
				}
				// {
				// 	odom_lifter.set_value(false);
				// }

				// if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
				// {
				// 	puncher.set_value(true); // fire puncher
				// 	mloader.set_value(false); // retract mloader to avoid jamming
				// 	mloader_state = false;
				// }

				//button_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
				//button_pressed2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

				pros::delay(20);
            }
}



