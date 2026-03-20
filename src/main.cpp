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
	ai_vision.enable_detection_types(pros::AivisionModeType::colors);
    pros::Task screen_task([&]() {
        lemlib::Pose pose{0,0,0};
        while (true) {
            console.clear();
            pose = chassis.getPose();

            //distancePose dpose = distanceReset(false);
            console.printf("X: %f\n", pose.x);
            console.printf("Y: %f\n", pose.y);
            console.printf("Theta: %f\n", pose.theta);
			std::vector<pros::AIVision::Object> objects = ai_vision.get_all_objects();
			for(const auto& obj : objects)
			{
				if(pros::AIVision::is_type(obj, pros::AivisionDetectType::color))
				{
					console.print("BALL DETECTED");
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
            
            pros::delay(10);
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
    while(true)
    {
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);


		if(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X))
		{
			followMultipleObjectsPID({1,2,3}, 5, 10000);
		}
		else
        	chassis.curvature(throttle, steer, false);
        pros::delay(10);
    }
}



