// const double MM_TO_IN = 0.0393701;
// const double FIELD_WIDTH = 3657.6 * MM_TO_IN;  // 144 inches
// const double FIELD_HEIGHT = 3657.6 * MM_TO_IN; // 144 inches
// const double HALF_WIDTH = FIELD_WIDTH / 2.0;   // 72 inches
// const double HALF_HEIGHT = FIELD_HEIGHT / 2.0; // 72 inches
// const double MAX_SENSOR_RANGE = 1800 * MM_TO_IN; // ~78.7 inches
// const double MIN_SENSOR_RANGE = 50 * MM_TO_IN;   // ~2 inches

// bool controller_screen_avilable;

// struct SensorConfig {
//     double forward_offset; // inches
//     double strafe_offset;  // inches
//     double mounting_angle; // degrees
// };

// const SensorConfig front_sensor_cfg = {9, 0.25, 0};   
// const SensorConfig left_sensor_cfg  = {-1.25, -5.5, 90};   
// const SensorConfig right_sensor_cfg = {-1.25, 5.5, -90};  
// const SensorConfig back_sensor_cfg  = {-7.75, -0.25, 180}; 

// struct SensorReadings {
//     double dist_mm;
//     int object_size;
//     int confidence;
// };

// struct distancePose {
//     double x;            // Estimated global X
//     double y;            // Estimated global Y
//     bool using_odom_x;   // True if X is from odometry (no sensor lock)
//     bool using_odom_y;   // True if Y from odometry (no sensor lock)
// };

// /*
// This function doesn't work at 45 degrees or other denominations of (pi/2, 3pi/2, 5pi/2 or 7pi/2)
// for reliability.
// */
// distancePose calculateGlobalPosition(
//     const SensorReadings& front_data,
//     const SensorReadings& left_data,
//     const SensorReadings& right_data,
//     const SensorReadings& back_data,
//     double heading_deg)
// {
//     lemlib::Pose current_pose = chassis.getPose();
//     double est_x = current_pose.x;
//     double est_y = current_pose.y;

//     bool using_odom_x = true;
//     bool using_odom_y = true;

//     struct SensorData { SensorConfig cfg; double dist_in; int confidence;};
//     const SensorData sensors[] = {
//         {front_sensor_cfg, front_data.dist_mm * MM_TO_IN, front.get_confidence()},
//         {left_sensor_cfg,  left_data.dist_mm * MM_TO_IN, left.get_confidence()},
//         {right_sensor_cfg, right_data.dist_mm * MM_TO_IN, right.get_confidence()},
//         {back_sensor_cfg,  back_data.dist_mm * MM_TO_IN, back.get_confidence()}
//     };
//     auto is_valid = [&](int i) {
//         return (sensors[i].dist_in < MAX_SENSOR_RANGE && 
//                 sensors[i].dist_in >= MIN_SENSOR_RANGE &&
//                 sensors[i].confidence > 20);
//     };

//     double norm_heading = std::fmod(heading_deg, 360.0);
//     if (norm_heading < 0) norm_heading += 360.0;
//     const double TOLERANCE = 40.0; 
    
//     std::vector<double> x_cands;
//     std::vector<double> y_cands;

//     auto get_global_offsets = [&](const SensorConfig& cfg, double heading_rad) {
//         double cos_h = std::cos(heading_rad);
//         double sin_h = std::sin(heading_rad);
//         double global_offset_x = (cfg.forward_offset * sin_h) + (cfg.strafe_offset * cos_h);
//         double global_offset_y = (cfg.forward_offset * cos_h) - (cfg.strafe_offset * sin_h);
        
//         return std::make_pair(global_offset_x, global_offset_y);
//     };

//     if (norm_heading <= TOLERANCE || norm_heading >= 360.0 - TOLERANCE) {
//         double angle_off_rad = (norm_heading <= TOLERANCE) ? 
//             lemlib::degToRad(norm_heading) : lemlib::degToRad(norm_heading - 360.0);
//         double heading_rad = angle_off_rad;

//         double perp_dist_0 = sensors[0].dist_in * std::cos(angle_off_rad);
//         double perp_dist_3 = sensors[3].dist_in * std::cos(angle_off_rad);
//         double perp_dist_1 = sensors[1].dist_in * std::cos(angle_off_rad);
//         double perp_dist_2 = sensors[2].dist_in * std::cos(angle_off_rad);

//         auto offset_0 = get_global_offsets(sensors[0].cfg, heading_rad);
//         auto offset_1 = get_global_offsets(sensors[1].cfg, heading_rad);
//         auto offset_2 = get_global_offsets(sensors[2].cfg, heading_rad);
//         auto offset_3 = get_global_offsets(sensors[3].cfg, heading_rad);

//         if (is_valid(0)) { y_cands.push_back(HALF_HEIGHT - perp_dist_0 - offset_0.second); }
//         if (is_valid(3)) { y_cands.push_back(-HALF_HEIGHT + perp_dist_3 - offset_3.second); }
//         if (is_valid(1)) { x_cands.push_back(-HALF_WIDTH + perp_dist_1 - offset_1.first); }
//         if (is_valid(2)) { x_cands.push_back(HALF_WIDTH - perp_dist_2 - offset_2.first); }
//     }
//     // (Case 2: Facing DOWN - unchanged)
//     else if (std::fabs(norm_heading - 180.0) <= TOLERANCE) {
//         double angle_off_rad = lemlib::degToRad(norm_heading - 180.0);
//         double heading_rad = lemlib::degToRad(norm_heading);

//         double perp_dist_0 = sensors[0].dist_in * std::cos(angle_off_rad);
//         double perp_dist_3 = sensors[3].dist_in * std::cos(angle_off_rad);
//         double perp_dist_1 = sensors[1].dist_in * std::cos(angle_off_rad);
//         double perp_dist_2 = sensors[2].dist_in * std::cos(angle_off_rad);

//         auto offset_0 = get_global_offsets(sensors[0].cfg, heading_rad);
//         auto offset_1 = get_global_offsets(sensors[1].cfg, heading_rad);
//         auto offset_2 = get_global_offsets(sensors[2].cfg, heading_rad);
//         auto offset_3 = get_global_offsets(sensors[3].cfg, heading_rad);

//         if (is_valid(0)) { y_cands.push_back(-HALF_HEIGHT + perp_dist_0 - offset_0.second); }
//         if (is_valid(3)) { y_cands.push_back(HALF_HEIGHT - perp_dist_3 - offset_3.second); }
//         if (is_valid(1)) { x_cands.push_back(HALF_WIDTH - perp_dist_1 - offset_1.first); }
//         if (is_valid(2)) { x_cands.push_back(-HALF_WIDTH + perp_dist_2 - offset_2.first); }
//     }
//     // (Case 3: Facing RIGHT - unchanged)
//     else if (std::fabs(norm_heading - 90.0) <= TOLERANCE) {
//         double angle_off_rad = lemlib::degToRad(norm_heading - 90.0);
//         double heading_rad = lemlib::degToRad(norm_heading);
        
//         double perp_dist_0 = sensors[0].dist_in * std::cos(angle_off_rad);
//         double perp_dist_3 = sensors[3].dist_in * std::cos(angle_off_rad);
//         double perp_dist_1 = sensors[1].dist_in * std::cos(angle_off_rad);
//         double perp_dist_2 = sensors[2].dist_in * std::cos(angle_off_rad);

//         auto offset_0 = get_global_offsets(sensors[0].cfg, heading_rad);
//         auto offset_1 = get_global_offsets(sensors[1].cfg, heading_rad);
//         auto offset_2 = get_global_offsets(sensors[2].cfg, heading_rad);
//         auto offset_3 = get_global_offsets(sensors[3].cfg, heading_rad);

//         if (is_valid(0)) { x_cands.push_back(HALF_WIDTH - perp_dist_0 - offset_0.first); }
//         if (is_valid(3)) { x_cands.push_back(-HALF_WIDTH + perp_dist_3 - offset_3.first); }
//         if (is_valid(1)) { y_cands.push_back(HALF_HEIGHT - perp_dist_1 - offset_1.second); }
//         if (is_valid(2)) { y_cands.push_back(-HALF_HEIGHT + perp_dist_2 - offset_2.second); }
//     }
//     // (Case 4: Facing LEFT - unchanged)
//     else if (std::fabs(norm_heading - 270.0) <= TOLERANCE) {
//         double angle_off_rad = lemlib::degToRad(norm_heading - 270.0);
//         double heading_rad = lemlib::degToRad(norm_heading);

//         double perp_dist_0 = sensors[0].dist_in * std::cos(angle_off_rad);
//         double perp_dist_3 = sensors[3].dist_in * std::cos(angle_off_rad);
//         double perp_dist_1 = sensors[1].dist_in * std::cos(angle_off_rad);
//         double perp_dist_2 = sensors[2].dist_in * std::cos(angle_off_rad);
        
//         auto offset_0 = get_global_offsets(sensors[0].cfg, heading_rad);
//         auto offset_1 = get_global_offsets(sensors[1].cfg, heading_rad);
//         auto offset_2 = get_global_offsets(sensors[2].cfg, heading_rad);
//         auto offset_3 = get_global_offsets(sensors[3].cfg, heading_rad);
        
//         if (is_valid(0)) { x_cands.push_back(-HALF_WIDTH + perp_dist_0 - offset_0.first); }
//         if (is_valid(3)) { x_cands.push_back(HALF_WIDTH - perp_dist_3 - offset_3.first); }
//         if (is_valid(1)) { y_cands.push_back(-HALF_HEIGHT + perp_dist_1 - offset_1.second); }
//         if (is_valid(2)) { y_cands.push_back(HALF_HEIGHT - perp_dist_2 - offset_2.second); }
//     }

//     if (!x_cands.empty()) {
//         est_x = std::accumulate(x_cands.begin(), x_cands.end(), 0.0) / x_cands.size();
//         using_odom_x = false;
//     }

//     if (!y_cands.empty()) {
//         est_y = std::accumulate(y_cands.begin(), y_cands.end(), 0.0) / y_cands.size();
//         using_odom_y = false;
//     }


//     distancePose pose;
//     pose.x = est_x;
//     pose.y = est_y;
//     pose.using_odom_x = using_odom_x;
//     pose.using_odom_y = using_odom_y;
//     return pose;
// }

// distancePose distanceReset() {
//     // Assumes you can get the IMU heading directly from the chassis or an IMU object
//     double heading_deg = chassis.getPose().theta; // Or imu.get_rotation(), etc.

//     // Create SensorReadings structs
//     SensorReadings front_data = {(double)front.get_distance(), front.get_object_size(), front.get_confidence()};
//     SensorReadings left_data  = {(double)left.get_distance(),  left.get_object_size(),  left.get_confidence()};
//     SensorReadings right_data = {(double)right.get_distance(), right.get_object_size(), right.get_confidence()};
//     SensorReadings back_data  = {(double)back.get_distance(),  back.get_object_size(),  back.get_confidence()};

//     return calculateGlobalPosition(front_data, left_data, right_data, back_data, heading_deg);
// }


// distancePose distanceReset(bool left_use, bool right_use, bool front_use, bool back_use) {
//     double heading_deg = chassis.getPose().theta; 

//     const int invalid_dist_mm = 10000; 
//     const int invalid_confidence = 0; 

//     SensorReadings front_data = front_use
//         ? SensorReadings{(double)front.get_distance(), front.get_object_size(), front.get_confidence()}
//         : SensorReadings{invalid_dist_mm, 0, invalid_confidence};
    
//     SensorReadings left_data = left_use
//         ? SensorReadings{(double)left.get_distance(), left.get_object_size(), left.get_confidence()}
//         : SensorReadings{invalid_dist_mm, 0, invalid_confidence};

//     SensorReadings right_data = right_use
//         ? SensorReadings{(double)right.get_distance(), right.get_object_size(), right.get_confidence()}
//         : SensorReadings{invalid_dist_mm, 0, invalid_confidence};

//     SensorReadings back_data = back_use
//         ? SensorReadings{(double)back.get_distance() ,back.get_object_size(), back.get_confidence()}
//         : SensorReadings{invalid_dist_mm, 0, invalid_confidence};


//     return calculateGlobalPosition(front_data, left_data, right_data, back_data, heading_deg);
// }

// enum Color {
//     NONE = 0,
//     RED = 1,
//     BLUE = 2
// };

// int get_color() {
//     double hue = color_sensor.get_hue();
//     // return red
//     Color color = NONE;
//     if ((hue > 0 && hue < 50) || (hue > 310 && hue < 361)) {
//         color = RED;
//     }
//     // return blue
//     else if (hue > 150 && hue < 270) {
//         color = BLUE;
//     }
//     return color;
// }

// bool alignToGoal(int SIG_NUM, int exposure) {

//     lemlib::PID aligner_pid(0.2, 0,0);
//     //Center of screen
//     const int CENTER_X = 158;
  
//     int alignedFrames = 0;
//     bool aligned = false;
//     double time = 0;

//     vision_sensor.clear_led();
//     // (brightness on utility)
  
//     while (time < 5000 && !controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
//         pros::vision_object_s_t goal = vision_sensor.get_by_sig(0, SIG_NUM);

//         if (goal.width > 10) { 
//             int error = -CENTER_X + goal.x_middle_coord;
//             float output = aligner_pid.update(error);

//             if (std::abs(error) < 10) {
//                 alignedFrames++;
//                 leftMotors.move_velocity(0);
//                 rightMotors.move_velocity(0);
//                 vision_sensor.set_led(pros::c::COLOR_GREEN);

//                 if (alignedFrames > 10) {
//                     aligned = true;
//                     break;
//                 }
//             } else {
//                 alignedFrames = 0;
//                 leftMotors.move_velocity(output);
//                 rightMotors.move_velocity(-output);
//                 vision_sensor.set_led(pros::c::COLOR_BLUE);
//             }
//         } else {
//             alignedFrames = 0;
//             leftMotors.move_velocity(20);
//             rightMotors.move_velocity(-20);
//             vision_sensor.set_led(pros::c::COLOR_YELLOW);
//         }
//         time += 20;
//         pros::delay(20);
//     }

//     aligner_pid.reset();
//     leftMotors.move_velocity(0);
//     rightMotors.move_velocity(0);

//     if (!aligned)
//         vision_sensor.set_led(pros::c::COLOR_RED);
//     else if((aligned && front.get_object_size() > 5 && front.get_object_size() < 140 && front.get()*MM_TO_IN < 100) || (front_2.get_object_size() > 5 && front_2.get_object_size() < 140 && front_2.get()*MM_TO_IN < 100))
//         {
//             time = 0;
//             lemlib::PID forward_goal_pid(0.5,0,0);
//             while(time < 1000 && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
//             {
//                 double forward_error = front.get();
//                 double vel = forward_goal_pid.update(forward_error);
//                 leftMotors.move_velocity(vel);
//                 rightMotors.move_velocity(vel);
//                 if(front.get() < 300)
//                 {
//                     leftMotors.move_velocity(0);
//                     rightMotors.move_velocity(0);
//                     break;
//                 }
//                 time += 20;
//                 pros::delay(20);
//             }
//             forward_goal_pid.reset();
//         }
//     return aligned;
// }


// void temp_warning() {
//     pros::Task temp_screening([]() {
//         while (true) {
//             std::vector<double> left_temps = leftMotors.get_temperature_all();
//             std::vector<double> right_temps = rightMotors.get_temperature_all();

//             bool is_overheating = false; 

//             const double TEMP_LIMIT = 55.0; 
//             for (int i = 0; i < left_temps.size(); i++) {
//                 if (left_temps[i] > TEMP_LIMIT) { 
//                     controller.rumble(".-.");
//                     controller.clear_line(0);
//                     controller.print(0, 0, "OVERHEAT! L%i", i + 1);
//                     controller_screen_avilable = false;
//                     pros::delay(1000); 
//                     controller_screen_avilable = true;
//                     controller.clear_line(0);
//                     is_overheating = true;
//                     break; 
//                 }
//             }

//             if (is_overheating) {
//                 pros::delay(10000); 
//                 return; 
//             }

//             for (int i = 0; i < right_temps.size(); i++) {
//                 if (right_temps[i] > TEMP_LIMIT) { 
//                     controller.rumble(".-.");
//                     controller.clear_line(0);
//                     controller.print(0, 0, "OVERHEAT! R%i", i + 1);
//                     controller_screen_avilable = false;
//                     pros::delay(1000); 
//                     controller_screen_avilable = true;
//                     controller.clear_line(0);
//                     is_overheating = true;
//                     break; 
//                 }
//             }
//             pros::delay(200);
//             if(is_overheating) {
//                 pros::delay(10000);
//                 return;
//             }
//         }
//     });
// }

// void motor_disconnect_warning()
// {
//     pros::Task disconnect_screening([]() {
//         std::vector<pros::Motor> all_motors = pros::Motor::get_all_devices();
//         std::vector<unsigned char> disconnected;
//         std::vector<unsigned char> last_disconnected;
//         bool is_notified = false;
//         uint32_t now = pros::millis();

//         constexpr int TASK_DELAY_MILLIS = 1000;
//         constexpr int CONTROLLER_DELAY_MILLIS = 50;
//         while (true) {
//                 std::string disc_motors = "MD: ";
//                 bool are_motors_disconnected = false;

//                 for (pros::Motor i : all_motors) {
//                     if (!i.is_installed()) {
//                         unsigned char port = i.get_port();
//                         disconnected.push_back(port);
//                         disc_motors = disc_motors + " " + std::to_string(port);
//                         are_motors_disconnected = true;
//                     }
//                 }
//                 // Controller screen commands must be delayed by 50ms due to polling limitations
//                 if (are_motors_disconnected) {
//                     controller.clear_line(0);
//                     pros::delay(CONTROLLER_DELAY_MILLIS);
//                     controller.set_text(0, 0, disc_motors);
//                     is_notified = true;
//                     pros::delay(CONTROLLER_DELAY_MILLIS);
//                 }
//                 if (disconnected.size() > last_disconnected.size()) {
//                     controller.rumble(". . .");
//                 } else if (disconnected.size() < last_disconnected.size()) {
//                     controller.rumble("-");
//                 } else if (!are_motors_disconnected && is_notified) {
//                     controller.clear_line(0);
//                     is_notified = false;
//                 }
//                 last_disconnected = disconnected;
//                 disconnected.clear();
//             pros::delay(300);
//         }
//     });


// }

// void distance_sensor_disconnect_warning()
// {
//     pros::Task disconnect_screening([]() {
//         std::vector<pros::Distance> all_distance_sensors = {front, back, left, right};
//         std::vector<unsigned char> disconnected;
//         std::vector<unsigned char> last_disconnected;
//         bool is_notified = false;

//         constexpr int TASK_DELAY_MILLIS = 1000;
//         constexpr int CONTROLLER_DELAY_MILLIS = 50;
//         while (true) {
//                 std::string disc_d_sensors = "DSD: ";
//                 bool are_distance_sensors_disconnected = false;

//                 for (pros::Distance i : all_distance_sensors) {
//                     if (!i.is_installed()) {
//                         unsigned char port = i.get_port();
//                         disconnected.push_back(port);
//                         disc_d_sensors = disc_d_sensors + " " + std::to_string(port);
//                         are_distance_sensors_disconnected = true;
//                     }
//                 }
//                 // Controller screen commands must be delayed by 50ms due to polling limitations
//                 if (are_distance_sensors_disconnected) {
//                     controller.clear_line(0);
//                     pros::delay(CONTROLLER_DELAY_MILLIS);
//                     controller.set_text(0, 0, disc_d_sensors);
//                     is_notified = true;
//                     pros::delay(CONTROLLER_DELAY_MILLIS);
//                 }
//                 if (disconnected.size() > last_disconnected.size()) {
//                     controller.rumble(". . .");
//                 } else if (disconnected.size() < last_disconnected.size()) {
//                     controller.rumble("-");
//                 } else if (!are_distance_sensors_disconnected && is_notified) {
//                     controller.clear_line(0);
//                     is_notified = false;
//                 }
//                 last_disconnected = disconnected;
//                 disconnected.clear();
//             pros::delay(300);
//         }
//     });


// }

// void calibrate_vision() {
//     pros::Task calibrate_vision([]() {
       
//         pros::vision_signature_s_t SIG_1 = pros::Vision::signature_from_utility(1, 2283, 6317, 4300, -5329, -4553, -4941, 5.3, 0);
//         pros::vision_signature_s_t SIG_2 = pros::Vision::signature_from_utility(2, -4793, -4177, -4485, 1449, 5079, 3264, 5.0, 0);
//         pros::vision_signature_s_t SIG_3 = pros::Vision::signature_from_utility(3, 4349, 11213, 7781, 261, 1081, 671, 1.7, 0);
//         vision_sensor.set_signature(1, &SIG_1);
//         vision_sensor.set_signature(2, &SIG_2);
//         vision_sensor.set_signature(3, &SIG_3);
//         vision_sensor.set_auto_white_balance(true);
//         pros::delay(1500); 
//         int wb_value = vision_sensor.get_white_balance();
//         vision_sensor.set_auto_white_balance(false);
//         vision_sensor.set_white_balance(wb_value);
//         pros::Task::current().remove();
//     });
// }

// void detect_line()
// {
//     pros::Task line_detection([]() {
//         bool detected = false;
//         int time = 5000;
//         while(time > 0)
//         {
//             int left_value = line_sensor_left.get_value();
//             int right_value = line_sensor_right.get_value();
//             if(left_value < 2700 && left_value > 2450)
//             {
//                 detected = true;
//                 break;
//             }
//             else if(right_value < 2700 && right_value > 2450)
//             {
//                 detected = true;
//                 break;
//             }
//             pros::delay(50);
//             time -= 100;
//         }
//         if(detected)
//         {
//             chassis.setPose(0, chassis.getPose().y, chassis.getPose().theta, true);
//         }
//         pros::Task::current().remove();
//     });
// }

// void detect_wall()
// {
//     pros::Task detect_wall([]() {
//         bool detected = false;
//         int time = 1000;
//         while(time > 0)
//         {
//             int bumped = bumper_sensor.get_value();

//             if(bumped == 1)
//             {
//                 detected = true;
//                 controller.rumble(".");
//                 break;
//             }
//             pros::delay(100);
//             time -= 100;
//         }
//         return detected;
//         pros::Task::current().remove();
//     });
// }

// void collect_velocity_vs_voltage_data() {
//     std::vector<float> inputs = {0.0f,  0.25f,  0.5f,  0.75f,  1.0f,  1.25f,  1.5f,  1.75f,  2.0f, 2.25f,
//                                  2.5f,  2.75f,  3.0f,  3.25f,  3.5f,  3.75f,  4.0f,  4.25f,  4.5f, 4.75f,
//                                  5.0f,  5.25f,  5.5f,  5.75f,  6.0f,  6.25f,  6.5f,  6.75f,  7.0f, 7.25f,
//                                  7.5f,  7.75f,  8.0f,  8.25f,  8.5f,  8.75f,  9.0f,  9.25f,  9.5f, 9.75f,
//                                  10.0f, 10.25f, 10.5f, 10.75f, 11.0f, 11.25f, 11.5f, 11.75f, 12.0f};
   

//     std::vector<float> outputs = {0.f};
//     outputs.reserve(inputs.size());

//     float direction = 1;
//     for (auto &input : inputs) {
//         if (input == 0)
//             continue;

//         leftMotors.move_voltage(direction * input * 1000);
//         rightMotors.move_voltage(direction * input * 1000);

//         pros::delay(1000);
//         float v_sum = 0;
//         int n;
//         for (n = 0; n < 500; ++n) {
//             v_sum += (std::fabs(leftMotors.get_actual_velocity()*rpm_to_mps_factor) + std::fabs(rightMotors.get_actual_velocity()*rpm_to_mps_factor)) / 2;
//         }
//         outputs.emplace_back( (v_sum / (float)n));
//         auto v = input * direction * 1000;
//         while (fabsf(v) > 0.5) {
//             v *= 0.9;

//             leftMotors.move_voltage(v);
//             rightMotors.move_voltage(v);
//             pros::delay(10);
//         }
//         direction = -direction;
//     }

//     leftMotors.brake();
//     rightMotors.brake();

//     for (int i = 0; i < inputs.size(); ++i) {
//         std::cout << Vector2(inputs[i], outputs[i]).latex() << ",";
//     }
//     std::cout << "\b" << std::endl;
// }

// void collect_voltage_step_data(float step_input, unsigned int duration) {
//     std::vector<float> outputs = {};
//     duration *= 100;
//     outputs.reserve(duration);

//     leftMotors.move_voltage(step_input * 1000);
//     rightMotors.move_voltage(step_input * 1000);

//     for (int i = 0; i < duration; ++i) {
//         auto speed = (std::fabs(leftMotors.get_actual_velocity()*rpm_to_mps_factor) + std::fabs(rightMotors.get_actual_velocity()*rpm_to_mps_factor)) / 2;
//         std::cout << Vector2((float)i / 100, speed).latex() << "," << std::flush;
//         pros::delay(10);
//     }

//     leftMotors.brake();
//     rightMotors.brake();
//     std::cout << "\b" << std::endl;
// }

// void test_w_controller() {
//     double right = 0;
//     double left = 0;
//     double time = pros::millis();
//     VelocityController right_controller(config.kV, config.kA, config.kS, config.kP, config.kI);
//     VelocityController left_controller(config.kV, config.kA, config.kS, config.kP, config.kI);
//     while(true) {
//         if (pros::millis() - time <= 2000) {
//             left = left_controller.update(200, leftMotors.get_actual_velocity());
//             right = right_controller.update(-200, rightMotors.get_actual_velocity());
//         }
//         else {
//             left = left_controller.update(-200, leftMotors.get_actual_velocity());
//             right = right_controller.update(200, rightMotors.get_actual_velocity());
//         }

//         rightMotors.move_voltage(right * 1000.0);
//         leftMotors.move_voltage(left * 1000.0);
//         pros::delay(10);
//         std::cout << Vector2(pros::millis() - time, leftMotors.get_actual_velocity()).latex() << ",";
//     }
// }

// /** Velocity controller testing functions
// *
// */
// class Vector2 {
// public:
//     Vector2(float x, float y) : x(x), y(y) {}
//     std::string latex() const {
//         std::ostringstream oss;
//         oss << "\\left(" << std::fixed << this->x << "," << std::fixed << this->y << "\\right)";
//         return oss.str();
//     }

//     float x;
//     float y;
// };

// void velocity_test(const VelocityControllerConfig &config, float max_velocity, int duration, int acceleration_time) {
//     duration /= 10;
//     acceleration_time /= 10;

//     VoltageController controller(
//         config.kV,
//         config.KA_straight,
//         config.KA_turn,
//         config.KS_straight,
//         config.KS_turn,
//         config.KP_straight,
//         config.KI_straight,
//         99999.0,
//         10.05 * INCH_TO_METER
//     );

//     std::cout << "\\left[";

//     int i;
//     for (i = 0; i < duration; ++i) {
//         auto v_d = max_velocity * fminf(fminf(1, (float)i / (float)acceleration_time),
//                                         (float)(duration - i) / (float)acceleration_time);
//         auto speed = (leftMotors.get_actual_velocity() + rightMotors.get_actual_velocity()) / 2;
//         std::cout << Vector2(i * 0.01f, speed).latex() << ",";
//         std::cout.flush();

//         auto voltage = controller.update(v_d, 0.0, leftMotors.get_actual_velocity() * rpm_to_mps_factor, rightMotors.get_actual_velocity() * rpm_to_mps_factor);
//         leftMotors.move_voltage(voltage.leftVoltage * 1000);
//         rightMotors.move_voltage(voltage.rightVoltage * 1000);

//         pros::delay(10);
//     }
//     std::cout << Vector2(0.01f * (float)i, (leftMotors.get_actual_velocity() + rightMotors.get_actual_velocity()) / 2)
//                      .latex()
//               << "\\right]" << std::endl;

//     leftMotors.brake();
//     rightMotors.brake();
//     std::cout << "\b" << std::endl;
// }