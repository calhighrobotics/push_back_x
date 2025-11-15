# 1516X - Push Back
This is the Push Back repository for team 1516X.

We are a VEX VRC team based in the Bay Area, California.

# Features:
 - Drivetrain - curvature
 - Odometry(Lemlib):
   - Use IMU and horizontal/vertical tracking wheels to measure robot linear and angular movement
 - Color Sorting:
   - Use VEX color sensor to detect color of blocks that are intaked
   - Controls roller storage mechanism for automated sorting
 - Motor Disconnect Warning:
   - Post motor disconnet warnings on controller to inform driver of disfunctional robot operations
 - Temperature Warning:
   - Controller warning when motor is overheating to prevent motor abuse
# Autonomous Features:
 - Voltage controller based on PID and feedforward loop to account for static)
 - RAMSETE Control:
   - uses odometry to correct error in motion based on set of velocity instructions and corresponding positions
   - convert relative error between current and expected positions to global error
   - produce exaggerated linear and angular velocity based on correction required
 - Vision-based Control:
   - Use VEX vision sensor for object detection
   - Detects blocks on the field
   - Detect color of goal base for automatic PID-based alignment and scoring
 - Line Detection:
   - Simple odometry recalibration
   - Detects when robot crosses line spanning field center, reset x position
 - Wall Detection:
   - Simple odometry recalibration
   - Use bumper sensor to detect when wall is hit to reset corresponding position index
 - Distance Reset:
   - localization through 4 sensors
   - read distance values from sensors and apply physical offset
   - ignore invalid sensor readings (beyond distance limit)
   - rotate relative reading to global distances using robot heading
   - return estimated global position for odometry recalibration
 - Codebase:
   - Meticulous branching/merging system with github
   - Object-oriented approach to codebase system representations
   - Modularization of subsystems/contstructs into classes
# Other Libraries
We use the following libraries to make our codebase possible. We are indebted to their contributions to make this codebase possible. Check out their code for types and functions that we use as well.
 - [PROS](https://pros.cs.purdue.edu/v5/pros-4/)
 - [LemLib](https://lemlib.readthedocs.io/)
 - [Eigen](https://libeigen.gitlab.io/)
 - [LVGL](https://lvgl.io/)
# License
See [LICENSE](https://github.com/calhighrobotics/push_back_x/blob/main/LICENSE.txt) for more information.
