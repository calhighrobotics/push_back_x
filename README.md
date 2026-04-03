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
 - Distance Sensor Disconnect Warning:
   - Controller warning when motor is overheating to prevent fault distance resets
# Autonomous Features:
 - Arc based odometry inspired by the Pilons:
   - Utilizes 2 odometry tracking wheels (horizontal and vertical)
   - Utilizes IMU for heading and acceleration
 - Voltage controller based on PID and feedforward loop to account for static friction, voltage slip, etc.
 - RAMSETE Control:
   - uses odometry to correct error in motion based on set of velocity instructions and corresponding positions
   - convert relative error between current and expected positions to global error
   - produce exaggerated linear and angular velocity based on correction required
   - Current velocity calculated by averaging motor encoders for more accurate tangential velocity calculations (Victim to slip and encoder noise)
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
 - Monte Carlo Localization:
   - Uses advanced particle filter for robot pose localization in 2d space
   - Utilizes Adaptive MCL for minimal tuning and behavior adapting around unexpected noise/unexpected obstacles
   - Current performance runs around 2000 particles (10ms) and 5000 particles (33ms (distance sensor refresh time))
   - Utilizes raycast and line intersection logic for map based local known MCL
   - Robust MCL in progress for more accurate sensor likelyhood model
 - Color Sorting:
   - Using optical sensor for color signature identification as well as sorting
   - Through the use of our topMotor, we are able to eject blocks of the other alliance
   - In skills we are able to utilize this for storing/separating blocks of different color for control bonus
   - Uses both simple blocking function as well as "multithreading" (TASK)
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
