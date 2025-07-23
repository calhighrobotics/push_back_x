#include "motion-profiler.hpp"
#include "motion-utils.hpp"       // for sFunction, curvature, findXandY, findTForS, wrapAngle, sinc
#include "motion-profiling.hpp"   // TrapezoidalProfile
#include "ramsete.hpp"            // RamseteFollower
#include "printer.hpp"            // Printer::printPoseVector / printVelocityVector
#include <cmath>
#include <vector>
#include <string>

// ----------------------------------------------------------------------------
// Constants (same values as your original printVels() used)
constexpr float   MAX_VELOCITY = 2.12f;  // m/s
constexpr float   MAX_ACCEL    = 7.778f;  // m/s²
constexpr float   TRACK_WIDTH  = 0.288925f;       // meters
constexpr float   RAMSETE_B    = 2.0f;
constexpr float   RAMSETE_ZETA = 0.7f;
constexpr float   DT           = 0.01f;           // 10 ms timestep

using namespace MotionUtils; // for sFunction, curvature, findXandY, findTForS, wrapAngle, sinc

void printVels(
    const std::vector<std::vector<Point>>& controlPoints,
    const std::vector<std::vector<KeyframeVelocitiesXandY>>& keyFrameVelocityInitList,
    bool useKeyFrames
) {
    float timeAccum = 0.0f;
    std::vector<std::vector<Pose>> Poses;
    std::vector<std::vector<VelocityLayout>> Velocities;
    std::vector<std::vector<Pose>> RamsetePoses;
    std::vector<std::vector<VelocityLayout>> RamseteVelocities;
    for (size_t i = 0; i < controlPoints.size(); ++i) {

      // 1) Convert (x,y,velocity) keyframes → (time, scalar‐velocity) keyframes,
      //    only if the user requested key-frame limiting.
      std::vector<KeyframeVelocities> keyframes;
      if (useKeyFrames && !keyFrameVelocityInitList.empty()) {
          keyframes = convertToTFrame(controlPoints[i], keyFrameVelocityInitList[i]);
      }

      // 2) Determine initial and exit velocity
      float initialVel = (useKeyFrames && !keyframes.empty())
                       ? keyframes.front().velocity
                       : 0.0f;
      float exitVel = (useKeyFrames && !keyframes.empty())
                    ? keyframes.back().velocity
                    : 0.0f;

      // 3) Compute total spline length = sFunction(…, t=1)
      float totalLength = sFunction(controlPoints[i], 1.0f);

      // 4) Compute deceleration distance exactly as in your original code:
      //    decelDist = totalLength − [ (exitVel² − maxVel²) / (−2·maxAccel) ]
      float decelDist = totalLength 
                    - ((std::pow(exitVel, 2.0f) - std::pow(MAX_VELOCITY, 2.0f))
                       / (-2.0f * MAX_ACCEL));
    
      // 5) Build and run the trapezoidal profile
      TrapezoidalProfile profiler(
          controlPoints[i],
          MAX_VELOCITY,
          MAX_ACCEL,
          decelDist,
	  timeAccum,
          initialVel,
          exitVel,
          keyframes,
          useKeyFrames,
          DT
      );

      profiler.start();
      while (!profiler.isFinished()) {
          profiler.step();
      }
      Poses.push_back(profiler.getPoses());
      Velocities.push_back(profiler.getVelocities());

      // 7) Build & run the RAMSETE follower
      RamseteFollower ramser(
          profiler.getPoses(), 
	  profiler.getVelocities(),
	  TRACK_WIDTH,
          RAMSETE_B,
          RAMSETE_ZETA,
	  timeAccum,
          DT,
	  false
      );

      while (!ramser.isFinished()) {
          ramser.step();
      }

      RamsetePoses.push_back(ramser.getExecutedPoses());
      RamseteVelocities.push_back(ramser.getExecutedVelocities());
      timeAccum = profiler.getVelocities()[profiler.getVelocities().size() - 1].time;
    }
    // 6) Print the open-loop (“nominal”) path:
    Printer::printPoseVector(    "X = ",  Poses           );
    Printer::printVelocityVector("L = ",  Velocities, "linear"  );
    Printer::printVelocityVector("A = ",  Velocities, "angular" );
    // 8) Print the closed-loop (“RAMSETE‐executed”) path:
    Printer::printPoseVector(    "X_r = ",  RamsetePoses           );
    Printer::printVelocityVector("L_r = ",  RamseteVelocities, "linear"  );     Printer::printVelocityVector("A_r = ",  RamseteVelocities, "angular" );

}
