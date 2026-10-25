// Connection quality test that keeps the robot still: commands the current joint position at
// 1 kHz for N seconds and reports libfranka's control command success rate.
// Usage: hold_test <robot-ip> [seconds] [cartesian]
//   cartesian: hold the last commanded Cartesian pose (O_T_EE_c) instead of the joint position
#include <algorithm>
#include <cstdlib>
#include <string>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip> [seconds]" << std::endl;
    return 1;
  }
  const double seconds = argc > 2 ? std::atof(argv[2]) : 10.0;
  const bool cartesian = argc > 3 && std::string(argv[3]) == "cartesian";
  double min_rate = 1.0, sum_rate = 0.0;
  long samples = 0;
  try {
    franka::Robot robot(argv[1]);
    std::array<double, 7> q_hold{};
    std::array<double, 16> pose_hold{};
    double t = 0.0;
    auto track = [&](const franka::RobotState& state, franka::Duration period) {
      t += period.toSec();
      if (t > 0.5) {
        min_rate = std::min(min_rate, state.control_command_success_rate);
        sum_rate += state.control_command_success_rate;
        samples++;
      }
    };
    if (cartesian) {
      robot.control([&](const franka::RobotState& state, franka::Duration period) {
        if (t == 0.0) {
          pose_hold = state.O_T_EE_c;
        }
        track(state, period);
        franka::CartesianPose cmd(pose_hold);
        if (t >= seconds) {
          return franka::MotionFinished(cmd);
        }
        return cmd;
      });
    } else
    robot.control([&](const franka::RobotState& state, franka::Duration period) {
      if (t == 0.0) {
        q_hold = state.q_d;  // last commanded position: no jump
      }
      track(state, period);  // the rate needs half a second to become meaningful
      franka::JointPositions cmd(q_hold);
      if (t >= seconds) {
        return franka::MotionFinished(cmd);
      }
      return cmd;
    });
  } catch (const franka::Exception& e) {
    std::cerr << "FAILED: " << e.what() << std::endl;
    return 2;
  }
  std::cout << "held still for " << seconds << " s, control command success rate: mean "
            << (samples ? sum_rate / samples : 0.0) * 100.0 << " %, min " << min_rate * 100.0
            << " %" << std::endl;
  return 0;
}
