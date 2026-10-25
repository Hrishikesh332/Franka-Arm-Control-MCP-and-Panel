// Minimal libfranka motion test: clears errors, then moves the end effector smoothly 2 cm up and
// back down (cosine profile, 4 s) using Cartesian pose control starting at O_T_EE_c.
// Usage: move_test <robot-ip> [height_m]
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip> [height_m]" << std::endl;
    return 1;
  }
  const double height = argc > 2 ? std::atof(argv[2]) : 0.02;
  const double duration = 4.0;
  try {
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();
    std::array<double, 16> start{};
    double t = 0.0, z0 = 0.0, z_max = -1.0, min_rate = 1.0;
    robot.control([&](const franka::RobotState& state, franka::Duration period) {
      if (t == 0.0) {
        start = state.O_T_EE_c;
        z0 = state.O_T_EE[14];
      }
      t += period.toSec();
      if (t > 0.5) {
        min_rate = std::min(min_rate, state.control_command_success_rate);
      }
      z_max = std::max(z_max, state.O_T_EE[14]);
      std::array<double, 16> pose = start;
      pose[14] += height * 0.5 * (1.0 - std::cos(2.0 * M_PI * std::min(t, duration) / duration));
      if (t >= duration) {
        return franka::MotionFinished(franka::CartesianPose(pose));
      }
      return franka::CartesianPose(pose);
    });
    std::cout << "moved up " << (z_max - z0) * 1000.0 << " mm and back; min success rate "
              << min_rate * 100.0 << " %" << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << "FAILED: " << e.what() << std::endl;
    return 2;
  }
  return 0;
}
