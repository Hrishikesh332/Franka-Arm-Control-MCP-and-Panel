// franka_teach_rt: the realtime robot process of franka_teach, built directly on libfranka.
//
// It owns the connection to the robot and runs one of three modes:
//   idle    - only reads the robot state (the robot holds itself); robot parameters
//             (load, collision behaviour) and error recovery are applied here
//   follow  - Cartesian pose control that smoothly follows the latest target pose, with an
//             optional force mode along z (pressing)
//   free    - torque control with gravity compensation and joint damping (hand guiding)
//   home    - slow joint motion to the ready pose (every joint well inside its range), then idle
//   jfollow - joint-space follower: smoothly moves to target joint angles chosen by the server
//             (replaying recorded configurations instead of letting the robot's IK choose)
//
// The teach server (Python) talks to it over two local UDP ports with one-line text messages:
// commands to 127.0.0.1:5555, state broadcasts at 50 Hz to 127.0.0.1:5556 (see send_state()).
//
// Usage: franka_teach_rt <robot-ip>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

namespace {

constexpr int kCommandPort = 5555;
constexpr int kStatePort = 5556;
constexpr double kStatePeriod = 0.02;  // s

// Motion limits of the follower (at speed scale 1).
constexpr double kMaxLinearVelocity = 0.25;       // m/s
constexpr double kMaxLinearAcceleration = 0.5;    // m/s^2
constexpr double kMaxAngularVelocity = 0.6;       // rad/s
constexpr double kMaxAngularAcceleration = 1.2;   // rad/s^2
constexpr double kSmoothingTime = 0.015;          // s per filter stage, bounds the jerk
constexpr double kMaxTargetDistance = 1.0;        // m, further targets are ignored
constexpr double kMinZ = 0.01;                    // m, lower targets are ignored
// O_F_ext_hat_K is the force the robot exerts: pressing down reads negative z (measured).
constexpr double kForceSign = -1.0;
constexpr double kForceFilterHz = 30.0;
constexpr double kForceAcceleration = 0.1;        // m/s^2, vertical acceleration in force mode
// Free drive: joint damping (Nm*s/rad), torque rate and magnitude limits.
constexpr std::array<double, 7> kDamping{{2.0, 2.0, 2.0, 2.0, 1.0, 1.0, 0.5}};
constexpr double kMaxTorqueRate = 500.0;          // Nm/s
constexpr double kMaxDampingTorque = 10.0;        // Nm
// The FR3's joint velocity limits depend on the joint position and drop to zero at the ends of
// the range. Above this fraction of the current limit, follow mode slows down and free drive
// damps harder, so neither runs into a joint_velocity_violation reflex.
constexpr double kJointSpeedSoftLimit = 0.5;
constexpr double kAutoRecoverDelay = 0.5;         // s after a reflex before clearing it
// Soft joint limits: at the end of a joint's range its allowed velocity is zero, and any motion
// there trips joint_velocity_violation (the arm can even get stuck). Free drive pushes back with a
// spring this far before the end; follow mode stops when a joint gets this close and keeps going.
constexpr double kFreeWallMargin = 0.12;          // rad - narrow, so free drive feels free
constexpr double kFreeWallStiffness = 10.0;       // Nm/rad
constexpr double kFreeDampingSoftLimit = 0.75;    // extra damping only close to the velocity limit
constexpr double kFollowStopMargin = 0.15;        // rad
// Home: Franka's ready pose, far from all joint limits; reached with a cosine profile whose peak
// joint speed is kHomeSpeed. A stop request slows the motion down to standstill smoothly.
constexpr std::array<double, 7> kHomePose{{0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4}};
constexpr double kHomeSpeed = 0.3;                // rad/s
constexpr double kHomeStopTime = 0.5;             // s to slow down to standstill
// Joint follower (at speed scale 1): all joints move synchronously along a straight line in joint
// space; the fastest joint is limited to these.
constexpr double kJointMaxVelocity = 1.0;         // rad/s
constexpr double kJointMaxAcceleration = 2.0;     // rad/s^2
constexpr double kJointTargetMargin = 0.1;        // rad, targets are clamped this far inside the ranges
constexpr double kPathDt = 0.01;                  // s between the samples of a joint trajectory
constexpr double kPathStopTime = 0.3;             // s to slow a trajectory down to standstill

enum class Mode { kIdle, kFollow, kFree, kHome, kJointFollow };
enum ForceFault { kNoFault = 0, kAbortForce = 1, kMaxTravel = 2, kNoContact = 3 };

const char* mode_name(Mode m) {
  return m == Mode::kFollow ? "follow" : m == Mode::kFree ? "free" : m == Mode::kHome ? "home"
       : m == Mode::kJointFollow ? "jfollow" : "idle";
}

// Per joint: |dq| as a fraction of the position-dependent velocity limit in that direction.
std::array<double, 7> joint_speed_ratios(const std::array<double, 7>& q, const std::array<double, 7>& dq) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto upper = franka::computeUpperLimitsJointVelocity(q);
  const auto lower = franka::computeLowerLimitsJointVelocity(q);
#pragma GCC diagnostic pop
  std::array<double, 7> r{};
  for (size_t i = 0; i < 7; i++) {
    const double limit = dq[i] >= 0.0 ? upper[i] : -lower[i];
    r[i] = std::abs(dq[i]) / std::max(limit, 1e-3);
  }
  return r;
}

// FR3 joint position ranges (rad), from Franka's control parameter documentation.
constexpr std::array<double, 7> kJointMin{{-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159}};
constexpr std::array<double, 7> kJointMax{{2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159}};

// Which joint came closest to its velocity limit in the states libfranka logged before a reflex,
// e.g. "joint 7 at 2.91 rad (range -3.02..3.02): 0.42 rad/s, allowed 0.35".
std::string joint_diagnosis(const std::vector<franka::Record>& log) {
  double worst = -1.0;
  size_t joint = 0;
  franka::RobotState at;
  for (const auto& record : log) {
    const auto r = joint_speed_ratios(record.state.q, record.state.dq);
    for (size_t i = 0; i < 7; i++) {
      if (r[i] > worst) {
        worst = r[i];
        joint = i;
        at = record.state;
      }
    }
  }
  if (worst < 0.0) {
    return "";
  }
  const double q = at.q[joint], dq = at.dq[joint];
  char buf[200];
  std::snprintf(buf, sizeof(buf), "joint %zu at %.2f rad (range %.2f..%.2f): %.2f rad/s, allowed %.2f",
                joint + 1, q, kJointMin[joint], kJointMax[joint], std::abs(dq),
                std::abs(dq) / std::max(worst, 1e-6));
  std::string out(buf);
  if (q - kJointMin[joint] < 0.25 || kJointMax[joint] - q < 0.25) {
    out += " - near the end of its range";
  }
  return out;
}

// 1-based index of a joint closer than margin to the end of its range and moving towards it.
int joint_approaching_limit(const franka::RobotState& s, double margin) {
  for (size_t i = 0; i < 7; i++) {
    if ((s.q[i] > kJointMax[i] - margin && s.dq[i] > 1e-3) ||
        (s.q[i] < kJointMin[i] + margin && s.dq[i] < -1e-3)) {
      return static_cast<int>(i) + 1;
    }
  }
  return 0;
}

Eigen::Vector3d clamp_norm(const Eigen::Vector3d& v, double max_norm) {
  const double n = v.norm();
  return n > max_norm ? Eigen::Vector3d(v * (max_norm / n)) : v;
}

struct ForceCommand {
  bool enable{false};
  double force{0}, gain{0}, approach_speed{0}, contact_speed{0}, max_depth{0}, abort_force{0};
  double max_travel{0.03};  // how far it may go into the object after contact (max_depth: search)
};

// Everything shared between the realtime loop and the communication thread.
struct Shared {
  std::mutex mutex;
  Mode requested_mode{Mode::kIdle};
  uint64_t target_seq{0};
  uint64_t jtarget_seq{0};
  std::array<double, 7> jtarget{};
  // joint trajectory sampled every kPathDt, built by the comm thread and swapped into the loop
  std::vector<std::array<double, 7>> path_building, path_ready;
  uint64_t path_seq{0};
  Eigen::Vector3d target_p{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_q{Eigen::Quaterniond::Identity()};
  uint64_t force_seq{0};
  ForceCommand force;
  std::deque<std::string> parameter_commands;  // executed in idle mode
  std::atomic<bool> stop_requested{false};
  std::atomic<double> speed{0.3};
  std::atomic<int> joint_warning{0};
  char start_info[160]{};              // written once at a motion start, printed by the comm thread
  std::atomic<bool> start_info_ready{false};  // 1-based joint that follow mode stopped for (0 = none)
  std::atomic<bool> running{true};

  // state snapshot written by the realtime loop
  std::mutex state_mutex;
  Mode mode{Mode::kIdle};
  franka::RobotState robot_state;
  bool have_state{false};
  std::array<double, 8> follower{};  // dist, angle, v, w, active, force active, contact, fault
  std::string last_error;
  uint64_t counter{0};
};

// Smoothly follows target poses: planner with velocity/acceleration limits, then two first-order
// filters (continuous acceleration). Optional admittance force control along z.
class Follower {
 public:
  void reset(const std::array<double, 16>& O_T_EE_c) {
    const Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(O_T_EE_c.data());
    p_cmd_ = p_f1_ = p_ref_ = p_target_ = T.block<3, 1>(0, 3);
    q_cmd_ = q_f1_ = q_ref_ = q_target_ = Eigen::Quaterniond(T.block<3, 3>(0, 0)).normalized();
    v_.setZero();
    w_.setZero();
    holding_ = false;
    force_ = ForceCommand();
    force_filtered_ = std::nan("");
    fault_ = kNoFault;
  }

  void set_target(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    if ((p - p_ref_).norm() <= kMaxTargetDistance && p.z() >= kMinZ) {
      p_target_ = p;
      q_target_ = q;
      holding_ = false;
    }
  }

  void stop() {
    holding_ = true;
    if (force_.enable) {
      leave_force_mode();
    }
  }

  void set_force(const ForceCommand& cmd) {
    if (cmd.enable && !force_.enable) {
      force_ = cmd;
      force_tare_ = std::isnan(force_filtered_) ? 0.0 : force_filtered_;  // starts in free air
      force_start_z_ = p_ref_.z();
      force_touched_ = false;
      force_holding_ = false;
      fault_ = kNoFault;
    } else if (!cmd.enable && force_.enable) {
      leave_force_mode();
    } else if (cmd.enable) {
      if (cmd.force != force_.force) {
        force_holding_ = false;  // new target (e.g. while measuring): regulate towards it again
      }
      force_.force = cmd.force;
    }
  }

  bool settled() const {
    return v_.norm() < 1e-9 && w_.norm() < 1e-9 && (p_cmd_ - p_ref_).norm() < 1e-6 &&
           q_cmd_.angularDistance(q_ref_) < 1e-6;
  }

  std::array<double, 16> step(double dt, double scale, double fz) {
    const double alpha_f = 1.0 - std::exp(-2.0 * M_PI * kForceFilterHz * dt);
    fz *= kForceSign;
    force_filtered_ = std::isnan(force_filtered_) ? fz : force_filtered_ + alpha_f * (fz - force_filtered_);
    contact_ = force_filtered_ - force_tare_;

    // planner: translation
    Eigen::Vector3d e = p_target_ - p_ref_;
    if (force_.enable) {
      e.z() = 0.0;
    }
    const double dist = e.norm();
    Eigen::Vector3d v_des = Eigen::Vector3d::Zero();
    if (!holding_ && dist > 1e-7) {
      v_des = e / dist * std::min(kMaxLinearVelocity * scale, std::sqrt(2.0 * kMaxLinearAcceleration * dist));
    }
    if (force_.enable) {
      if (!force_touched_ && contact_ > std::min(2.0, 0.3 * force_.force)) {
        force_touched_ = true;
        force_contact_z_ = p_ref_.z();
      }
      const double limit = force_touched_ ? force_.contact_speed : force_.approach_speed;
      const double error = force_.force - contact_;
      // Once the force is reached, hold the height instead of regulating on: continuous
      // regulation oscillates on stiff objects (and parts that bottom out). Only correct again,
      // slowly, if the force drifts far away (hysteresis).
      if (!force_holding_ && std::abs(error) < std::max(0.5, 0.05 * force_.force)) {
        force_holding_ = true;
      } else if (force_holding_ && std::abs(error) > 0.3 * force_.force) {
        force_holding_ = false;
      }
      v_des.z() = force_holding_ ? 0.0 : -std::clamp(force_.gain * error, -limit, limit);
      if (contact_ > force_.abort_force) {
        fault_ = kAbortForce;
      } else if (!force_touched_ && force_start_z_ - p_ref_.z() > force_.max_depth && v_des.z() < 0.0) {
        fault_ = kNoContact;   // searched the whole distance without touching anything
      } else if (force_touched_ && force_contact_z_ - p_ref_.z() > force_.max_travel && v_des.z() < 0.0) {
        fault_ = kMaxTravel;   // pressed this far into the object without reaching the force
      }
      if (fault_ != kNoFault) {
        leave_force_mode();
        holding_ = true;
        v_des.setZero();
      }
    }
    Eigen::Vector3d dv = clamp_norm(v_des - v_, kMaxLinearAcceleration * dt);
    if (force_.enable) {
      dv.z() = std::clamp(v_des.z() - v_.z(), -kForceAcceleration * dt, kForceAcceleration * dt);
    }
    v_ += dv;
    p_ref_ += v_ * dt;
    if (!force_.enable && dist < 1e-5 && v_.norm() < 2.0 * kMaxLinearAcceleration * dt) {
      p_ref_ = p_target_;
      v_.setZero();
    }

    // planner: rotation (world frame angular velocity)
    Eigen::Quaterniond q_err = q_target_ * q_ref_.conjugate();
    if (q_err.w() < 0) {
      q_err.coeffs() *= -1.0;
    }
    const Eigen::AngleAxisd aa(q_err);
    const double angle = aa.angle();
    Eigen::Vector3d w_des = Eigen::Vector3d::Zero();
    if (!holding_ && angle > 1e-6) {
      w_des = aa.axis() * std::min(kMaxAngularVelocity * scale, std::sqrt(2.0 * kMaxAngularAcceleration * angle));
    }
    w_ += clamp_norm(w_des - w_, kMaxAngularAcceleration * dt);
    const double w_norm = w_.norm();
    if (w_norm > 1e-12) {
      q_ref_ = (Eigen::Quaterniond(Eigen::AngleAxisd(w_norm * dt, w_ / w_norm)) * q_ref_).normalized();
    }
    if (angle < 1e-5 && w_norm < 2.0 * kMaxAngularAcceleration * dt) {
      q_ref_ = q_target_;
      w_.setZero();
    }

    // smoothing: two first-order filters in series (z bypasses them in force mode)
    const Eigen::Vector3d p_prev = p_cmd_;
    const Eigen::Quaterniond q_prev = q_cmd_;
    const double a = std::min(1.0, dt / kSmoothingTime);
    p_f1_ += a * (p_ref_ - p_f1_);
    p_cmd_ += a * (p_f1_ - p_cmd_);
    q_f1_ = q_f1_.slerp(a, q_ref_).normalized();
    q_cmd_ = q_cmd_.slerp(a, q_f1_).normalized();
    if (force_.enable) {
      p_f1_.z() = p_ref_.z();
      p_cmd_.z() = p_ref_.z();
    }
    v_cmd_ = (p_cmd_ - p_prev).norm() / dt;
    w_cmd_ = q_cmd_.angularDistance(q_prev) / dt;

    if (holding_ && v_.norm() < 1e-9 && w_.norm() < 1e-9) {
      p_target_ = p_ref_;
      q_target_ = q_ref_;
    }

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = q_cmd_.toRotationMatrix();
    T.block<3, 1>(0, 3) = p_cmd_;
    std::array<double, 16> out{};
    Eigen::Map<Eigen::Matrix4d>(out.data()) = T;
    return out;
  }

  std::array<double, 8> status() const {
    Eigen::Vector3d remaining = p_target_ - p_cmd_;
    if (force_.enable) {
      remaining.z() = 0.0;
    }
    return {remaining.norm(), q_target_.angularDistance(q_cmd_), v_cmd_, w_cmd_, 1.0,
            force_.enable ? (force_holding_ ? 2.0 : 1.0) : 0.0, contact_, static_cast<double>(fault_)};
  }

 private:
  void leave_force_mode() {
    force_.enable = false;
    p_target_.z() = p_ref_.z() + v_.z() * std::abs(v_.z()) / (2.0 * kMaxLinearAcceleration);
  }

  Eigen::Vector3d p_ref_, p_f1_, p_cmd_, p_target_, v_, w_;
  Eigen::Quaterniond q_ref_, q_f1_, q_cmd_, q_target_;
  double v_cmd_{0}, w_cmd_{0};
  bool holding_{false};
  ForceCommand force_;
  double force_filtered_{0}, force_tare_{0}, force_start_z_{0}, force_contact_z_{0}, contact_{0};
  bool force_touched_{false};
  bool force_holding_{false};  // force reached: holding the height
  ForceFault fault_{kNoFault};
};

void pin_current_thread(int first_cpu, int last_cpu) {
  cpu_set_t set;
  CPU_ZERO(&set);
  for (int c = first_cpu; c <= last_cpu; c++) {
    CPU_SET(c, &set);
  }
  pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
}

void store_state(Shared& shared, Mode mode, const franka::RobotState& state,
                 const std::array<double, 8>& follower) {
  std::unique_lock<std::mutex> lock(shared.state_mutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;  // never block the realtime loop; the next cycle will update it
  }
  shared.mode = mode;
  shared.robot_state = state;
  shared.have_state = true;
  shared.follower = follower;
  shared.counter++;
}

// ------------------------------------------------------------------ communication thread
void send_state(Shared& shared, int sock, const sockaddr_in& dest) {
  franka::RobotState s;
  std::array<double, 8> f{};
  Mode mode;
  std::string error;
  uint64_t counter;
  {
    std::lock_guard<std::mutex> lock(shared.state_mutex);
    if (!shared.have_state) {
      // not connected to the robot: report why (e.g. FCI not activated in Desk)
      const std::string msg = "E " + shared.last_error;
      sendto(sock, msg.data(), msg.size(), 0, reinterpret_cast<const sockaddr*>(&dest), sizeof(dest));
      return;
    }
    s = shared.robot_state;
    f = shared.follower;
    mode = shared.mode;
    error = shared.last_error;
    counter = shared.counter;
  }
  // S <mode> <robot_mode> <success> <O_T_EE x16> <O_F_ext_hat_K x6> <follower x8> <q x7> <counter>
  //   |<current errors>|<last error>|<warning>
  // or, while there is no connection to the robot:  E <reason>
  std::ostringstream out;
  out.precision(9);
  out << "S " << mode_name(mode) << ' ' << static_cast<int>(s.robot_mode) << ' '
      << s.control_command_success_rate;
  for (double v : s.O_T_EE) out << ' ' << v;
  for (double v : s.O_F_ext_hat_K) out << ' ' << v;
  for (double v : f) out << ' ' << v;
  for (double v : s.q) out << ' ' << v;
  const int joint = shared.joint_warning.load();
  out << ' ' << counter << '|' << static_cast<std::string>(s.current_errors) << '|' << error << '|'
      << (joint ? "joint " + std::to_string(joint) + " near the end of its range - stopped" : "");
  const std::string msg = out.str();
  sendto(sock, msg.data(), msg.size(), 0, reinterpret_cast<const sockaddr*>(&dest), sizeof(dest));
}

void handle_command(Shared& shared, const std::string& line) {
  std::istringstream in(line);
  std::string cmd;
  in >> cmd;
  std::lock_guard<std::mutex> lock(shared.mutex);
  if (cmd == "mode") {
    std::string m;
    in >> m;
    shared.requested_mode = m == "follow" ? Mode::kFollow : m == "free" ? Mode::kFree
                            : m == "home"   ? Mode::kHome
                            : m == "jfollow" ? Mode::kJointFollow : Mode::kIdle;
    shared.joint_warning = 0;
    if (shared.requested_mode != Mode::kIdle) {
      std::lock_guard<std::mutex> state_lock(shared.state_mutex);
      shared.last_error.clear();
    }
  } else if (cmd == "jpath_begin") {
    shared.path_building.clear();
  } else if (cmd == "jpath") {  // any number of 7-value points
    std::array<double, 7> q{};
    while (true) {
      for (double& x : q) in >> x;
      if (!in) break;
      for (size_t i = 0; i < 7; i++) {
        q[i] = std::clamp(q[i], kJointMin[i] + kJointTargetMargin, kJointMax[i] - kJointTargetMargin);
      }
      shared.path_building.push_back(q);
    }
  } else if (cmd == "jpath_end") {
    shared.path_ready.swap(shared.path_building);  // allocation stays in this thread
    shared.path_building.clear();
    shared.path_seq++;
  } else if (cmd == "jtarget") {
    std::array<double, 7> q{};
    for (double& x : q) in >> x;
    if (in) {
      for (size_t i = 0; i < 7; i++) {  // never aim into the last kJointTargetMargin of a range
        q[i] = std::clamp(q[i], kJointMin[i] + kJointTargetMargin, kJointMax[i] - kJointTargetMargin);
      }
      shared.jtarget = q;
      shared.jtarget_seq++;
    }
  } else if (cmd == "target") {
    double v[7];
    for (double& x : v) in >> x;
    if (in) {
      shared.target_p = Eigen::Vector3d(v[0], v[1], v[2]);
      shared.target_q = Eigen::Quaterniond(v[6], v[3], v[4], v[5]).normalized();
      shared.target_seq++;
    }
  } else if (cmd == "stop") {
    shared.stop_requested = true;
  } else if (cmd == "speed") {
    double s;
    if (in >> s) shared.speed = std::clamp(s, 0.01, 1.0);
  } else if (cmd == "force") {
    ForceCommand f;
    int enable = 0;
    in >> enable;
    f.enable = enable != 0;
    if (f.enable) {
      in >> f.force >> f.gain >> f.approach_speed >> f.contact_speed >> f.max_depth >> f.abort_force;
      if (!in) return;
      double travel;
      if (in >> travel) f.max_travel = std::clamp(travel, 0.0, 0.1);
      f.force = std::clamp(f.force, 0.0, 60.0);
      f.gain = std::clamp(f.gain, 0.0, 0.01);
      f.approach_speed = std::clamp(f.approach_speed, 0.0, 0.05);
      f.contact_speed = std::clamp(f.contact_speed, 0.0, f.approach_speed);
      f.max_depth = std::clamp(f.max_depth, 0.0, 0.2);
      f.abort_force = std::clamp(f.abort_force, f.force, 80.0);
    }
    shared.force = f;
    shared.force_seq++;
  } else if (cmd == "load" || cmd == "collision" || cmd == "recover") {
    shared.parameter_commands.push_back(line);
  }
}

void communication_thread(Shared& shared) {
  pin_current_thread(0, 2);
  int rx = socket(AF_INET, SOCK_DGRAM, 0);
  int tx = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(kCommandPort);
  if (bind(rx, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    perror("bind command port");
    shared.running = false;
    return;
  }
  sockaddr_in dest = addr;
  dest.sin_port = htons(kStatePort);
  auto next_state = std::chrono::steady_clock::now();
  static char buf[65536];  // trajectory chunks are several kB
  while (shared.running) {
    pollfd pfd{rx, POLLIN, 0};
    const int timeout_ms = std::max<int>(0, std::chrono::duration_cast<std::chrono::milliseconds>(
                                                next_state - std::chrono::steady_clock::now()).count());
    if (poll(&pfd, 1, timeout_ms) > 0) {
      const ssize_t n = recv(rx, buf, sizeof(buf) - 1, 0);
      if (n > 0) {
        buf[n] = '\0';
        handle_command(shared, std::string(buf));
      }
    }
    if (shared.start_info_ready.exchange(false)) {
      std::cout << shared.start_info << std::endl;
    }
    if (std::chrono::steady_clock::now() >= next_state) {
      send_state(shared, tx, dest);
      next_state += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(kStatePeriod));
    }
  }
  close(rx);
  close(tx);
}

// ------------------------------------------------------------------ robot modes
void set_default_collision_behavior(franka::Robot& robot) {
  const std::array<double, 7> t_lo{{25, 25, 22, 20, 19, 17, 14}}, t_hi{{35, 35, 32, 30, 29, 27, 24}};
  const std::array<double, 6> f_lo{{30, 30, 30, 25, 25, 25}}, f_hi{{40, 40, 40, 35, 35, 35}};
  robot.setCollisionBehavior(t_lo, t_hi, t_lo, t_hi, f_lo, f_hi, f_lo, f_hi);
}

void run_parameter_command(franka::Robot& robot, Shared& shared, const std::string& line) {
  std::istringstream in(line);
  std::string cmd;
  in >> cmd;
  if (cmd == "load") {
    double m, x, y, z;
    in >> m >> x >> y >> z;
    robot.setLoad(m, {{x, y, z}}, {{1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3}});
  } else if (cmd == "collision") {
    set_default_collision_behavior(robot);
  } else if (cmd == "recover") {
    robot.automaticErrorRecovery();
    std::lock_guard<std::mutex> lock(shared.state_mutex);
    shared.last_error.clear();
  }
  std::cout << "done: " << line << std::endl;
}

Mode requested(Shared& shared) {
  std::lock_guard<std::mutex> lock(shared.mutex);
  return shared.requested_mode;
}

void run_idle(franka::Robot& robot, Shared& shared) {
  for (;;) {
    std::string cmd;
    {
      std::lock_guard<std::mutex> lock(shared.mutex);
      if (shared.parameter_commands.empty()) break;
      cmd = shared.parameter_commands.front();
      shared.parameter_commands.pop_front();
    }
    try {
      run_parameter_command(robot, shared, cmd);
    } catch (const franka::Exception& e) {
      std::lock_guard<std::mutex> lock(shared.state_mutex);
      shared.last_error = std::string(cmd.substr(0, cmd.find(' '))) + ": " + e.what();
    }
  }
  robot.read([&](const franka::RobotState& state) {
    store_state(shared, Mode::kIdle, state, {});
    std::lock_guard<std::mutex> lock(shared.mutex);
    return shared.running && shared.requested_mode == Mode::kIdle &&
           shared.parameter_commands.empty();
  });
}

void run_follow(franka::Robot& robot, Shared& shared) {
  Follower follower;
  bool first = true;
  uint64_t target_seq = 0, force_seq = 0;
  {
    std::lock_guard<std::mutex> lock(shared.mutex);
    target_seq = shared.target_seq;  // ignore targets sent before this mode started
    force_seq = shared.force_seq;
  }
  shared.stop_requested = false;
  bool leaving = false;
  double governor = 1.0;  // slows the follower down when a joint approaches its velocity limit
  robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose {
    if (first) {
      // The robot expects the first command at its last commanded pose (O_T_EE_c). After a joint
      // motion that value may be stale, so fall back to the desired / measured pose if it is off.
      auto offset = [&](const std::array<double, 16>& T) {
        return std::sqrt(std::pow(T[12] - state.O_T_EE[12], 2) + std::pow(T[13] - state.O_T_EE[13], 2) +
                         std::pow(T[14] - state.O_T_EE[14], 2));
      };
      const double off_c = offset(state.O_T_EE_c), off_d = offset(state.O_T_EE_d);
      const char* used = off_c < 0.002 ? "O_T_EE_c" : off_d < 0.002 ? "O_T_EE_d" : "O_T_EE";
      follower.reset(off_c < 0.002 ? state.O_T_EE_c : off_d < 0.002 ? state.O_T_EE_d : state.O_T_EE);
      std::snprintf(shared.start_info, sizeof(shared.start_info),
                    "follow starts at %s (O_T_EE_c %.2f mm, O_T_EE_d %.2f mm from measured)", used,
                    off_c * 1000, off_d * 1000);
      shared.start_info_ready = true;
      first = false;
    }
    std::unique_lock<std::mutex> lock(shared.mutex, std::try_to_lock);
    if (lock.owns_lock()) {
      if (shared.target_seq != target_seq) {
        target_seq = shared.target_seq;
        follower.set_target(shared.target_p, shared.target_q);
        shared.joint_warning = 0;
      }
      if (shared.force_seq != force_seq) {
        force_seq = shared.force_seq;
        follower.set_force(shared.force);
      }
      leaving = leaving || shared.requested_mode != Mode::kFollow || !shared.running;
      lock.unlock();
    }
    if (shared.stop_requested.exchange(false) || leaving) {
      follower.stop();
    }
    if (const int joint = joint_approaching_limit(state, kFollowStopMargin)) {
      follower.stop();  // the server sees the warning and aborts a running sequence
      shared.joint_warning = joint;
    }
    const double dt = period.toSec() > 0.0 ? std::clamp(period.toSec(), 0.0005, 0.005) : 0.001;
    const auto ratios = joint_speed_ratios(state.q, state.dq);
    const double worst = *std::max_element(ratios.begin(), ratios.end());
    const double wanted = std::clamp((0.85 - worst) / (0.85 - kJointSpeedSoftLimit), 0.05, 1.0);
    governor += (wanted < governor ? 0.2 : 0.01) * (wanted - governor);  // brake fast, recover slowly
    const auto pose = follower.step(dt, shared.speed.load() * governor, state.O_F_ext_hat_K[2]);
    store_state(shared, Mode::kFollow, state, follower.status());
    if (leaving && follower.settled()) {
      return franka::MotionFinished(franka::CartesianPose(pose));
    }
    return franka::CartesianPose(pose);
  });
}

void run_joint_follow(franka::Robot& robot, Shared& shared) {
  using Vec7 = Eigen::Matrix<double, 7, 1>;
  Vec7 q_ref, q_f1, q_cmd, q_target, v = Vec7::Zero();
  bool first = true, leaving = false, holding = false;
  double governor = 1.0;
  uint64_t seq, path_seq;
  std::vector<std::array<double, 7>> path;  // the trajectory being played (swapped in, no alloc)
  path.reserve(1);
  double path_t = 0.0, path_rate = 1.0;
  bool path_active = false, path_stopping = false;
  {
    std::lock_guard<std::mutex> lock(shared.mutex);
    seq = shared.jtarget_seq;  // ignore targets / paths sent before this mode started
    path_seq = shared.path_seq;
  }
  shared.stop_requested = false;
  robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
    if (first) {
      // start at the last commanded joint positions (q_d); fall back to measured if they are off
      const Vec7 q_d = Eigen::Map<const Vec7>(state.q_d.data()), q = Eigen::Map<const Vec7>(state.q.data());
      const double off = (q_d - q).cwiseAbs().maxCoeff();
      q_ref = q_f1 = q_cmd = q_target = off < 0.01 ? q_d : q;
      std::snprintf(shared.start_info, sizeof(shared.start_info), "jfollow starts at %s (q_d %.4f rad from measured)",
                    off < 0.01 ? "q_d" : "q", off);
      shared.start_info_ready = true;
      first = false;
    }
    {
      std::unique_lock<std::mutex> lock(shared.mutex, std::try_to_lock);
      if (lock.owns_lock()) {
        if (shared.jtarget_seq != seq) {
          seq = shared.jtarget_seq;
          q_target = Eigen::Map<const Vec7>(shared.jtarget.data());
          holding = false;
          path_active = false;
          shared.joint_warning = 0;
        }
        if (shared.path_seq != path_seq) {
          path_seq = shared.path_seq;
          path.swap(shared.path_ready);  // O(1); the comm thread frees the old buffer later
          // a trajectory must start where we are (the server plans it from here)
          if (!path.empty() &&
              (Eigen::Map<const Vec7>(path.front().data()) - q_ref).cwiseAbs().maxCoeff() < 0.02) {
            path_active = true;
            path_stopping = false;
            path_t = 0.0;
            path_rate = 1.0;
            holding = false;
            shared.joint_warning = 0;
          }
        }
        leaving = leaving || shared.requested_mode != Mode::kJointFollow || !shared.running;
      }
    }
    const bool stop_now = shared.stop_requested.exchange(false) || leaving;
    const int near_limit = joint_approaching_limit(state, kFollowStopMargin);
    if (near_limit) {
      shared.joint_warning = near_limit;
    }
    if (stop_now || near_limit) {
      holding = true;
      path_stopping = true;
    }
    const double dt = period.toSec() > 0.0 ? std::clamp(period.toSec(), 0.0005, 0.005) : 0.001;
    const auto ratios = joint_speed_ratios(state.q, state.dq);
    const double worst = *std::max_element(ratios.begin(), ratios.end());
    const double wanted = std::clamp((0.85 - worst) / (0.85 - kJointSpeedSoftLimit), 0.05, 1.0);
    governor += (wanted < governor ? 0.2 : 0.01) * (wanted - governor);
    const double vmax = kJointMaxVelocity * shared.speed.load() * governor;

    if (path_active) {
      // play the trajectory; slowing the playback rate down keeps every derivative continuous
      const double wanted_rate = path_stopping ? 0.0 : std::min(1.0, governor);
      path_rate += std::clamp(wanted_rate - path_rate, -dt / kPathStopTime, dt / kPathStopTime);
      path_t += path_rate * dt;
      const double last = static_cast<double>(path.size() - 1);
      const double x = std::min(path_t / kPathDt, last);
      const size_t i0 = static_cast<size_t>(x);
      const size_t i1 = std::min(i0 + 1, path.size() - 1);
      const double f = x - static_cast<double>(i0);
      q_ref = (1.0 - f) * Eigen::Map<const Vec7>(path[i0].data()) + f * Eigen::Map<const Vec7>(path[i1].data());
      v.setZero();
      if (x >= last || (path_stopping && path_rate <= 0.0)) {
        path_active = false;
        q_target = q_ref;
      } else {
        q_target = Eigen::Map<const Vec7>(path.back().data());
      }
    }
    // planner: straight line in joint space, the fastest joint bounded (infinity norm)
    const Vec7 e = q_target - q_ref;
    const double dist = e.cwiseAbs().maxCoeff();
    Vec7 v_des = Vec7::Zero();
    if (!holding && !path_active && dist > 1e-7) {
      v_des = e / dist * std::min(vmax, std::sqrt(2.0 * kJointMaxAcceleration * dist));
    }
    Vec7 dv = v_des - v;
    const double dv_max = dv.cwiseAbs().maxCoeff();
    if (dv_max > kJointMaxAcceleration * dt) {
      dv *= kJointMaxAcceleration * dt / dv_max;
    }
    if (!path_active) {
      v += dv;
      q_ref += v * dt;
    }
    if (!path_active && !holding && dist < 1e-5 && v.cwiseAbs().maxCoeff() < 2.0 * kJointMaxAcceleration * dt) {
      q_ref = q_target;
      v.setZero();
    }
    if (holding && v.cwiseAbs().maxCoeff() < 1e-9) {
      q_target = q_ref;
    }
    // smoothing: two first-order filters (continuous acceleration)
    const Vec7 q_prev = q_cmd;
    const double a = std::min(1.0, dt / kSmoothingTime);
    q_f1 += a * (q_ref - q_f1);
    q_cmd += a * (q_f1 - q_cmd);
    // "remaining": time left on the trajectory plus the distance still to the target
    const double remaining = (path_active ? (path.size() - 1) * kPathDt - path_t : 0.0) +
                             (q_target - q_cmd).cwiseAbs().maxCoeff();
    const double speed = ((q_cmd - q_prev) / dt).cwiseAbs().maxCoeff();
    store_state(shared, Mode::kJointFollow, state, {remaining, 0.0, speed, 0.0, 1.0, 0.0, 0.0, 0.0});

    std::array<double, 7> out{};
    Eigen::Map<Vec7>(out.data()) = q_cmd;
    if (leaving && v.cwiseAbs().maxCoeff() < 1e-9 && (q_cmd - q_ref).cwiseAbs().maxCoeff() < 1e-7) {
      return franka::MotionFinished(franka::JointPositions(out));
    }
    return franka::JointPositions(out);
  });
}

void run_home(franka::Robot& robot, Shared& shared) {
  std::array<double, 7> q0{};
  double duration = 0.0, s = 0.0, rate = 1.0;  // s: progress in "motion time", rate: time scale
  bool first = true, stopping = false;
  shared.stop_requested = false;
  robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
    if (first) {
      q0 = state.q_d;  // the first command must be the last commanded position
      double max_delta = 0.0;
      for (size_t i = 0; i < 7; i++) {
        max_delta = std::max(max_delta, std::abs(kHomePose[i] - q0[i]));
      }
      // cosine profile: peak speed = pi/2 * delta / duration
      duration = std::max(1.0, M_PI_2 * max_delta / kHomeSpeed);
      first = false;
    }
    {
      std::unique_lock<std::mutex> lock(shared.mutex, std::try_to_lock);
      if (lock.owns_lock() && (shared.requested_mode != Mode::kHome || !shared.running)) {
        stopping = true;
      }
    }
    if (shared.stop_requested.exchange(false)) {
      stopping = true;
    }
    const double dt = period.toSec();
    if (stopping) {
      rate = std::max(0.0, rate - dt / kHomeStopTime);  // velocity scales with rate: smooth stop
    }
    s = std::min(duration, s + rate * dt);
    const double f = 0.5 * (1.0 - std::cos(M_PI * s / duration));
    std::array<double, 7> q{};
    for (size_t i = 0; i < 7; i++) {
      q[i] = q0[i] + f * (kHomePose[i] - q0[i]);
    }
    store_state(shared, Mode::kHome, state, {});
    franka::JointPositions out(q);
    if (s >= duration || (stopping && rate <= 0.0)) {
      return franka::MotionFinished(out);
    }
    return out;
  });
  std::lock_guard<std::mutex> lock(shared.mutex);
  if (shared.requested_mode == Mode::kHome) {
    shared.requested_mode = Mode::kIdle;  // done: hold there
  }
}

void run_free(franka::Robot& robot, Shared& shared) {
  std::array<double, 7> tau{};
  bool leaving = false;
  robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
    const double max_step = kMaxTorqueRate * std::clamp(period.toSec(), 0.0005, 0.005);
    std::unique_lock<std::mutex> lock(shared.mutex, std::try_to_lock);
    if (lock.owns_lock()) {
      leaving = leaving || shared.requested_mode != Mode::kFree || !shared.running;
      lock.unlock();
    }
    bool zero = true;
    const auto ratios = joint_speed_ratios(state.q, state.dq);
    for (size_t i = 0; i < 7; i++) {
      // extra damping only when a joint gets close to its current velocity limit
      const double excess = std::max(0.0, ratios[i] - kFreeDampingSoftLimit) / (1.0 - kFreeDampingSoftLimit);
      const double damping = kDamping[i] * (1.0 + 8.0 * excess);
      // soft wall before the ends of the range: a spring pushing back towards the middle
      double wall = 0.0;
      if (state.q[i] > kJointMax[i] - kFreeWallMargin) {
        wall = -kFreeWallStiffness * (state.q[i] - (kJointMax[i] - kFreeWallMargin));
      } else if (state.q[i] < kJointMin[i] + kFreeWallMargin) {
        wall = kFreeWallStiffness * ((kJointMin[i] + kFreeWallMargin) - state.q[i]);
      }
      const double desired = leaving ? 0.0
                                     : std::clamp(-damping * state.dq[i] + wall, -kMaxDampingTorque,
                                                  kMaxDampingTorque);
      tau[i] += std::clamp(desired - tau[i], -max_step, max_step);
      zero = zero && std::abs(tau[i]) < 1e-9;
    }
    store_state(shared, Mode::kFree, state, {});
    franka::Torques out(tau);
    if (leaving && zero) {
      return franka::MotionFinished(out);
    }
    return out;
  });
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip>" << std::endl;
    return 1;
  }
  Shared shared;
  std::thread comm(communication_thread, std::ref(shared));
  pin_current_thread(3, 3);  // the isolated core (isolcpus=3), shared only with the eth0 IRQs

  while (shared.running) {
    try {
      franka::Robot robot(argv[1]);
      std::cout << "connected to " << argv[1] << std::endl;
      set_default_collision_behavior(robot);
      while (shared.running) {
        const Mode mode = requested(shared);
        try {
          if (mode == Mode::kFollow) {
            run_follow(robot, shared);
          } else if (mode == Mode::kFree) {
            run_free(robot, shared);
          } else if (mode == Mode::kHome) {
            run_home(robot, shared);
          } else if (mode == Mode::kJointFollow) {
            run_joint_follow(robot, shared);
          } else {
            run_idle(robot, shared);
          }
        } catch (const franka::ControlException& e) {
          const std::string diagnosis = joint_diagnosis(e.log);
          std::cerr << "control error: " << e.what() << (diagnosis.empty() ? "" : " | ") << diagnosis
                    << std::endl;
          {
            std::lock_guard<std::mutex> lock(shared.state_mutex);
            shared.last_error = std::string(e.what()) + (diagnosis.empty() ? "" : " | " + diagnosis);
          }
          {
            std::lock_guard<std::mutex> lock(shared.mutex);
            shared.requested_mode = Mode::kIdle;  // stay idle until the server asks again
          }
          // Do not leave the robot locked in the reflex: clear it after a short pause. The
          // error text stays in the state messages until the next mode change.
          std::this_thread::sleep_for(std::chrono::duration<double>(kAutoRecoverDelay));
          try {
            robot.automaticErrorRecovery();
            std::cout << "recovered automatically" << std::endl;
          } catch (const franka::Exception& e2) {
            std::cerr << "automatic recovery failed: " << e2.what() << std::endl;
          }
        }
      }
    } catch (const franka::Exception& e) {
      std::cerr << "robot connection: " << e.what() << " - retrying" << std::endl;
      {
        std::lock_guard<std::mutex> lock(shared.state_mutex);
        shared.last_error = e.what();
        shared.have_state = false;
      }
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }
  comm.join();
  return 0;
}
