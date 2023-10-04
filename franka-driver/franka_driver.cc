#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_throw.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/text_logging.h"
#include "drake/lcmt_panda_command.hpp"
#include "drake/lcmt_panda_status.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"

DEFINE_string(robot_ip_address, "", "Address of the shop floor interface");
DEFINE_string(lcm_url, "", "LCM URL for Panda driver");
DEFINE_string(lcm_command_channel, "PANDA_COMMAND",
              "Channel to listen for lcmt_panda_command messages on");
DEFINE_string(lcm_status_channel, "PANDA_STATUS",
              "Channel to publish lcmt_panda_status messages on");
DEFINE_double(joint_torque_limit, 100.0, "Joint torque limit");
DEFINE_double(cartesian_force_limit, 100.0, "Cartesian force/torque limit");
DEFINE_string(
    control_mode, "velocity",
    "Choose from: status_only, velocity (default), position, torque");
DEFINE_bool(
    latch, false, "Latch previous command if no new command has arrived");
DEFINE_double(
    expire_sec, 0.1,
    "How much delay is allowed for messages to be allowed. Converted to "
    "usec, must be non-negative + finite.");
DEFINE_double(
    low_pass_freq, 30.0,
    "Low-pass cutoff frequency (Hz) for desired position or velocity.");
DEFINE_double(
    diff_low_pass_freq, 30.0,
    "Low-pass cutoff frequency (Hz) for computing desired velocities and "
    "accelerations via finite differencing. Note that accelerations are "
    "filtered after being computed from *filtered* velocities.");

// Options shared for any mode producing torques / generalized forces.
DEFINE_double(
    torque_kp_scale, 1.0,
    "Scale the kp term for computing tau_fb_q = -kp * (q_actual - q_desired)");
DEFINE_double(
    torque_kd_scale, 1.0,
    "Scale the kd term for computing tau_fb_v = -kd * (v_actual - v_desired)");
DEFINE_bool(
    use_mbp, false,
    "Use Drake MbP for dynamics computation, rather than Franka's inbuilt "
    "model.");
DEFINE_string(mbp_model_runpath,
              "drake_franka_driver/models/add_franka_control.yaml",
              "Model file to use.");

// Options for --control_mode=position --use_torque_for_position=true
DEFINE_bool(
    use_torque_for_position, false,
    R"""(
Use custom torque controller for position control. This uses commanded
position, and then finite differences the position to velocity.

More conceretely, it looks like this:
  torque_command =
      -kp * (q_actual - q_desired) - kd * (v_actual - v_desired)
      + inertia_matrix(q_actual) * a_desired + coriolis(q_actual, v_actual)
)""");
DEFINE_bool(
    torque_zero_desired_velocity, false,
    "Set desired velocity to zero (do not use finite differencing).");
DEFINE_double(
    torque_coriolis_scale, 1.0,
    "Scaling for Coriolis feedforward term in torque.");
DEFINE_double(
    torque_inertia_scale, 1.0,
    "Scaling for inertia_matrix matrix feedforward term. This uses a finite "
    "differencing of the desired velocity (which seems fine enough for "
    "this control).");
DEFINE_bool(
    torque_feedback_in_acceleration, false,
    "Rather than feedback in torque, meaning gains must account for link "
    "inertias, compute feedback in acceleration, and project through "
    "inertia_matrix matrix. This should have --torque_inerta_scale=1, and "
    "replaces the desired acceleration with this feedback term.");

// Options for --control_mode=torque
DEFINE_bool(
    remove_gravity_compensation, false,
    "If you send torques to FCI, then you should *not* include gravity (or "
    "friction) compenstation terms, as they handle that on the robot itself. "
    "Setting this to true (default) will subtract this driver's internal "
    "computed gravity terms from the desired torques. Note that "
    "--use_mbp=true means that gravity compenstation will use a Drake-"
    "supplied model. Otherwise, Franka's internal Panda model is used.");

namespace robotlocomotion {
namespace franka_driver {
namespace {

namespace sp = std::placeholders;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::parsing::LoadModelDirectives;
using drake::systems::Context;

// dest, src syntax like memcpy
template <typename T, std::size_t N>
void CopyArrayToVector(std::vector<T>* dest, const std::array<T, N>& src) {
  dest->resize(N);
  memcpy(dest->data(), src.data(), N * sizeof(T));
}

template <typename T, std::size_t N>
std::string PrintArray(const std::string& name, std::array<T, N> data) {
  return
      name + ": " + /*common_robotics_utilities::print::Print(data, true) +*/ "\n";
}

constexpr int kNdof = 7;

enum class ControlMode {
  /** Only publish status */
  kStatusOnly,
  /** Command velocities via Joint impedance */
  kVelocity,
  /** Command positions and velocities via Joint impedance */
  kPosition,
  /** Command torques directly */
  kTorque,
};

ControlMode ToControlMode(std::string value) {
  if (value == "status_only") {
    return ControlMode::kStatusOnly;
  } else if (value == "velocity") {
    return ControlMode::kVelocity;
  } else if (value == "position") {
    return ControlMode::kPosition;
  } else if (value == "torque") {
    return ControlMode::kTorque;
  } else {
    throw std::runtime_error("Invalid ControlMode: " + value);
  }
}

std::string PrintRobotState(const franka::RobotState& state) {
  const std::string indent = "  ";
  std::string str_rep;
  str_rep += indent + PrintArray("q", state.q);
  str_rep += indent + PrintArray("q_d", state.q_d);
  str_rep += indent + PrintArray("dq", state.dq);
  str_rep += indent + PrintArray("dq_d", state.dq_d);
  str_rep += indent + PrintArray("ddq_d", state.ddq_d);
  str_rep += indent + PrintArray("theta", state.theta);
  str_rep += indent + PrintArray("dtheta", state.dtheta);
  str_rep += indent + PrintArray("tau_J", state.tau_J);
  str_rep += indent + PrintArray("tau_J_d", state.tau_J_d);
  str_rep += indent + PrintArray("dtau_J", state.dtau_J);
  str_rep += indent + PrintArray(
      "tau_ext_hat_filtered", state.tau_ext_hat_filtered);
  str_rep += indent + PrintArray("joint_contact", state.joint_contact);
  str_rep += indent + PrintArray("joint_contact", state.joint_collision);
  str_rep += indent + PrintArray("cartesian_contact", state.cartesian_contact);
  str_rep += indent + PrintArray(
      "cartesian_collision", state.cartesian_collision);

  str_rep += indent + "errors: " + std::string(state.current_errors) + "\n";
  str_rep += indent + "control success rate: "
          + std::to_string(state.control_command_success_rate) + "\n";
  str_rep += indent + "mode: "
          + std::to_string(static_cast<int>(state.robot_mode)) + "\n";
  str_rep += indent + "time: " + std::to_string(state.time.toSec());

  return str_rep;
}

void SetGainsForJointStiffnessErrorToTorque(
    Eigen::VectorXd* kp, Eigen::VectorXd* kd) {
  DRAKE_DEMAND(kp != nullptr || kd != nullptr);
  // Taken originally from libfranka's joint_impedance_control demo, then
  // tweaked with empircal testing based on tracking and stability with
  // multi_frame_pose_stream based teleop.
  // TODO(eric.cousineau): Refine further using tools / investigation from
  // #9183.
  if (kp != nullptr) {
    DRAKE_DEMAND(kp->size() == kNdof);
    *kp << 875.0, 1050.0, 1050.0, 875.0, 175.0, 350.0, 87.5;
    *kp *= FLAGS_torque_kp_scale;
  }
  if (kd != nullptr) {
    DRAKE_DEMAND(kd->size() == kNdof);
    *kd << 37.5, 50.0, 37.5, 25.0, 5.0, 3.75, 2.5;
    *kd *= FLAGS_torque_kd_scale;
  }
}

class PandaDriver {
 public:
  PandaDriver(
      const std::string& robot_ip_address,
      const std::string& lcm_url,
      const std::string& lcm_command_channel,
      const std::string& lcm_status_channel,
      double joint_torque_limit,
      double cartesian_force_limit,
      bool latch,
      std::unique_ptr<MultibodyPlant<double>> plant,
      uint32_t expire_usec,
      bool remove_gravity_compensation)
      : robot_(robot_ip_address, franka::RealtimeConfig::kIgnore),
        model_(robot_.loadModel()),
        lcm_(lcm_url),
        lcm_command_channel_(lcm_command_channel),
        lcm_status_channel_(lcm_status_channel),
        latch_(latch),
        expire_usec_(expire_usec),
        joint_torque_limit_(joint_torque_limit),
        cartesian_force_limit_(cartesian_force_limit),
        remove_gravity_compensation_(remove_gravity_compensation),
        plant_(std::move(plant)) {
    drake::log()->info(
        "Connected to Panda arm version {}", robot_.serverVersion());

    const auto state = robot_.readOnce();

    DRAKE_DEMAND(state.q.size() == kNdof);
    status_msg_.num_joints = kNdof;
    status_msg_.joint_position.resize(kNdof, 0.);
    status_msg_.joint_position_desired.resize(kNdof, 0.);
    status_msg_.joint_velocity.resize(kNdof, 0.);
    status_msg_.joint_velocity_desired.resize(kNdof, 0.);
    status_msg_.joint_velocity_desired.resize(kNdof, 0.);
    status_msg_.joint_acceleration_desired.resize(kNdof, 0.);
    status_msg_.joint_torque.resize(kNdof, 0.);
    status_msg_.joint_torque_desired.resize(kNdof, 0.);
    status_msg_.joint_torque_external.resize(kNdof, 0.);

    // TODO(eric.cousineau): Will the driver have the effectively correct q_d?
    q_cmd_latest_.resize(kNdof);
    for (size_t i = 0; i < kNdof; ++i) {
      q_cmd_latest_(i) = state.q[i];
    }
    v_cmd_latest_.resize(kNdof);
    v_cmd_latest_.setZero();
    a_cmd_latest_.resize(kNdof);
    a_cmd_latest_.setZero();

    lcm::Subscription* sub = lcm_.subscribe(
        lcm_command_channel_, &PandaDriver::HandleCommandMessage, this);
    sub->setQueueCapacity(100);

    // Preallocate.
    coriolis_vector_.resize(kNdof);
    inertia_matrix_.resize(kNdof, kNdof);
    gravity_vector_.resize(kNdof);
    if (plant_) {
      DRAKE_DEMAND(plant_->num_positions() == kNdof);
      DRAKE_DEMAND(plant_->num_velocities() == kNdof);
      context_ = plant_->CreateDefaultContext();
    }
  }

  void ControlLoop(ControlMode mode) {
    try {
      const bool limit_rate = true;
      switch (mode) {
        case ControlMode::kStatusOnly: {
          robot_.read(std::bind(&PandaDriver::DoStateRead, this, sp::_1));
          break;
        }
        case ControlMode::kVelocity: {
          DoNonRealtimeControlSetup();
          robot_.control(
              std::bind(&PandaDriver::DoVelocityControl, this, sp::_1, sp::_2),
              franka::ControllerMode::kJointImpedance,
              limit_rate, FLAGS_low_pass_freq);
          break;
        }
        case ControlMode::kPosition: {
          DoNonRealtimeControlSetup();
          if (FLAGS_use_torque_for_position) {
            robot_.control(
                std::bind(
                    &PandaDriver::DoPositionControlViaTorque,
                    this, sp::_1, sp::_2),
                limit_rate, franka::kDefaultCutoffFrequency);
          } else {
            robot_.control(
                std::bind(
                    &PandaDriver::DoPositionControl,
                    this, sp::_1, sp::_2),
                franka::ControllerMode::kJointImpedance,
                limit_rate, FLAGS_low_pass_freq);
          }
          break;
        }
        case ControlMode::kTorque: {
          DoNonRealtimeControlSetup();
          robot_.control(
              std::bind(
                  &PandaDriver::DoTorqueControl,
                  this, sp::_1, sp::_2),
              limit_rate, FLAGS_low_pass_freq);
          break;
        }
        default: DRAKE_DEMAND(false);
      }
    } catch (const std::exception& ex) {
      drake::log()->error(
          "Control loop exception: [{}]\n"
          "Last control robot state:\n{}\n"
          "Current robot state:\n{}\n",
          ex.what(),
          PrintRobotState(state_latest_),
          PrintRobotState(robot_.readOnce()));
      command_.reset();
      if (mode != ControlMode::kStatusOnly) {
        drake::log()->error("Running recovery");
        robot_.automaticErrorRecovery();
      }
    }
  }

 private:
  bool DoStateRead(const franka::RobotState& state) {
    PublishRobotState(state);

    if (is_first_tick_) {
      is_first_tick_ = false;
    }

    return true;
  }

  void DoNonRealtimeControlSetup() {
    // Values originally from setDefaultBehavior in libfranka example code
    std::array<double, kNdof> joint_torque_limits;
    joint_torque_limits.fill(joint_torque_limit_);

    std::array<double, 6> cartesian_force_limits;
    cartesian_force_limits.fill(cartesian_force_limit_);

    robot_.setCollisionBehavior(
        joint_torque_limits,
        joint_torque_limits,
        cartesian_force_limits,
        cartesian_force_limits);
    robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
  }

  franka::JointVelocities DoVelocityControl(
      const franka::RobotState& state, franka::Duration)  {
    PublishRobotState(state);

    // Poll for incoming command messages.
    while (lcm_.handleTimeout(0) > 0) {}

    if (!command_ && latch_) {
      command_ = command_prev_;
    }

    franka::JointVelocities v({0, 0, 0, 0, 0, 0, 0});
    if (!command_) {
      return v;
    }

    DRAKE_THROW_UNLESS(
        command_->control_mode_expected ==
        drake::lcmt_panda_status::CONTROL_MODE_VELOCITY);
    DRAKE_THROW_UNLESS(command_->num_joint_position == 0);
    DRAKE_THROW_UNLESS(command_->num_joint_torque == 0);

    const int64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    if (std::abs(now - command_->utime) > expire_usec_) {
      drake::log()->warn("Resetting state velocity command to zero");
      command_.reset();
      command_prev_.reset();
      return v;
    }

    if (command_->num_joint_velocity != std::ssize(state.q)) {
      throw std::runtime_error(
          "Received command with unexpected num_joint_velocity");
    }

    for (int i = 0; i < command_->num_joint_velocity; ++i) {
      v.dq[i] = command_->joint_velocity[i];
    }

    if (is_first_tick_) {
      is_first_tick_ = false;
    }

    command_prev_ = command_;
    return v;
  }

  franka::JointPositions DoPositionControl(
      const franka::RobotState& state, franka::Duration)  {
    PublishRobotState(state);

    // Poll for incoming command messages.
    while (lcm_.handleTimeout(0) > 0) {}

    if (!command_ && latch_) {
      command_ = command_prev_;
    }

    franka::JointPositions q({0, 0, 0, 0, 0, 0, 0});
    for (size_t i = 0; i < q.q.size(); ++i) {
      q.q[i] = q_cmd_latest_(i);
    }

    if (!command_) {
      return q;
    }

    DRAKE_THROW_UNLESS(
        command_->control_mode_expected ==
        drake::lcmt_panda_status::CONTROL_MODE_POSITION);
    DRAKE_THROW_UNLESS(command_->num_joint_velocity == 0);
    DRAKE_THROW_UNLESS(command_->num_joint_torque == 0);

    const int64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    if (std::abs(now - command_->utime) > expire_usec_) {
      drake::log()->warn(
          "Leaving state position command at last known command");
      command_.reset();
      command_prev_.reset();
      return q;
    }

    if (command_->num_joint_position != std::ssize(state.q)) {
      throw std::runtime_error(
          "Received command with unexpected num_joint_position");
    }

    for (int i = 0; i < command_->num_joint_position; ++i) {
      q.q[i] = command_->joint_position[i];
      q_cmd_latest_(i) = q.q[i];
    }

    if (is_first_tick_) {
      is_first_tick_ = false;
    }

    command_prev_ = command_;
    return q;
  }

  franka::Torques DoPositionControlViaTorque(
      const franka::RobotState& state, franka::Duration period)  {
    PublishRobotState(state);

    // Poll for incoming command messages.
    while (lcm_.handleTimeout(0) > 0) {}

    const double dt = period.toSec();
    if (is_first_tick_) {
      // First tick should indicate zero dt.
      DRAKE_DEMAND(dt == 0.0);
      // Initialize command to libfranka's reported last command.
      for (size_t i = 0; i < state.q.size(); ++i) {
        q_cmd_latest_(i) = state.q_d[i];
      }
    } else {
      // N.B. The period from libfranka is in multiples of 1ms.
      DRAKE_DEMAND(dt > 0.0);
    }

    if (!command_ && latch_) {
      command_ = command_prev_;
    }

    Eigen::VectorXd q_cmd_raw(kNdof);

    if (command_) {
      command_prev_ = command_;

      DRAKE_THROW_UNLESS(
          command_->control_mode_expected ==
          drake::lcmt_panda_status::CONTROL_MODE_POSITION);
      DRAKE_THROW_UNLESS(command_->num_joint_velocity == 0);
      DRAKE_THROW_UNLESS(command_->num_joint_torque == 0);
      if (command_->num_joint_position != std::ssize(state.q)) {
        throw std::runtime_error(
            "Received command with unexpected num_joint_position");
      }
      for (int i = 0; i < command_->num_joint_position; ++i) {
        q_cmd_raw(i) = command_->joint_position[i];
      }
    } else {
      // Use previous.
      q_cmd_raw = q_cmd_latest_;
    }

    // Read actual positions and velocities.
    Eigen::VectorXd q_actual(kNdof), v_actual(kNdof);
    for (size_t i = 0; i < kNdof; ++i) {
      q_actual[i] = state.q[i];
      v_actual[i] = state.dq[i];
    }

    // Record previous for finite differencing.
    const Eigen::VectorXd q_cmd_prev = q_cmd_latest_;
    if (is_first_tick_) {
      q_cmd_latest_ = q_cmd_raw;
    } else {
      // Filter commands if enabled.
      if (FLAGS_low_pass_freq < franka::kMaxCutoffFrequency) {
        for (int i = 0; i < kNdof; ++i) {
          q_cmd_latest_[i] = franka::lowpassFilter(
              dt, q_cmd_raw[i], q_cmd_prev[i], FLAGS_low_pass_freq);
        }
      } else {
        q_cmd_latest_ = q_cmd_raw;
      }
    }

    if (is_first_tick_) {
      // Use zero value.
      v_cmd_latest_.setZero();
      a_cmd_latest_.setZero();
    } else {
      // Compute simple finite differencing.
      const Eigen::VectorXd v_cmd_prev = v_cmd_latest_;
      const Eigen::VectorXd v_cmd_raw = (q_cmd_latest_ - q_cmd_prev) / dt;
      for (int i = 0; i < kNdof; ++i) {
        v_cmd_latest_[i] = franka::lowpassFilter(
              dt, v_cmd_raw[i], v_cmd_latest_[i], FLAGS_diff_low_pass_freq);
      }
      const Eigen::VectorXd a_cmd_raw = (v_cmd_latest_ - v_cmd_prev) / dt;
      for (int i = 0; i < kNdof; ++i) {
        a_cmd_latest_[i] = franka::lowpassFilter(
              dt, a_cmd_raw[i], a_cmd_latest_[i], FLAGS_diff_low_pass_freq);
      }
    }

    // Feedback gains.
    Eigen::VectorXd kp(kNdof), kd(kNdof);
    kp.setZero();
    kd.setZero();
    if (FLAGS_torque_feedback_in_acceleration) {
      // Tested empirically. Still needs work!
      // TODO(eric.cousineau): Do more targeted testing using tools /
      // investigation from #9183.
      kp.setConstant(1000.0);
      kd.setConstant(45.0);
    } else {
      SetGainsForJointStiffnessErrorToTorque(&kp, &kd);
    }

    // Feedback term.
    Eigen::VectorXd v_desired = v_cmd_latest_;
    if (FLAGS_torque_zero_desired_velocity) {
      v_desired.setZero();
    }
    Eigen::VectorXd feedback =
        -kp.array() * (q_actual - q_cmd_latest_).array()
        - kd.array() * (v_actual - v_desired).array();

    if (plant_) {
      DRAKE_DEMAND(context_ != nullptr);
      // Update context.
      plant_->SetPositions(context_.get(), q_actual);
      plant_->SetVelocities(context_.get(), v_actual);
      // Compute values.
      plant_->CalcBiasTerm(*context_, &coriolis_vector_);
      plant_->CalcMassMatrix(*context_, &inertia_matrix_);
    } else {
      // Compute Coriolis and inerta terms from Franka model.
      const std::array<double, kNdof> coriolis_array = model_.coriolis(state);
      coriolis_vector_ = Eigen::VectorXd::Map(&coriolis_array[0], kNdof);
      const std::array<double, kNdof * kNdof> inertia_array =
          model_.mass(state);
      inertia_matrix_ = Eigen::MatrixXd::Map(&inertia_array[0], kNdof, kNdof);
    }

    coriolis_vector_ *= FLAGS_torque_coriolis_scale;
    inertia_matrix_ *= FLAGS_torque_inertia_scale;

    // N.B. Franka docs say that `franka::Torques` should indicate desired
    // torques without gravity or friction.
    Eigen::VectorXd tau_cmd(kNdof);
    if (FLAGS_torque_feedback_in_acceleration) {
      // Project feedback (in acceleration) to torques, add feedforward
      // coriolis.
      // TODO(eric.cousineau): Still provide option to add feedfoward inertia
      // terms?
      tau_cmd = inertia_matrix_ * feedback + coriolis_vector_;
    } else {
      // Use torques directly, add feedforward based on command acceleration.
      tau_cmd = feedback + inertia_matrix_ * a_cmd_latest_ + coriolis_vector_;
    }

    if (is_first_tick_) {
      is_first_tick_ = false;
    }

    std::array<double, kNdof> tau_cmd_array{};
    Eigen::VectorXd::Map(&tau_cmd_array[0], kNdof) = tau_cmd;
    return franka::Torques(tau_cmd_array);
  }

  franka::Torques DoTorqueControl(
      const franka::RobotState& state, franka::Duration period)  {
    PublishRobotState(state);

    // Poll for incoming command messages.
    while (lcm_.handleTimeout(0) > 0) {}

    if (!command_ && latch_) {
      command_ = command_prev_;
    }

    const int64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    if (command_ && std::abs(now - command_->utime) > expire_usec_) {
      drake::log()->warn(
          "Torque command expiration! Engaging gravity compensation and "
          "damping controller.");
      command_.reset();
      command_prev_.reset();
    }

    // Read actual positions and velocities.
    Eigen::VectorXd q_actual(kNdof), v_actual(kNdof);
    for (size_t i = 0; i < kNdof; ++i) {
      q_actual[i] = state.q[i];
      v_actual[i] = state.dq[i];
    }

    if (!command_) {
      // Gravity compensation and damping controller.
      Eigen::VectorXd kd(kNdof);
      SetGainsForJointStiffnessErrorToTorque(nullptr, &kd);
      const Eigen::VectorXd damping = -kd.array() * v_actual.array();
      std::array<double, kNdof> tau_cmd_array{};
      Eigen::VectorXd::Map(&tau_cmd_array[0], kNdof) = damping;
      return franka::Torques(tau_cmd_array);
    }

    DRAKE_THROW_UNLESS(
        command_->control_mode_expected ==
        drake::lcmt_panda_status::CONTROL_MODE_TORQUE);
    DRAKE_THROW_UNLESS(command_->num_joint_position == 0);
    DRAKE_THROW_UNLESS(command_->num_joint_velocity == 0);
    DRAKE_THROW_UNLESS(command_->num_joint_torque == kNdof);
    Eigen::VectorXd tau_cmd(kNdof);
    for (size_t i = 0; i < kNdof; ++i) {
      tau_cmd(i) = command_->joint_torque[i];
    }

    if (remove_gravity_compensation_) {
      if (plant_) {
        plant_->SetPositions(context_.get(), q_actual);
        plant_->SetVelocities(context_.get(), v_actual);
        gravity_vector_ = plant_->CalcGravityGeneralizedForces(*context_);
      } else {
        const std::array<double, kNdof> gravity_array = model_.gravity(state);
        gravity_vector_ = Eigen::VectorXd::Map(&gravity_array[0], kNdof);
      }
      const Eigen::VectorXd gravity_compensation = -gravity_vector_;
      tau_cmd -= gravity_compensation;
    }

    std::array<double, kNdof> tau_cmd_array{};
    Eigen::VectorXd::Map(&tau_cmd_array[0], kNdof) = tau_cmd;
    return franka::Torques(tau_cmd_array);
  }

  void HandleCommandMessage(
      const lcm::ReceiveBuffer*, const std::string&,
      const drake::lcmt_panda_command* command) {
    command_ = *command;
  }

  void PublishRobotState(const franka::RobotState& state) {
    state_latest_ = state;
    status_msg_.utime = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    CopyArrayToVector(&status_msg_.joint_position, state.q);
    CopyArrayToVector(&status_msg_.joint_position_desired, state.q_d);
    CopyArrayToVector(&status_msg_.joint_velocity, state.dq);
    CopyArrayToVector(&status_msg_.joint_velocity_desired, state.dq_d);
    CopyArrayToVector(&status_msg_.joint_acceleration_desired, state.ddq_d);
    CopyArrayToVector(&status_msg_.joint_torque, state.tau_J);
    CopyArrayToVector(&status_msg_.joint_torque_desired, state.tau_J_d);
    CopyArrayToVector(
        &status_msg_.joint_torque_external, state.tau_ext_hat_filtered);
    status_msg_.control_command_success_rate =
        state.control_command_success_rate;
    switch (state.robot_mode) {
      case franka::RobotMode::kOther: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kOther;
        break;
      }
      case franka::RobotMode::kIdle: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kIdle;
        break;
      }
      case franka::RobotMode::kMove: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kMove;
        break;
      }
      case franka::RobotMode::kGuiding: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kGuiding;
        break;
      }
      case franka::RobotMode::kReflex: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kReflex;
        break;
      }
      case franka::RobotMode::kUserStopped: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kUserStopped;
        break;
      }
      case franka::RobotMode::kAutomaticErrorRecovery: {
        status_msg_.robot_mode =
            drake::lcmt_panda_status::kAutomaticErrorRecovery;
        break;
      }
    }

    status_msg_.robot_utime = state.time.toMSec() * 1000;
    lcm_.publish(lcm_status_channel_, &status_msg_);
  }

  franka::Robot robot_;
  franka::Model model_;
  lcm::LCM lcm_;
  const std::string lcm_command_channel_;
  const std::string lcm_status_channel_;
  const bool latch_{};
  const uint32_t expire_usec_{};
  const double joint_torque_limit_{};
  const double cartesian_force_limit_{};

  bool is_first_tick_{true};
  Eigen::VectorXd q_cmd_latest_;
  Eigen::VectorXd v_cmd_latest_;
  Eigen::VectorXd a_cmd_latest_;
  franka::RobotState state_latest_;

  drake::lcmt_panda_status status_msg_{};
  std::optional<drake::lcmt_panda_command> command_;
  std::optional<drake::lcmt_panda_command> command_prev_;

  Eigen::VectorXd coriolis_vector_;
  Eigen::MatrixXd inertia_matrix_;
  Eigen::VectorXd gravity_vector_;

  bool remove_gravity_compensation_{};

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;
};

// N.B. Using a resource path allows us to locate
// any Bazel resource, be it Drake or another repository.
std::string GetPathOrThrow(const drake::RlocationOrError& result) {
  if (!result.error.empty()) {
    throw std::runtime_error(result.error);
  }
  DRAKE_DEMAND(!result.abspath.empty());
  return result.abspath;
}

std::unique_ptr<MultibodyPlant<double>> MaybeLoadPlant() {
  if (!FLAGS_use_mbp) {
    return nullptr;
  }
  const std::string model_file =
        GetPathOrThrow(drake::FindRunfile(FLAGS_mbp_model_runpath));
  const double time_step = 0.0;
  auto plant = std::make_unique<MultibodyPlant<double>>(time_step);
  drake::multibody::Parser parser(plant.get());
  drake::multibody::parsing::ModelDirectives directives = 
        drake::multibody::parsing::LoadModelDirectives(model_file);
  drake::multibody::parsing::ProcessModelDirectives(directives, plant.get(), 
                                                nullptr, &parser);
  plant->Finalize();
  return plant;
}

int DoMain() {
  DRAKE_THROW_UNLESS(FLAGS_robot_ip_address != "");
  DRAKE_THROW_UNLESS(FLAGS_lcm_command_channel != "");
  DRAKE_THROW_UNLESS(FLAGS_lcm_status_channel != "");
  DRAKE_THROW_UNLESS(FLAGS_joint_torque_limit > 0.0);
  DRAKE_THROW_UNLESS(FLAGS_cartesian_force_limit > 0.0);
  DRAKE_THROW_UNLESS(
      FLAGS_expire_sec >= 0.0 && std::isfinite(FLAGS_expire_sec));

  const ControlMode mode = ToControlMode(FLAGS_control_mode);

  const uint32_t expire_usec =
      static_cast<uint32_t>(FLAGS_expire_sec * 1e6);
  PandaDriver driver(
      FLAGS_robot_ip_address,
      FLAGS_lcm_url,
      FLAGS_lcm_command_channel,
      FLAGS_lcm_status_channel,
      FLAGS_joint_torque_limit,
      FLAGS_cartesian_force_limit,
      FLAGS_latch,
      MaybeLoadPlant(),
      expire_usec,
      FLAGS_remove_gravity_compensation);

  driver.ControlLoop(mode);

  return 0;
}
}  // namespace
} // namespace franka_driver
} // namespace robotlocomotion

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return robotlocomotion::franka_driver::DoMain();
}
