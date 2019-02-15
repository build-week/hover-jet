//%bin(controller_balsaq_main)
#include "control/controller_balsaq.hh"

#include <cstddef>
#include <iostream>

#include "control/control_utilities.hh"
#include "control/jet_vane_mapper.hh"
#include "control/servo_interface.hh"
#include "filtering/pose_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/engine/turbine_state_message.hh"

#include "third_party/experiments/estimation/time_point.hh"

namespace jet {
namespace control {

namespace {

estimation::TimePoint to_time_point(const Timestamp& ts) {
  const auto epoch_offset = std::chrono::nanoseconds(uint64_t(ts));
  const estimation::TimePoint time_point = estimation::TimePoint{} + epoch_offset;
  return time_point;
}

jcc::Vec3 sigmoid(const jcc::Vec3& v) {
  const double v_nrm = v.norm();

  const double interp_value = (1.0 / (1.0 + std::exp(-v_nrm)));

  return interp_value * v.normalized();
}

QuadraframeStatus generate_control(const SO3& world_from_target, const Pose& pose, const JetStatus& jet_status) {
  JetVaneMapper mapper_;

  const MatNd<3, 3> K = jcc::Vec3(0.3, 0.3, 0.4).asDiagonal();

  //
  // Compute the current expected jet force (All servos zero'd)
  //
  QuadraframeStatus all_zeroed;
  {
    all_zeroed.servo_0_angle_rad = 0.0;
    all_zeroed.servo_1_angle_rad = 0.0;
    all_zeroed.servo_2_angle_rad = 0.0;
    all_zeroed.servo_3_angle_rad = 0.0;
  }

  const auto wrench_for_zero = mapper_.wrench_for_status(all_zeroed, jet_status);

  const jcc::Vec3 desired_force_jet_frame = wrench_for_zero.force_N;

  const SO3 target_from_jet = world_from_target.inverse() * pose.world_from_jet.so3();
  const jcc::Vec3 desired_torque_jet_frame = -K * target_from_jet.log();

  Wrench target_wrench;
  target_wrench.torque_Nm = desired_torque_jet_frame;
  target_wrench.force_N = desired_force_jet_frame;

  constexpr double FORCE_WEIGHTING = 0.02;
  const auto result = mapper_.map_wrench(target_wrench, jet_status, FORCE_WEIGHTING);

  std::cout << "\n" << std::endl;
  std::cout << "Want (Torque Nm): " << target_wrench.torque_Nm.transpose() << std::endl;
  std::cout << "Want (Force N): " << target_wrench.force_N.transpose() << std::endl;
  std::cout << "Got (Torque Nm) : " << result.achieved_wrench.torque_Nm.transpose() << std::endl;
  std::cout << "Got (Force N) : " << result.achieved_wrench.force_N.transpose() << std::endl;

  const auto qframe_status = result.optimal_status;
  std::cout << qframe_status.servo_0_angle_rad << ", " << qframe_status.servo_1_angle_rad << ", "
            << qframe_status.servo_2_angle_rad << ", " << qframe_status.servo_3_angle_rad << ", " << std::endl;

  return result.optimal_status;
}
}  // namespace

ControllerBq::ControllerBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void ControllerBq::init(const Config& config) {
  std::cout << "Subscribing Roll" << std::endl;
  roll_sub_ = make_subscriber("joystick_roll");

  std::cout << "Subscribing Pitch" << std::endl;
  pitch_sub_ = make_subscriber("joystick_pitch");

  std::cout << "Subscribing Yaw" << std::endl;
  yaw_sub_ = make_subscriber("joystick_yaw");

  std::cout << "Subscribing pose" << std::endl;
  pose_sub_ = make_subscriber("pose");

  std::cout << "Advertising servos" << std::endl;
  servo_pub_ = make_publisher("servo_command_channel");

  std::cout << "Subscribing Turbine State" << std::endl;
  turbine_state_sub_ = make_subscriber("turbine_state");

  std::cout << "Controller starting" << std::endl;
}

void ControllerBq::loop() {
  PoseMessage pose_msg;

  TurbineStateMessage turbine_msg;
  while (turbine_state_sub_->read(turbine_msg, 1)) {
    jet_status_.throttle = static_cast<double>(turbine_msg.throttle_position_percent) * 0.01;
    got_turbine_status_ = true;
  }

  const auto target_from_world = target_from_joy(joy_pitch_, joy_roll_, joy_yaw_);

  // TODO
  got_turbine_status_ = true;
  jet_status_.throttle = 0.5;

  bool got_pose_msg = false;
  while (pose_sub_->read(pose_msg, 1)) {
    gonogo().go();
    got_pose_msg = true;
  }

  if (got_pose_msg && got_turbine_status_) {
    const Pose pose = pose_msg.to_pose();
    const auto target_qframe_status = generate_control(target_from_world, pose, jet_status_);
    SetServoMessage servo_message = create_servo_command(target_qframe_status);
    servo_pub_->publish(servo_message);
  }
}

void ControllerBq::shutdown() {
  std::cout << "Filter shutdown" << std::endl;
}

}  // namespace control
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::control::ControllerBq)
