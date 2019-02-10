//%bin(controller_balsaq_main)
#include "control/controller_balsaq.hh"

#include <cstddef>
#include <iostream>

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

QuadraframeStatus generate_control(const Pose& pose, const JetStatus& jet_status) {
  JetVaneMapper mapper_;

  const SO3 world_from_target;  // Identity! Orient up!

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

  const double gain_p = 0.1;
  const SO3 target_from_jet = world_from_target.inverse() * pose.world_from_jet.so3();
  const jcc::Vec3 desired_torque_jet_frame = gain_p * target_from_jet.log();

  Wrench target_wrench;
  target_wrench.torque_Nm = desired_torque_jet_frame;
  target_wrench.force_N = desired_force_jet_frame;

  const auto result = mapper_.map_wrench(target_wrench, jet_status);

  std::cout << "Want (Torque Nm): " << target_wrench.torque_Nm.transpose() << std::endl;
  std::cout << "Got (Torque Nm) : " << result.achieved_wrench.torque_Nm.transpose() << std::endl;

  return result.optimal_status;
}
}  // namespace

ControllerBq::ControllerBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void ControllerBq::init(int argc, char* argv[]) {
  std::cout << "Subscribing Roll" << std::endl;
  roll_sub_ = make_subscriber("roll");

  std::cout << "Subscribing Pitch" << std::endl;
  pitch_sub_ = make_subscriber("pitch");

  std::cout << "Subscribing Yaw" << std::endl;
  yaw_sub_ = make_subscriber("Yaw");

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

  bool got_pose_msg = false;
  while (pose_sub_->read(pose_msg, 1)) {
    got_pose_msg = true;
  }

  if (got_pose_msg && got_turbine_status_) {
    const Pose pose = pose_msg.to_pose();
    const auto target_qframe_status = generate_control(pose, jet_status_);
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
