//%bin(imu_balsaq_main)
#include "embedded/imu_driver/imu_balsaq.hh"
#include "embedded/imu_driver/imu_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "third_party/experiments/estimation/time_point.hh"

#include <cstddef>
#include <iostream>

namespace jet {
namespace embedded {

ImuBq::ImuBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void ImuBq::init() {
  publisher_ = make_publisher("imu");
  std::cout << "IMU starting" << std::endl;

  imu_driver_.initialize();
}

void ImuBq::loop() {
  ImuMessage msg;

  // msg.timestamp_ns = jcc::now().time_since_epoch().count();
  msg.timestamp = get_current_time();

  const jcc::Vec3 accel_mpss = imu_driver_.read_accel_mpss();
  const jcc::Vec3 angvel_radps = imu_driver_.read_gyro_radps();

  msg.accel_mpss_x = accel_mpss.x();
  msg.accel_mpss_y = accel_mpss.y();
  msg.accel_mpss_z = accel_mpss.z();

  msg.gyro_radps_x = angvel_radps.x();
  msg.gyro_radps_y = angvel_radps.y();
  msg.gyro_radps_z = angvel_radps.z();

  publisher_->publish(msg);
}

void ImuBq::shutdown() {
  std::cout << "IMU shutdown" << std::endl;
}

}  // namespace embedded
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::embedded::ImuBq)
