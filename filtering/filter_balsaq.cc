//%bin(filter_balsaq_main)
#include "filtering/filter_balsaq.hh"

#include "embedded/imu_driver/imu_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "vision/fiducial_detection_message.hh"

#include <cstddef>
#include <iostream>

#include "third_party/experiments/estimation/time_point.hh"

// %deps(simple_geometry)
// %deps(window_3d)
#include "third_party/experiments/viewer/primitives/simple_geometry.hh"
#include "third_party/experiments/viewer/window_3d.hh"

namespace jet {
namespace embedded {

FilterBq::FilterBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void FilterBq::init(int argc, char *argv[]) {
  const auto view = viewer::get_window3d("Filter Debug");
  geo_ = view->add_primitive<viewer::SimpleGeometry>();

  persistent_ = view->add_primitive<viewer::SimpleGeometry>();

  imu_sub_ = make_subscriber("imu");
  fiducial_sub_ = make_subscriber("fiducial_detection_channel");
  std::cout << "Filter starting" << std::endl;
}

void FilterBq::loop() {
  const auto view = viewer::get_window3d("Filter Debug");
  ImuMessage imu_msg;

  FiducialDetectionMessage detetection_msg;
  if (fiducial_sub_->read(detetection_msg, 1)) {
    std::cout << "got msg" << std::endl;
  }

  if (imu_sub_->read(imu_msg, 1)) {
    const jcc::Vec3 accel_mpss(imu_msg.accel_mpss_x, imu_msg.accel_mpss_y,
                               imu_msg.accel_mpss_z);

    const jcc::Vec3 mag_utesla(imu_msg.mag_utesla_x, imu_msg.mag_utesla_y,
                               imu_msg.mag_utesla_z);
    persistent_->add_point({mag_utesla});
    persistent_->flush();

    const jcc::Vec3 gyro_radps(imu_msg.gyro_radps_x, imu_msg.gyro_radps_y,
                               imu_msg.gyro_radps_z);

    geo_->add_sphere({jcc::Vec3::Zero(), 9.81});
    geo_->add_line({jcc::Vec3::Zero(), accel_mpss, jcc::Vec4(0.0, 1.0, 0.0, 0.8)});

    geo_->add_line({jcc::Vec3::Zero(), gyro_radps, jcc::Vec4(1.0, 0.0, 0.0, 0.8)});
    geo_->flip();
  }
}

void FilterBq::shutdown() {
  std::cout << "Filter shutdown" << std::endl;
}

}  // namespace embedded
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::embedded::FilterBq)
