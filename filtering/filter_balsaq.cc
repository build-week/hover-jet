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

//%deps(fit_ellipse)
#include "third_party/experiments/geometry/shapes/fit_ellipse.hh"

namespace jet {
namespace embedded {

FilterBq::FilterBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void FilterBq::init(int argc, char* argv[]) {
  const auto view = viewer::get_window3d("Filter Debug");
  view->set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), jcc::Vec3(-1.0, 0.0, -1.0)));
  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground, 0.1});
  background->flip();

  geo_ = view->add_primitive<viewer::SimpleGeometry>();
  persistent_ = view->add_primitive<viewer::SimpleGeometry>();

  std::cout << "Subscribing IMU" << std::endl;
  imu_sub_ = make_subscriber("imu");
  std::cout << "Subscribing Fiducial" << std::endl;
  fiducial_sub_ = make_subscriber("fiducial_detection_channel");
  std::cout << "Filter starting" << std::endl;
}

void FilterBq::loop() {
  const auto view = viewer::get_window3d("Filter Debug");
  ImuMessage imu_msg;

  FiducialDetectionMessage detection_msg;
  if (fiducial_sub_->read(detection_msg, 1)) {
    fiducial_history_.push_back(detection_msg.fiducial_from_camera());
  }

  while (imu_sub_->read(imu_msg, 1)) {
    const jcc::Vec3 accel_mpss(imu_msg.accel_mpss_x, imu_msg.accel_mpss_y, imu_msg.accel_mpss_z);

    const jcc::Vec3 mag_utesla(imu_msg.mag_utesla_x, imu_msg.mag_utesla_y, imu_msg.mag_utesla_z);

    const jcc::Vec3 gyro_radps(imu_msg.gyro_radps_x, imu_msg.gyro_radps_y, imu_msg.gyro_radps_z);

    accel_history_.push_back({accel_mpss, gyro_radps, mag_utesla});
    mag_utesla_.push_back({mag_utesla});
  }

  while (accel_history_.size() > 15u) {
    accel_history_.pop_front();
  }

  while (fiducial_history_.size() > 10u) {
    fiducial_history_.pop_front();
  }

  while (mag_utesla_.size() > 300) {
    mag_utesla_.pop_front();
  }

  for (int k = 0; k < static_cast<int>(fiducial_history_.size()); ++k) {
    const double fraction = static_cast<double>(fiducial_history_.size() - k) / fiducial_history_.size();
    const SE3 world_from_camera = fiducial_history_.at(k);
    geo_->add_axes({world_from_camera, fraction * 1.0, 3.0});
  }

  const auto visitor = [this](const geometry::shapes::EllipseFit& fit) {
    geo_->add_ellipsoid({fit.ellipse, jcc::Vec4(0.4, 0.6, 0.4, 0.7), 2.0});
    geo_->flush();
  };

  const std::vector<jcc::Vec3> viz_utesla(mag_utesla_.begin(), mag_utesla_.end());
  if (mag_utesla_.size() > 250) {
    for (int u = 0; u < static_cast<int>(viz_utesla.size()); ++u) {
      const double fraction = static_cast<double>(viz_utesla.size() - u) / viz_utesla.size();

      const jcc::Vec3 mag_utesla = viz_utesla.at(u);
      // geo_->add_point({mag_utesla, jcc::Vec4(0.8, 0.8, 0.0, 1.0), 4.0});
    }
    // const auto result = geometry::shapes::fit_ellipse(viz_utesla);
    // geo_->add_ellipsoid({result.ellipse, jcc::Vec4(0.2, 1.0, 0.2, 0.7), 4.0});
  }

  for (int j = 0; j < static_cast<int>(accel_history_.size()); ++j) {
    const double fraction = static_cast<double>(accel_history_.size() - j) / accel_history_.size();

    const auto accel_meas = accel_history_.at(j);

    // if (std::abs(accel_meas.accel_mpss.norm() - 9.81) < 0.25) {
    geo_->add_line({jcc::Vec3::Zero(), accel_meas.accel_mpss, jcc::Vec4(1.0, 0.0, 0.0, fraction), 3.0});
    // }
    geo_->add_line({jcc::Vec3::Zero(), accel_meas.gyro_radps, jcc::Vec4(0.0, 1.0, 1.0, 1.0), 3.0});
  }

  geo_->add_sphere({jcc::Vec3::Zero(), 9.81});
  geo_->flip();
}

void FilterBq::shutdown() {
  std::cout << "Filter shutdown" << std::endl;
}

}  // namespace embedded
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::embedded::FilterBq)
