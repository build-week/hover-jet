//%bin(measurement_sim_bq_main)
#include "filtering/measurement_sim_bq.hh"

#include "filtering/to_time_point.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "third_party/experiments/estimation/time_point.hh"

#include "filtering/imu_model_from_yaml.hh"

#include <cstddef>
#include <iostream>

namespace jet {
namespace filtering {

constexpr int IMU_1_ID = 78;
constexpr int IMU_2_ID = 36;

MeasurementSimBq::MeasurementSimBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void MeasurementSimBq::init(const Config& config) {
  gonogo().nogo("Starting up");

  transform_network_ = transform_network_from_yaml(config["transforms"]);

  const auto cfg_imu_1 = imu_model_from_yaml(config["imus"][IMU_1_ID]);
  const auto cfg_imu_2 = imu_model_from_yaml(config["imus"][IMU_2_ID]);

  imu_model_from_id_[cfg_imu_1.imu_id] = cfg_imu_1.imu_model;
  imu_model_from_id_[cfg_imu_2.imu_id] = cfg_imu_2.imu_model;

  imu_1_pub_ = make_publisher("imu_1");
  imu_2_pub_ = make_publisher("imu_2");
  fiducial_pub_ = make_publisher("fiducial_detection_channel");
}

ImuMessage imu_msg_from_acceleration(const jcc::Vec3& acceleration_mpss, const Timestamp& now_ts, const int imu_id) {
  ImuMessage imu_msg;
  {
    imu_msg.accel_mpss_x = acceleration_mpss.x();
    imu_msg.accel_mpss_y = acceleration_mpss.y();
    imu_msg.accel_mpss_z = acceleration_mpss.z();
    imu_msg.gyro_radps_x = 0.0;
    imu_msg.gyro_radps_y = 0.0;
    imu_msg.gyro_radps_z = 0.0;
    imu_msg.mag_utesla_x = 0.0;
    imu_msg.mag_utesla_y = 0.0;
    imu_msg.mag_utesla_z = 0.0;
    imu_msg.timestamp = now_ts;
    imu_msg.imu_id_leading_byte = imu_id;
  }
  return imu_msg;
}

void MeasurementSimBq::loop() {
  const auto now_ts = get_current_time();
  const auto now_tp = to_time_point(now_ts);

  const jcc::Vec3 g_camera = jcc::Vec3::UnitY() * 9.81;

  {
    const SO3 camera_from_imu = transform_network_.find_source_from_destination("camera", "imu_78").so3();

    const auto& imu_model = imu_model_from_id_[IMU_1_ID];
    const jcc::Vec3 true_accel_imu = camera_from_imu.inverse() * g_camera;
    const jcc::Vec3 measured_accel_imu = imu_model.distort_true_accel(true_accel_imu);
    auto imu_1_msg = imu_msg_from_acceleration(measured_accel_imu, now_ts, IMU_1_ID);
    imu_1_pub_->publish(imu_1_msg);
  }
  {
    const SO3 camera_from_imu = transform_network_.find_source_from_destination("camera", "imu_36").so3();
    const auto& imu_model = imu_model_from_id_[IMU_2_ID];
    const jcc::Vec3 true_accel_imu = camera_from_imu.inverse() * g_camera;
    const jcc::Vec3 measured_accel_imu = imu_model.distort_true_accel(true_accel_imu);
    auto imu_2_msg = imu_msg_from_acceleration(measured_accel_imu, now_ts, IMU_2_ID);
    imu_2_pub_->publish(imu_2_msg);
  }

  if (estimation::to_seconds(now_tp - to_time_point(last_publish_)) > fiducial_latency_s_) {
    last_publish_ = now_ts;
    FiducialDetectionMessage fiducial_msg;

    const jcc::Vec3 translation_jet_from_fiducial(1.0, 1.0, 2.0);
    const SO3 R_jet_from_fiducial = SO3::exp(jcc::Vec3(-M_PI, 1.0, -1.0));
    const SE3 jet_from_fiducial(R_jet_from_fiducial, translation_jet_from_fiducial);
    const jcc::Vec6 log_fiducial_from_camera = jet_from_fiducial.inverse().log();

    fiducial_msg.fiducial_from_camera_log =
        std::array<double, 6>{{log_fiducial_from_camera[0], log_fiducial_from_camera[1], log_fiducial_from_camera[2],
                               log_fiducial_from_camera[3], log_fiducial_from_camera[4], log_fiducial_from_camera[5]}};
    fiducial_msg.timestamp = Timestamp(now_tp - estimation::to_duration(fiducial_latency_s_));

    fiducial_pub_->publish(fiducial_msg);
  }
}

void MeasurementSimBq::shutdown() {
  std::cout << "Filter shutdown" << std::endl;
}

}  // namespace filtering
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::filtering::MeasurementSimBq)
