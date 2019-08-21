#include "filtering/convert_messages.hh"

#include "filtering/to_time_point.hh"

namespace jet {

estimation::TimedMeasurement<ejf::AccelMeasurement> to_accel_meas(const ImuMessage& msg) {
  const auto time_of_validity = to_time_point(msg.timestamp);
  const jcc::Vec3 accel_mpss(msg.accel_mpss_x, msg.accel_mpss_y, msg.accel_mpss_z);
  ejf::AccelMeasurement accel_meas;
  accel_meas.observed_acceleration = accel_mpss;
  return estimation::TimedMeasurement<ejf::AccelMeasurement>{accel_meas, time_of_validity};
}
estimation::TimedMeasurement<ejf::GyroMeasurement> to_gyro_meas(const ImuMessage& msg) {
  const auto time_of_validity = to_time_point(msg.timestamp);
  const jcc::Vec3 gyro_radps(msg.gyro_radps_x, msg.gyro_radps_y, msg.gyro_radps_z);
  ejf::GyroMeasurement gyro_meas;
  gyro_meas.observed_w = gyro_radps;
  return estimation::TimedMeasurement<ejf::GyroMeasurement>{gyro_meas, time_of_validity};
}
estimation::TimedMeasurement<estimation::MagnetometerMeasurement> to_mag_meas(const ImuMessage& msg) {
  const auto time_of_validity = to_time_point(msg.timestamp);
  const jcc::Vec3 mag_utesla(msg.mag_utesla_x, msg.mag_utesla_y, msg.mag_utesla_z);
  estimation::MagnetometerMeasurement mag_meas;
  mag_meas.observed_bfield = mag_utesla;
  return estimation::TimedMeasurement<estimation::MagnetometerMeasurement>{mag_meas, time_of_validity};
}
estimation::TimedMeasurement<ejf::FiducialMeasurement> to_fiducial_meas(const FiducialDetectionMessage& msg) {
  const ejf::FiducialMeasurement fiducial_meas{
      .fiducial_id = 1,                                     //
      .T_fiducial_from_camera = msg.fiducial_from_camera()  //
  };
  const auto time_of_validity = to_time_point(msg.timestamp);
  return {fiducial_meas, time_of_validity};
}

}  // namespace jet