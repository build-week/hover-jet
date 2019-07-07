#pragma once

#include "embedded/imu_driver/imu_message.hh"
#include "vision/fiducial_detection_message.hh"

#include "third_party/experiments/estimation/calibration/calibration_dataset.hh"

namespace jet {
namespace ejf = estimation::jet_filter;
estimation::TimedMeasurement<ejf::AccelMeasurement> to_accel_meas(const ImuMessage& msg);
estimation::TimedMeasurement<ejf::GyroMeasurement> to_gyro_meas(const ImuMessage& msg);
estimation::TimedMeasurement<estimation::MagnetometerMeasurement> to_mag_meas(const ImuMessage& msg);
estimation::TimedMeasurement<ejf::FiducialMeasurement> to_fiducial_meas(const FiducialDetectionMessage& msg);
}  // namespace jet