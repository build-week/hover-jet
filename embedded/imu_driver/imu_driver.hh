#pragma once

#include <memory>

#include "third_party/bno055/RPi_BNO055.h"
#include "third_party/experiments/eigen.hh"

namespace jet {
namespace embedded {

// Common Use Case:
// ```
//  ImuDriver driver;
//  driver.initialize();
//  while(true) {
//    Eigen::Vector3d measured_acceleration_mpss = driver.read_accel_mpss()
//    do_work(measured_acceleration_mpss);
//  }
// ```
class ImuDriver {
 public:
  ImuDriver() = default;

  // This function
  // WARNING: Must be called before sampling from the IMU
  void initialize();

  // Sampling more frequently than this will provide no new measurements
  int sample_period_ms() const;

  // Read the accelerometer register on the IMU
  // NOTE: It is suggested that one does not call this function
  //       more frequently than the IMU sample rate
  // NOTE: This function is non-const because the BNO055 API
  //       is not const-correct
  //
  // @returns: Estimated acceleration in the IMU frame
  jcc::Vec3 read_accel_mpss();

  // Read the gyro register on the IMU
  // NOTE: It is suggested that one does not call this function
  //       more frequently than the IMU sample rate
  // NOTE: This function is non-const because the BNO055 API
  //       is not const-correct
  //
  // @returns: Estimated angular velocity in the IMU frame
  jcc::Vec3 read_gyro_radps();

 private:
  bool initialized_ = false;
  std::shared_ptr<Adafruit_BNO055> bno_;

  constexpr static double GYRO_FREQUENCY_HZ = 116.0;
  constexpr static double ACCEL_FREQUENCY_HZ = 125.0;
};

}  // namespace embedded
}  // namespace jet