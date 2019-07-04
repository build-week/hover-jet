#pragma once

#include <memory>

#include "third_party/bno055/RPi_BNO055.h"
#include "third_party/experiments/eigen.hh"
#include "third_party/experiments/util/optional.hh"
#include "embedded/imu_driver/imu_definitions.hh"

// %deps(crossguid)
#include <crossguid/guid.hpp>

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

  // This function initializes the imu interface
  // WARNING: Must be called before sampling from the IMU
  bool initialize(const std::string& i2c_bus, const uint8_t i2c_address);

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

  // Read the magnetometer register on the IMU
  // NOTE: It is suggested that one does not call this function
  //       more frequently than the IMU sample rate
  // NOTE: This function is non-const because the BNO055 API
  //       is not const-correct
  //
  // @returns: Estimated magnetic field strength in the IMU frame
  jcc::Vec3 read_magnetometer_utesla();

  // Read all three accel, gyro, and mag registers on the IMU.
  // NOTE: It is suggested that one does not call this function
  //       more frequently than the IMU sample rate
  // NOTE: This function is non-const because the BNO055 API
  //       is not const-correct
  const jcc::Optional<ImuMeasurements> read_accel_mag_gyro();

  const xg::Guid& imu_guid() const {
    return guid_;
  }

 private:

  bool set_amg_mode(int max_num_tries);
  bool initialized_ = false;
  std::string i2c_bus_;
  std::shared_ptr<Adafruit_BNO055> bno_;

  xg::Guid guid_;

  constexpr static double GYRO_FREQUENCY_HZ = 116.0;
  constexpr static double ACCEL_FREQUENCY_HZ = 125.0;
};

}  // namespace embedded
}  // namespace jet
