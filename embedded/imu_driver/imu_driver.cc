#include "embedded/imu_driver/imu_driver.hh"

#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>

#include "embedded/imu_driver/imu_definitions.hh"
#include "embedded/imu_driver/imu_message.hh"

// %deps(rpi_bno055)
#include "third_party/bno055/RPi_Sensor.h"
#include "third_party/bno055/utility/imumaths.h"
#include "third_party/experiments/eigen.hh"

namespace jet {
namespace embedded {

bool ImuDriver::initialize(const std::string& i2c_bus, const uint8_t i2c_address) {
  i2c_bus_ = i2c_bus;

  // Note: The adafruit driver does not take ownership over this string.
  // TODO(Jake): Rewrite the adafruit imu driver, it's bananas.
  constexpr int32_t MEANINGLESS_SENSOR_ID = -1;  // Stupid driver.
  bno_ = std::make_shared<Adafruit_BNO055>(i2c_bus_.c_str(), MEANINGLESS_SENSOR_ID, i2c_address);

  //
  // Wait until we have stable comms with the IMU
  //
  int tries = 0;
  constexpr int MAX_TRIES = 5;
  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // TODO
    std::cout << "Attempting to reconnect to bno (If this happens more than once, power reset the IMU)"
              << std::endl;
    ++tries;
  } while (!bno_->begin(Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_AMG) && (tries < MAX_TRIES));

  if (tries >= MAX_TRIES) {
    return false;
  }

  //
  // Configure gyro range and sample rate
  //
  constexpr uint8_t gyro_cfg = GyroConfig0::DPS_125 | GyroConfig0::HZ_116;
  constexpr int gyro_cfg_addr = ConfigAdresses::GYR_CONFIG_0;
  bno_->configure_page_1(gyro_cfg_addr, gyro_cfg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  //
  // Configure accelerometer range and sample rate
  //
  constexpr uint8_t accel_cfg = AccConfig::g_2 | AccConfig::HZ_125;
  constexpr int accel_cfg_addr = ConfigAdresses::ACC_CONFIG;
  bno_->configure_page_1(accel_cfg_addr, accel_cfg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  //
  // Configure Magnetometer
  //
  constexpr uint8_t high_accuracy = 0b00011000;
  constexpr uint8_t normal_power = 0b00000000;
  constexpr int mag_cfg_addr = 0x6d;
  bno_->configure_page_1(mag_cfg_addr, high_accuracy | normal_power);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // omg sleep is not a synchronization primitive
  // TODO(jake): Figure out if we can get an "i2c ready" flag
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  initialized_ = set_amg_mode(MAX_TRIES);
  std::array<unsigned char, 16> bytes;
  for (uint8_t addr = 0x50; addr <= 0x5F; ++addr) {
    const uint8_t uid = bno_->read_page_1(addr);
    bytes[addr - 0x50] = uid;
  }

  guid_ = xg::Guid(bytes);

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  bno_->setExtCrystalUse(true);

  initialized_ = true;
  return initialized_;
}

int ImuDriver::sample_period_ms() const {
  const double min_freq_hz = std::min(GYRO_FREQUENCY_HZ, ACCEL_FREQUENCY_HZ);
  const double max_period_sec = 1.0 / min_freq_hz;
  constexpr double MILLISECONDS_PER_SECOND = 1e3;
  const double max_period_ms = max_period_sec * MILLISECONDS_PER_SECOND;

  return static_cast<int>(max_period_ms);
}

jcc::Vec3 ImuDriver::read_accel_mpss() {
  assert(initialized_);
  const imu::Vector<3> accel_mpss =
      bno_->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  return jcc::Vec3(accel_mpss.x(), accel_mpss.y(), accel_mpss.z());
}

jcc::Vec3 ImuDriver::read_gyro_radps() {
  assert(initialized_);
  const imu::Vector<3> gyro_radps = bno_->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  return jcc::Vec3(gyro_radps.x(), gyro_radps.y(), gyro_radps.z());
}

jcc::Vec3 ImuDriver::read_magnetometer_utesla() {
  assert(initialized_);
  const imu::Vector<3> mag_utesla = bno_->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  return jcc::Vec3(mag_utesla.x(), mag_utesla.y(), mag_utesla.z());
}

bool ImuDriver::set_amg_mode(int max_num_tries) {
  bno_->setMode(Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_AMG);
  Adafruit_BNO055::adafruit_bno055_opmode_t mode = bno_->getMode();

  int num_tries = 0;
  while(mode != Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_AMG) {
    if (num_tries == max_num_tries) {
      return false;
    }
    bno_->setMode(Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_AMG);
    mode = bno_->getMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    num_tries += 1;
  }
  return true;
}

}  // namespace embedded
}  // namespace jet
