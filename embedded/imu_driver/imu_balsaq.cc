//%bin(imu_balsaq_main)
#include "embedded/imu_driver/imu_balsaq.hh"
#include "embedded/imu_driver/imu_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include <cstddef>
#include <iostream>

namespace jet {
namespace embedded {

void ImuBq::init(const Config& config) {
  const std::string bus = config["bus"].as<std::string>();

  imu_drivers_.reserve(config["imus"].size());

  bool all_go = true;
  for (const auto& sub_config : config["imus"]) {
    const std::string channel = sub_config["channel"].as<std::string>();
    std::cout << "IMU starting (" << sub_config["location_description"].as<std::string>() << ")" << std::endl;
    // Must be read as an int, then narrowing conversion.
    const uint8_t i2c_address = static_cast<uint8_t>(sub_config["i2c_address"].as<int>());
    const int unique_identifier = sub_config["serial_number"].as<int>();

    std::cout << "Starting IMU '" << unique_identifier << "' on:" << bus << ":" << static_cast<int>(i2c_address)
              << std::endl;

    // Note: We do not want to copy the driver *after* it has been initialized
    imu_drivers_.push_back({.driver = {},
                            .publisher = make_publisher(channel),
                            .imu_unique_identifier = unique_identifier,
                            .i2c_address = i2c_address,
                            .i2c_bus = bus});

    const bool init_success = imu_drivers_.back().reset();
    if (!init_success) {
      gonogo().nogo("Failed to intialize IMU driver");
      all_go = false;
    }
  }

  if (all_go) {
    gonogo().go("Initialized all IMU's");
  }
}

void ImuBq::loop() {
  bool all_go = true;
  for (auto& imu : imu_drivers_) {
    ImuMessage msg;
    msg.timestamp = get_current_time();

    const jcc::Vec3 accel_mpss = imu.driver.read_accel_mpss();
    const jcc::Vec3 angvel_radps = imu.driver.read_gyro_radps();
    const jcc::Vec3 mag_utesla = imu.driver.read_magnetometer_utesla();

    msg.accel_mpss_x = accel_mpss.x();
    msg.accel_mpss_y = accel_mpss.y();
    msg.accel_mpss_z = accel_mpss.z();

    msg.gyro_radps_x = angvel_radps.x();
    msg.gyro_radps_y = angvel_radps.y();
    msg.gyro_radps_z = angvel_radps.z();

    msg.mag_utesla_x = mag_utesla.x();
    msg.mag_utesla_y = mag_utesla.y();
    msg.mag_utesla_z = mag_utesla.z();

    msg.imu_id = imu.imu_unique_identifier;

    // It's expected that we'll see this failure during startup.
    // TODO(jpanikulam): Resolve why.
    // It's *not* expected that this happens and we try to recover an IMU during operation
    if (msg.accel_mpss_x == 0 && msg.accel_mpss_y == 0 && msg.accel_mpss_z == 0) {
      gonogo().nogo("IMU reading all zeroes (We will attempt a reset)!");
      all_go = false;
      imu.reset();
    } else {
      imu.publisher->publish(msg);
    }
  }

  if (all_go) {
    gonogo().go("Recovered IMU");
  }
}

void ImuBq::shutdown() {
  std::cout << "IMU shutdown" << std::endl;
}

}  // namespace embedded
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::embedded::ImuBq)
