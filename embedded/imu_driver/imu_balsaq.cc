//%bin(imu_balsaq_main)
#include "embedded/imu_driver/imu_balsaq.hh"
#include "embedded/imu_driver/imu_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include <cstddef>
#include <iostream>
#include <iomanip>

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

    const xg::Guid expected_imu_guid(sub_config["unique_id"].as<std::string>());
    const int unique_identifier = expected_imu_guid.bytes()[0];

    std::cout << "Starting IMU '" << unique_identifier << "' on:" << bus << ":" << std::hex
              << static_cast<int>(i2c_address) << std::endl;

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
      break;
    }

    const xg::Guid actual_guid = imu_drivers_.back().driver.imu_guid();
    const bool got_correct_guid = (actual_guid == expected_imu_guid);
    if (!got_correct_guid) {
      std::cout << "Recieved GUID: " << actual_guid << std::endl;;
      throw std::runtime_error("GUID mismatch with IMU");
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

    jcc::Vec3 accel_mpss = {};
    jcc::Vec3 angvel_radps = {};
    jcc::Vec3 mag_utesla = {};

    if (!imu.driver.read_agm(accel_mpss, angvel_radps, mag_utesla)) {
      throw std::runtime_error("Error reading IMU I2C");
    }

    msg.accel_mpss_x = accel_mpss.x();
    msg.accel_mpss_y = accel_mpss.y();
    msg.accel_mpss_z = accel_mpss.z();

    msg.gyro_radps_x = angvel_radps.x();
    msg.gyro_radps_y = angvel_radps.y();
    msg.gyro_radps_z = angvel_radps.z();

    msg.mag_utesla_x = mag_utesla.x();
    msg.mag_utesla_y = mag_utesla.y();
    msg.mag_utesla_z = mag_utesla.z();

    msg.imu_id_leading_byte = imu.imu_unique_identifier;

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
