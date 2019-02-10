#include "embedded/imu_driver/imu_message.hh"
#include "force_sensors/force_sensor_message.hh"
#include "infrastructure/engine/turbine_state_message.hh"

#include "infrastructure/logging/log_reader.hh"

namespace jet {

void go() {
  const std::string force_sensor_output_channel = "force_sensor_output_channel";
  const std::string servo_command_channel = "servo_command_channel";
  const std::string turbine_ignition = "turbine_ignition";
  const std::string turbine_set_throttle = "turbine_set_throttle";
  const std::string turbine_state = "turbine_state";
  const std::string imu = "imu";

  const std::vector<std::string> channels = {
      {force_sensor_output_channel, servo_command_channel, turbine_ignition, turbine_set_throttle, turbine_state, imu}};

  // const std::string log_path = "/jet/jet-test-1-15--2";
  const std::string log_path = "/jet/test-logs/20190126231856";
  LogReader reader(log_path, channels);

  uint64_t first_time = 0;
  bool got_first_time = false;
  for (;;) {
    ForceSensorMessage force_msg;
    if (reader.read_next_message(force_sensor_output_channel, force_msg)) {
      if (!got_first_time) {
        first_time = force_msg.timestamp;
        got_first_time = true;
      }
      const uint64_t t = uint64_t(force_msg.timestamp);
      std::cout << "force " << force_msg.id << ": " << (t - first_time) << "," << force_msg.value << std::endl;
    } else {
      break;
    }
  }

  for (;;) {
    ImuMessage imu_msg;
    if (reader.read_next_message(imu, imu_msg)) {
      const uint64_t t = uint64_t(imu_msg.timestamp);

      std::cout << "mag_utesla_x: " << (t - first_time) << "," << imu_msg.mag_utesla_x << std::endl;
      std::cout << "mag_utesla_y: " << (t - first_time) << "," << imu_msg.mag_utesla_y << std::endl;
      std::cout << "mag_utesla_z: " << (t - first_time) << "," << imu_msg.mag_utesla_z << std::endl;
    } else {
      break;
    }
  }

  for (;;) {
    TurbineStateMessage turb_msg;
    if (reader.read_next_message(turbine_state, turb_msg)) {
      const uint64_t t = turb_msg.header.timestamp_ns;
      std::cout << "turbine_rpm: " << (t - first_time) << "," << turb_msg.turbine_rpm << std::endl;
      std::cout << "pump_voltage: " << (t - first_time) << "," << turb_msg.pump_voltage << std::endl;
    } else {
      break;
    }
  }
}

}  // namespace jet

int main() {
  jet::go();
}
