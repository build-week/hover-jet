#include "force_sensors/force_sensor_message.hh"
#include "infrastructure/logging/log_reader.hh"

namespace jet {

void go() {
  const std::string force_sensor_output_channel = "force_sensor_output_channel";
  const std::string servo_command_channel = "servo_command_channel";
  const std::string turbine_ignition = "turbine_ignition";
  const std::string turbine_set_throttle = "turbine_set_throttle";
  const std::string turbine_state = "turbine_state";
  const std::string imu = "imu";

  const std::vector<std::string> channels = {{force_sensor_output_channel,
                                              servo_command_channel, turbine_ignition,
                                              turbine_set_throttle, turbine_state, imu}};

  const std::string log_path = "/jet/jet-test-1-15--2";
  LogReader reader(log_path, channels);

  for (int k = 0; k < 10000; ++k) {
    ForceSensorMessage force_msg;
    if (reader.read_next_message(force_sensor_output_channel, force_msg)) {
      std::cout << force_msg.id << ": " << force_msg.value << std::endl;
    }
  }
}

}  // namespace jet

int main() {
  jet::go();
}
