//%bin(turbine_interface_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/engine/turbine_interface_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "infrastructure/engine/throttle_command_message.hh"
#include "infrastructure/engine/turbine_ignition_message.hh"
#include "infrastructure/engine/turbine_state_message.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <unistd.h>
#include <iostream>

namespace jet {

namespace {
constexpr char* SERIAL_PORT_PATH = "/dev/ttyUSB0";
}  // namespace

TurbineInterfaceBQ::TurbineInterfaceBQ() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void TurbineInterfaceBQ::init(int argc, char* argv[]) {
  // Only send commands to the turbine every 200ms.
  loop_delay_microseconds = 200000;
  turbine_ignition_subscriber_ = make_subscriber("turbine_ignition");
  turbine_throttle_setting_subscriber_ = make_subscriber("turbine_set_throttle");
  turbine_state_publisher_ = make_publisher("turbine_state");

  turbine_ptr_ = std::make_unique<JetCatTurbine>(SERIAL_PORT_PATH);
}

void TurbineInterfaceBQ::start_turbine() {
  std::cout << "Received turbine start command!" << std::endl;
  bool success = turbine_ptr_->start_engine();
  if (!success) {
    std::cerr << "Failed to start engine!" << std::endl;
  } else {
    std::cout << "Successful engine start." << std::endl;
  }
}

void TurbineInterfaceBQ::shutdown_turbine() {
  std::cout << "Received turbine stop command!" << std::endl;
  bool success = turbine_ptr_->stop_engine();
  if (!success) {
    std::cerr << "Failed to stop engine!" << std::endl;
  } else {
    std::cout << "Successful engine stop." << std::endl;
  }
}

void TurbineInterfaceBQ::set_turbine_throttle(uint32_t throttle_percent) {
  std::cout << "Received turbine throttle command! Setting throttle to: "
            << throttle_percent << std::endl;
  turbine_ptr_->set_rpm(throttle_percent);
}

void TurbineInterfaceBQ::loop() {
  // Check to see if we have any new commands we should send to the turbine.
  TurbineIgnitionCommandMessage ignition_command_message;
  bool ignition_command_found = false;
  while (turbine_ignition_subscriber_->read(ignition_command_message, 0)) {
    ignition_command_found = true;
  }
  if (ignition_command_found) {
    if (ignition_command_message.command == IgnitionCommand::START) {
      start_turbine();
    } else if (ignition_command_message.command == IgnitionCommand::STOP) {
      shutdown_turbine();
    }
  }

  ThrottleCommandMessage throttle_command_message;
  bool throttle_command_found = false;
  while (turbine_throttle_setting_subscriber_->read(throttle_command_message, 0)) {
    throttle_command_found = true;
  }
  if (throttle_command_found) {
    set_turbine_throttle(throttle_command_message.throttle_percent);
  }

  std::optional<JetCat::LiveValues> live_values = turbine_ptr_->get_live_values();
  if (!live_values) {
    std::cerr << "Could not read live values from turbine." << std::endl;
  } else {
    TurbineStateMessage state_message;
    state_message.turbine_rpm = live_values->turbine_rpm;
    state_message.exhaust_gas_temperature_c = live_values->exhaust_gas_temperature_c;
    state_message.pump_voltage = live_values->pump_voltage;
    state_message.turbine_state = live_values->turbine_state;
    state_message.throttle_position_percent = live_values->throttle_position_percent;
    turbine_state_publisher_->publish(state_message);
  }
}

void TurbineInterfaceBQ::shutdown() {
  std::cout << "Shutting down!" << std::endl;
  shutdown_turbine();
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::TurbineInterfaceBQ)
