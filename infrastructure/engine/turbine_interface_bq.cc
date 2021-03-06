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

void TurbineInterfaceBQ::init(const Config& config) {
  // Only send commands to the turbine every 200ms.
  loop_delay_microseconds = 200000;
  turbine_ignition_subscriber_ = make_subscriber("turbine_ignition");
  turbine_throttle_setting_subscriber_ = make_subscriber("turbine_set_throttle");
  turbine_state_publisher_ = make_publisher("turbine_state");
  go_no_go_subscriber_ = make_subscriber("GoNoGo_output");

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

void TurbineInterfaceBQ::set_thrust_percent(uint8_t thrust_percent) {
  std::cout << "Received turbine thrust command! Setting thrust to: "
            << thrust_percent << "%" << std::endl;
  if (!turbine_ptr_->set_thrust_percent(thrust_percent)) {
    std::cerr << "Failed to set thrust." << std::endl;
  } else {
    std::cout << "Successfully set turbine thrust!" << std::endl;
  }
}

void TurbineInterfaceBQ::set_target_rpm(uint32_t target_rpm) {
  std::cout << "Received turbine thrust command! Setting target RPM to: "
            << target_rpm << "RPM" << std::endl;
  if (!turbine_ptr_->set_target_rpm(target_rpm)) {
    std::cerr << "Failed to set target RPM." << std::endl;
  } else {
    std::cout << "Successfully set turbine RPM!" << std::endl;
  }
}

void TurbineInterfaceBQ::loop() {
  // Check to see if we have any new commands we should send to the turbine.
  TurbineIgnitionCommandMessage ignition_command_message;
  if (turbine_ignition_subscriber_->read_latest(ignition_command_message, 0)) {
    if (ignition_command_message.command == IgnitionCommand::START) {
      start_turbine();
    } else if (ignition_command_message.command == IgnitionCommand::STOP) {
      shutdown_turbine();
    }
  }

  go_no_go_subscriber_->read_latest(go_nogo_message_, 0);

  if (go_nogo_message_.ready) {
    ThrottleCommandMessage throttle_command_message;
    if (turbine_throttle_setting_subscriber_->read_latest(throttle_command_message, 0)) {
      set_thrust_percent(throttle_command_message.throttle_percent);
    }
  } else {
    set_thrust_percent(0);
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
    std::cout << "RPM: " << state_message.turbine_rpm << " egt: " << state_message.exhaust_gas_temperature_c << std::endl;
  }
}

void TurbineInterfaceBQ::shutdown() {
  std::cout << "Shutting down!" << std::endl;
  shutdown_turbine();
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::TurbineInterfaceBQ)
