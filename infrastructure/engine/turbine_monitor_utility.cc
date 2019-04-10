//%bin(turbine_monitor_bq)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/engine/turbine_monitor_utility.hh"

#include "infrastructure/engine/turbine_state_message.hh"

#include <iostream>

namespace jet {

void TurbineMonitorUtilityBQ::init(const Config& config) {
  turbine_state_subscriber_ = make_subscriber("turbine_state");
}

void TurbineMonitorUtilityBQ::loop() {
  TurbineStateMessage message;
  if (turbine_state_subscriber_->read(message, 1)) {
    std::cout << "Turbine State: " << enum_to_string(message.turbine_state) << " RPM: " << message.turbine_rpm
              << " EGT: " << message.exhaust_gas_temperature_c << " Pump Voltage: " << message.pump_voltage
              << " throttle position: " << message.throttle_position_percent << "%" << std::endl;
  }
}

void TurbineMonitorUtilityBQ::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::TurbineMonitorUtilityBQ)
