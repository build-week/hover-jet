#pragma once

#include "infrastructure/comms/schemas/message.hh"

#include <string>

namespace jet {

//
struct PowerReading : Message {
  float bus_voltage_mV;
  float current_mA;
  float power_mW;

  MESSAGE(PowerReading, bus_voltage_mV, current_mA, power_mW);
};

} //  namespace jet
