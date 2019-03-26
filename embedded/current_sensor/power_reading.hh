#pragma once

#include "infrastructure/comms/schemas/message.hh"

#include <string>

namespace jet {

// A message for communicating readings from a current sensor.
struct PowerReading : Message {
  float bus_voltage_V;
  float current_mA;
  float power_mW;

  MESSAGE(PowerReading, bus_voltage_V, current_mA, power_mW);
};

}  //  namespace jet
