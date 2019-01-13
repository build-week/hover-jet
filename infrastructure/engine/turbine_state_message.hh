#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/comms/serialization/serialization_macros.hh"
#include "infrastructure/engine/jetcat_schemas.hh"

namespace jet {

struct TurbineStateMessage : JetCat::LiveValues, Message {
  MESSAGE(TurbineStateMessage,
          turbine_rpm,
          exhaust_gas_temperature_c,
          pump_voltage,
          turbine_state,
          throttle_position_percent);
};

}  // namespace jet
