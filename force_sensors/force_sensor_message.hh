#pragma once

#include "infrastructure/time/timestamp.hh"
#include "infrastructure/comms/schemas/message.hh"

namespace jet {

struct ForceSensorMessage : Message {
	//TODO Ralph rename values to something better?
  std::vector<float> values;

  MESSAGE(ForceSensorMessage, values);
};


}  //  namespace jet
