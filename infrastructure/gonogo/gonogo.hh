#pragma once

#include "infrastructure/comms/comms_factory.hh"
#include "infrastructure/gonogo/gonogo_message.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <string>

namespace jet {

class GoNoGo {
  public: 
   GoNoGo();
   void go();
   void nogo(std::string statusMessage);
   bool isReady();
   std::string getStatusMessage();

  private: 
   bool ready_;
   std::optional<std::string> statusMessage_;
   PublisherPtr publisher_;
   std::unique_ptr<CommsFactory> comms_factory_;
   void publishStatus();
};

} // namespace jet
