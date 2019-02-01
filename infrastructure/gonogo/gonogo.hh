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
   void nogo(const std::string &status_message);

  private:
   GoNoGoMessage gonogomessage;
   PublisherPtr publisher_;
   std::unique_ptr<CommsFactory> comms_factory_;
   void publish_status();
};

} // namespace jet
