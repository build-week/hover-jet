#pragma once

#include "infrastructure/comms/comms_factory.hh"
#include "infrastructure/gonogo/gonogo_message.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <string>
#include <iostream>

namespace jet {

class GoNoGo {
  public:
   GoNoGo();
   void setName(std::string name);
   void go(const std::string &status_message = "");
   void nogo(const std::string &status_message);

  private:
   GoNoGoMessage gonogomessage_;
   PublisherPtr publisher_;
   std::unique_ptr<CommsFactory> comms_factory_;
   void publish_status();
};

} // namespace jet
