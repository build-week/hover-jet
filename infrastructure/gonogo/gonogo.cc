#include "infrastructure/gonogo/gonogo.hh"

#include <string>

namespace jet {

GoNoGo::GoNoGo() {
    gonogomessage.bq_name = "";
    gonogomessage.ready_ = false;
    gonogomessage.status_message_ = "";
    comms_factory_ = std::move(std::make_unique<jet::MqttCommsFactory>());
    publisher_ = comms_factory_->make_publisher("GoNoGo");
}

void GoNoGo::setName(std::string name) {
    gonogomessage.bq_name = name;
}

void GoNoGo::go() {
    gonogomessage.ready_ = true;
    // for now this is an empty message because the message type won't compile with an optional string
    gonogomessage.status_message_ = "";
    publish_status();
}

void GoNoGo::nogo(const std::string &status_message) {
    gonogomessage.ready_ = false;
    gonogomessage.status_message_ = status_message;
    publish_status();
}

void GoNoGo::publish_status() {
    publisher_->publish(gonogomessage);
}

} // namespace jet
