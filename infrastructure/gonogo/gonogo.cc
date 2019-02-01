#include "infrastructure/gonogo/gonogo.hh"

namespace jet {

GoNoGo::GoNoGo() {
    ready_ = false;
    statusMessage_ = "";
    comms_factory_ = std::move(std::make_unique<jet::MqttCommsFactory>());
    publisher_ = comms_factory_->make_publisher("GoNoGo");
}

void GoNoGo::go() {
    ready_ = true;
    publishStatus();
}

void GoNoGo::nogo(std::string statusMessage) {
    ready_ = false;
    statusMessage_ = statusMessage;
    publishStatus();
}

bool GoNoGo::isReady() {
    return ready_;
}

std::string GoNoGo::getStatusMessage() {
    return statusMessage_.value_or("empty");
}

void GoNoGo::publishStatus() {
    GoNoGoMessage message;
    message.ready = ready_;
    message.statusMessage = statusMessage_.value_or("empty");
    publisher_->publish(message);
}

} // namespace jet
