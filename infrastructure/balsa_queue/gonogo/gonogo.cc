

#include "infrastructure/balsa_queue/gonogo/gonogo.hh"

namespace jet {

GoNoGo::GoNoGo() {
    ready = false;
    message = "";
}

void GoNoGo::go(std::string message) {
    ready = true;
    message = message;
}

void GoNoGo::nogo(std::string message) {
    ready = false;
    message = message;
}

bool GoNoGo::isReady() {
    return ready;
}

std::string GoNoGo::getMessage() {
    return message;
}

} // namespace jet