
#include "infrastructure/joystick/joystick.hh"

int main(int argc, char *argv[]) {
  jet::Joystick joystick("/dev/input/js2");
  // jet::Joystick joystick("/dev/input/by-id/usb-Thustmaster_Joystick_-_HOTAS_Warthog-joystick");
  while (true) {
    auto event = joystick.read_event();
    if (!event) {
      continue;
    }
    switch (event->event_type) {
      case jet::EventType::BUTTON: {
        std::cerr << "Button " << unsigned(event->axis_id) << ": "
                  << (static_cast<uint8_t>(*event->button_state) ? "pressed" : "released") << std::endl;
        break;
      }
      case jet::EventType::AXIS: {
        std::cerr << "Axis " << unsigned(event->axis_id) << " "
                  << " value: " << *event->axis_value << std::endl;
        break;
      }
      default:
        break;
    }
  }
}
