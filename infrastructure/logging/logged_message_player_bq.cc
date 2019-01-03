//%deps(balsa_queue)

#include "logged_message_player_bq.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <ctime>
#include <iomanip>
#include <iostream>

namespace jet {

LoggedMessagePlayerBQ::LoggedMessagePlayerBQ() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void LoggedMessagePlayerBQ::init() {
  std::string log_path;
  if (const char* env_log_path = std::getenv("LOG_PATH")) {
      log_path = env_log_path;
  } else {
    throw std::runtime_error("Environment variable LOG_PATH must be set.");
  }

  log_reader_ptr_ = std::make_unique<LogReader>(log_path, channels_);

  for (const std::string& channel_name : log_reader_ptr_->get_available_channels()) {
    publishers_.emplace_back(channel_name, make_publisher(channel_name));
  }
}

void LoggedMessagePlayerBQ::loop() {
  std::string message_content;
  for (auto& publisher_pair : publishers_) {
    if (log_reader_ptr_->read_next_message_raw(publisher_pair.first, message_content)) {
      publisher_pair.second->publish_raw(message_content);
    }
  }
}

void LoggedMessagePlayerBQ::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet
