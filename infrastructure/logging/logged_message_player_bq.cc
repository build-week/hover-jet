//%deps(balsa_queue)

#include "logged_message_player_bq.hh"

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <ctime>
#include <iomanip>
#include <iostream>
#include <thread>
#include <unistd.h>

//%deps(pthread)

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

  for (const std::string& channel_name : LogReader(log_path, {}).get_available_channels()) {
    std::thread pub_thread = std::thread([this, channel_name, log_path]() { channel_handler_thread_fn(channel_name, log_path); });
    pub_thread.detach();
    threads_.emplace_back(std::move(pub_thread));
  }
}

void LoggedMessagePlayerBQ::channel_handler_thread_fn(const std::string& channel_name, const std::string& log_path) {
  PublisherPtr publisher = make_publisher(channel_name);
  LogReader log_reader = LogReader(log_path, {channel_name});

  std::string message_content;
  Message message;
  while (!shutdown_) {
    if (log_reader.read_next_message_raw(channel_name, message_content)) {
      message.deserialize(message_content);
      if (current_log_time_ == 0) {
        current_log_time_ = message.header.timestamp_ns;
        publisher->publish_raw(message_content);
        continue;
      }

      uint64_t sleep_time = message.header.timestamp_ns - current_log_time_;
      usleep(sleep_time / 1000);
      publisher->publish_raw(message_content);
      current_log_time_ = message.header.timestamp_ns;
    } else {
      break;
    }
  }
}

void LoggedMessagePlayerBQ::loop() {
}

void LoggedMessagePlayerBQ::shutdown() {
  shutdown_ = true;
  for (auto& thread : threads_) {
    thread.join();
  }
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet
