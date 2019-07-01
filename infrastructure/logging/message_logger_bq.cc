//%bin(message_logger_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "infrastructure/logging/message_logger_bq.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include <ctime>
#include <iomanip>
#include <iostream>

namespace jet {

void MessageLoggerBQ::init(const Config& config) {
  // Require presence of the fields we're using
  assert(config["logged_channels"]);
  assert(config["log_base_path"]);

  // Load log channels
  channels_ = config["logged_channels"].as<std::vector<std::string>>();
  const std::string log_base_path = config["log_base_path"].as<std::string>();


  // Generate a log name. Log name will be the UTC date and time in the format YYYYMMDDHHMMSS
  time_t now = time(0);
  tm* gmtm = gmtime(&now);
  std::ostringstream str_time;
  str_time << std::put_time(gmtm, "%Y%m%d%H%M%S");
  const std::string log_path = log_base_path + str_time.str();
  std::cout << "Creating log with name: " << log_path << std::endl;

  log_writer_ptr_ = std::make_unique<LogWriter>(log_path, channels_);

  for (const std::string& channel_name : channels_) {
    subscribers_.emplace_back(channel_name, make_subscriber(channel_name));
  }
  gonogo().go();
}

void MessageLoggerBQ::loop() {
  std::string message_content;
  for (auto& subscriber : subscribers_) {
    if (subscriber.second->read_raw(message_content, 0)) {
      log_writer_ptr_->write_message(subscriber.first, message_content);
    }
  }
}

void MessageLoggerBQ::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::MessageLoggerBQ)
