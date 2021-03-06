#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/logging/log_writer.hh"

namespace jet {

class MessageLoggerBQ : public BalsaQ {
 public:
  MessageLoggerBQ() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  std::unique_ptr<LogWriter> log_writer_ptr_;
  std::vector<std::pair<std::string, SubscriberPtr>> subscribers_;
  std::vector<std::string> channels_;
};

}  // namespace jet
