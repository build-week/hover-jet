#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/logging/log_reader.hh"

namespace jet {

class LoggedMessagePlayerBQ : public BalsaQ {
 public:
  LoggedMessagePlayerBQ();
  void init();
  void loop();
  void shutdown();

 private:
  std::unique_ptr<LogReader> log_reader_ptr_;
  std::vector<std::pair<std::string, PublisherPtr>> publishers_;
  std::vector<std::string> channels_;
};

}  // namespace jet
