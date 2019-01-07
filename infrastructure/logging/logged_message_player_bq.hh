#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/logging/log_reader.hh"

#include <atomic>

namespace jet {

class LoggedMessagePlayerBQ : public BalsaQ {
 public:
  LoggedMessagePlayerBQ();
  void init();
  void loop();
  void shutdown();

 private:
  void channel_handler_thread_fn(const std::string& channel_name, const std::string& log_path);
  bool shutdown_{false};
  std::vector<std::thread> threads_;
  std::atomic<uint64_t> current_log_time_{0};
};

}  // namespace jet
