#include "balsa_queue.hh"

#include <chrono>

namespace jet {

void BalsaQ::set_comms_factory(std::unique_ptr<CommsFactory> comms_factory) {
  comms_factory_ = std::move(comms_factory);
}

std::unique_ptr<Publisher> BalsaQ::make_publisher(const std::string& channel_name) {
  return comms_factory_->make_publisher(channel_name);
}
std::unique_ptr<Subscriber> BalsaQ::make_subscriber(const std::string& channel_name) {
  return comms_factory_->make_subscriber(channel_name);
}

Timestamp BalsaQ::get_current_time() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
}

}  // namespace jet
