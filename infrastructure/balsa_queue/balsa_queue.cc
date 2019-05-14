#include "balsa_queue.hh"
#include "balsa_queue_state_message.hh"

#include "infrastructure/time/duration.hh"

#include <chrono>

namespace jet {

void BalsaQ::set_comms_factory(std::unique_ptr<CommsFactory> comms_factory) {
  comms_factory_ = std::move(comms_factory);
}

void BalsaQ::base_init() {
  bq_state_publisher_ = make_publisher("bq_state");
}

void BalsaQ::base_loop() {
  publish_state();
}

void BalsaQ::publish_state() {
  static Timestamp last_state_publish_time_{0};
  if (last_state_publish_time_ + Duration::from_seconds(1) < get_current_time() || last_state_publish_time_ == 0) {
    BQStateMessage message;
    message.bq_instance_name = bq_name_;
    bq_state_publisher_->publish(message);
    last_state_publish_time_ = get_current_time();
  }
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
