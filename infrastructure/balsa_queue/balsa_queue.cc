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
  timestamp_last_report_ = get_current_time();
}

void BalsaQ::base_loop() {
  publish_state();
  ++loop_count_since_last_report_;
}

void BalsaQ::publish_state() {
  static Timestamp last_state_publish_time_{0};
  if (last_state_publish_time_ + Duration::from_seconds(1) < get_current_time() || last_state_publish_time_ == 0) {
    BQStateMessage message;
    message.bq_instance_name = bq_name_;

    if (loop_durations_.size() > 0) {
      auto max_loop_time_it = std::max_element(loop_durations_.begin(), loop_durations_.end());
      if (max_loop_time_it != loop_durations_.end()) {
        message.loop_execution_times.max_time = *max_loop_time_it;
      }

      auto min_loop_time_it = std::min_element(loop_durations_.begin(), loop_durations_.end());
      if (min_loop_time_it != loop_durations_.end()) {
        message.loop_execution_times.min_time = *min_loop_time_it;
      }

      const size_t median_element_index = loop_durations_.size() / 2u;
      std::nth_element(loop_durations_.begin(), loop_durations_.begin() + median_element_index, loop_durations_.end());
      message.loop_execution_times.median_time = loop_durations_[median_element_index];

      loop_durations_.clear();
    }

    if (loop_count_since_last_report_ > 0) {
      message.loop_rate_hz =
          loop_count_since_last_report_ / std::chrono::duration_cast<std::chrono::seconds>(
                                              std::chrono::nanoseconds(get_current_time() - timestamp_last_report_))
                                              .count();
      loop_count_since_last_report_ = 0;
      timestamp_last_report_ = get_current_time();
    }

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

Timestamp BalsaQ::get_current_time() const {
  return Timestamp::current_time();
}

}  // namespace jet
