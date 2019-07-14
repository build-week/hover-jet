#include <infrastructure/balsa_queue/balsa_queue.hh>

#include "infrastructure/comms/shared_struct/shared_struct_subscriber.hh"
#include "infrastructure/comms/tests/comms_latency_test/latency_probe_message.hh"

namespace jet {

class RingBuffer {
public:
  void insert_into_ring_buffer(uint64_t latency) {
    if (latency_ring_buffer_.size() < ring_buffer_max_size_) {
      latency_ring_buffer_.emplace_back(latency);
    }
    else {
      latency_ring_buffer_.pop_front();
      latency_ring_buffer_.emplace_back(latency);
    }
  }

  float get_avg_from_ring_buffer() {
    uint64_t total_latency_in_buffer = 0;
    for (auto& value : latency_ring_buffer_) {
      total_latency_in_buffer += value;
    }
    return total_latency_in_buffer / latency_ring_buffer_.size() / 1000000.0;
  }
 private:
  std::deque<uint64_t> latency_ring_buffer_;
  size_t ring_buffer_max_size_ = 10;
};

class CommsLatencyTestSubscriberBq : public BalsaQ {
 public:
  CommsLatencyTestSubscriberBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  SubscriberPtr subscriber_;
  uint64_t total_time_between_messages_ns_ = 0;
  uint32_t total_message_count_ = 0;
  RingBuffer ring_buffer_;
  uint64_t recent_sequence_number_ = 0;

  Timestamp previous_loop_time_ = 0;

  std::unique_ptr<SharedStructSubscriber> shared_struct_;

};

}  // namespace jet
