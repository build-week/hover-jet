#include <infrastructure/balsa_queue/balsa_queue.hh>

#include "infrastructure/comms/shared_struct/shared_struct_publisher.hh"
#include "infrastructure/comms/tests/comms_latency_test/latency_probe_message.hh"

namespace jet {

class CommsLatencyTestPublisherBq : public BalsaQ {
 public:
  CommsLatencyTestPublisherBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
  std::string payload_;

  std::unique_ptr<jet::SharedStructPublisher> shared_struct_;
};

}  // namespace jet
