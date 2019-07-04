#include "infrastructure/balsa_queue/balsa_queue.hh"

namespace jet {

class SubscriberDemoBq : public BalsaQ {
 public:
  SubscriberDemoBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  SubscriberPtr subscriber_;
};

}  // namespace jet
