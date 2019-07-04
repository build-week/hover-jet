#include "infrastructure/balsa_queue/balsa_queue.hh"

namespace jet {

class TurbineMonitorUtilityBQ : public BalsaQ {
 public:
  TurbineMonitorUtilityBQ() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  SubscriberPtr turbine_state_subscriber_;
};

}  // namespace jet
