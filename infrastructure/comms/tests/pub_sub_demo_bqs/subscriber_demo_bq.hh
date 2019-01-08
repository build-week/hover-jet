#include <infrastructure/balsa_queue/balsa_queue.hh>

namespace jet {

class SubscriberDemoBq : public BalsaQ {
 public:
  SubscriberDemoBq();
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  SubscriberPtr subscriber_;
};

}  // namespace jet
