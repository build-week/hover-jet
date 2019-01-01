#include <infrastructure/balsa_queue/balsa_queue.hh>

namespace jet {

class PublisherDemoBq : public BalsaQ {
 public:
  PublisherDemoBq();
  void init();
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
};

}  // namespace jet
