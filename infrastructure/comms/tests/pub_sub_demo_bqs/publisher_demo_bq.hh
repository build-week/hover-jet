#include <infrastructure/balsa_queue/balsa_queue.hh>

namespace jet {

class PublisherDemoBq : public BalsaQ {
 public:
  PublisherDemoBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
};

}  // namespace jet
