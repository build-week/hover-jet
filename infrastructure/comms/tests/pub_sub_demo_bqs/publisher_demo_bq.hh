#include <infrastructure/balsa_queue/balsa_queue.hh>

namespace jet {

class PublisherDemoBq : public BalsaQ {
 public:
  PublisherDemoBq() = default;
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
};

}  // namespace jet
