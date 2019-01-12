#pragma once

#include <infrastructure/balsa_queue/balsa_queue.hh>

#include "infrastructure/engine/jetcat_turbine.hh"

namespace jet {

class TurbineInterfaceBQ : public BalsaQ {
 public:
  TurbineInterfaceBQ();
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  std::unique_ptr<JetCatTurbine> turbine_ptr_;
};

}  // namespace jet
