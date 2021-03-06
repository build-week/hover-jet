#pragma once

#include <infrastructure/balsa_queue/balsa_queue.hh>

#include "infrastructure/engine/jetcat_turbine.hh"
#include "infrastructure/gonogo/gonogo_message.hh"

namespace jet {

class TurbineInterfaceBQ : public BalsaQ {
 public:
  TurbineInterfaceBQ();
  void init(const Config& config);
  void loop();
  void shutdown();

  void start_turbine();
  void shutdown_turbine();
  void set_thrust_percent(uint8_t thrust_percent);
  void set_target_rpm(uint32_t target_rpm);

 private:
  std::unique_ptr<JetCatTurbine> turbine_ptr_;
  SubscriberPtr turbine_ignition_subscriber_;
  SubscriberPtr turbine_throttle_setting_subscriber_;
  SubscriberPtr go_no_go_subscriber_;
  PublisherPtr turbine_state_publisher_;

  GoNoGoMessage go_nogo_message_;

  Timestamp last_turbine_read_time_{0};
};

}  // namespace jet
