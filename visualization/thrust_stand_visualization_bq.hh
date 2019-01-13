#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include "control/servo_interface.hh"
#include "embedded/servo_bq/set_servo_message.hh"

#include "third_party/experiments/viewer/primitives/simple_geometry.hh"
#include "third_party/experiments/viewer/window_3d.hh"

namespace jet {
namespace visualization {

class ThrustStandVisualizerBq : public BalsaQ {
 public:
  const static uint loop_delay_microseconds = 50000;

  ThrustStandVisualizerBq() = default;
  void init(int argc, char *argv[]) override;
  void loop() override;
  void shutdown() override;

 private:
  SubscriberPtr servo_sub_;
  std::shared_ptr<viewer::SimpleGeometry> geo_;
  std::shared_ptr<viewer::Window3D> view_;
};

}  // namespace visualization
}  // namespace jet
