#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include "control/servo_interface.hh"
#include "embedded/servo_bq/set_servo_message.hh"

#include "third_party/experiments/viewer/primitives/simple_geometry.hh"
#include "third_party/experiments/viewer/window_3d.hh"
#include "visualization/thrust_stand_visualizer.hh"

namespace jet {
namespace visualization {

class ThrustStandVisualizerBq : public BalsaQ {
 public:
  const static uint loop_delay_microseconds = 1000;

  ThrustStandVisualizerBq() = default;
  void init(const Config& config) override;
  void loop() override;
  void shutdown() override;

 private:
  SubscriberPtr servo_sub_;
  SubscriberPtr load_cell_sub_;

  ThrustStandStatus thrust_stand_status_;

  std::shared_ptr<viewer::SimpleGeometry> servo_geo_;
  std::shared_ptr<viewer::SimpleGeometry> force_geo_;

  std::shared_ptr<viewer::Window3D> view_;

  bool print_thrust_stand_values_ = true;
};

}  // namespace visualization
}  // namespace jet
