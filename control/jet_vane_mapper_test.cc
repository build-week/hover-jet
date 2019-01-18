#include "control/jet_vane_mapper.hh"

namespace jet {
namespace control {

void go() {
  JetVaneMapper mapper;
  // Randomly chosen wrench; verifying that it actually produces the desired wrench
  const Wrench wrench({.force_N = jcc::Vec3(2.92087, 1.11022e-16, -2.53459),
                       .torque_Nm = jcc::Vec3(-0.00415546, -0.0277483, -0.0487785)});

  JetStatus jet_status({.throttle = 1.0});

  mapper.map_wrench(wrench, jet_status);
}

}  // namespace control
}  // namespace jet

int main() {
  jet::control::go();
}
