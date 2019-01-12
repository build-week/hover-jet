#include "control/wrench.hh"

#include "third_party/experiments/eigen_helpers.hh"

// TODO
#include <iostream>

namespace jet {
namespace control {

Wrench wrench_from_force_arm(const jcc::Vec3& force_N, const jcc::Vec3& arm_m) {
  Wrench wrench;
  wrench.force_N = force_N;
  wrench.torque_Nm = arm_m.cross(force_N);
  return wrench;
}

Wrench operator+(const Wrench& w1, const Wrench& w2) {
  return Wrench({
      .force_N = w1.force_N + w2.force_N,       //
      .torque_Nm = w1.torque_Nm + w2.torque_Nm  //
  });
}

Wrench wrench_in_frame(const SE3& frame_2_from_frame_1, const Wrench& wrench_frame_1) {
  // TODO(Matt, Jake): One should expect this to work, why doesn't it?
  // Explore: It's not transposed; the txR block is just *swapped*
  /*  const jcc::Vec6 ft_frame_1 = jcc::vstack(wrench_frame_1.force_N, wrench_frame_1.torque_Nm);
    const jcc::Vec6 ft_frame_2 = frame_2_from_frame_1.Adj() * ft_frame_1;
    return Wrench({
        .force_N = ft_frame_2.head<3>(),   //
        .torque_Nm = ft_frame_2.tail<3>()  //
    });
  */

  const jcc::Vec3 force_frame_2 = frame_2_from_frame_1.so3() * wrench_frame_1.force_N;
  const jcc::Vec3 torque_frame_2 = (frame_2_from_frame_1.so3() * wrench_frame_1.torque_Nm) +
                                   (frame_2_from_frame_1.translation().cross(force_frame_2));
  return Wrench({
      .force_N = force_frame_2,    //
      .torque_Nm = torque_frame_2  //
  });
}

void print_wrench(const Wrench& wrench) {
  std::cout << wrench.force_N.transpose() << "; " << wrench.torque_Nm.transpose() << std::endl;
}

}  // namespace control
}  // namespace jet
