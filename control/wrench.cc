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

Wrench operator+(const Wrench& wrench_a, const Wrench& wrench_b) {
  return Wrench({
      .force_N = wrench_a.force_N + wrench_b.force_N,
      .torque_Nm = wrench_a.torque_Nm + wrench_b.torque_Nm,
  });
}
// Wrench operator-(const Wrench& wrench_a, const Wrench& wrench_b) {
//   return Wrench({
//       .force_N = wrench_a.force_N - wrench_b.force_N,
//       .torque_Nm = wrench_a.torque_Nm - wrench_b.torque_Nm,
//   });
// }

}  // namespace control
}  // namespace jet
