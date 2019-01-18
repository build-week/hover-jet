/* Don't edit this; this code was generated by op_graph */
#include "control/vanes_generated.hh"

namespace jet {
namespace control {
VecNd<4>
QuadraframeStatusDelta::to_vector(const QuadraframeStatusDelta &in_grp) {
  const VecNd<4> out =
      (VecNd<4>() << (in_grp.servo_0_angle_rad_error),
       (in_grp.servo_1_angle_rad_error), (in_grp.servo_2_angle_rad_error),
       (in_grp.servo_3_angle_rad_error))
          .finished();
  return out;
}
QuadraframeStatusDelta
QuadraframeStatusDelta::from_vector(const VecNd<4> &in_vec) {
  const QuadraframeStatusDelta out = QuadraframeStatusDelta{
      (in_vec[0]), (in_vec[1]), (in_vec[2]), (in_vec[3])};
  return out;
}
QuadraframeStatus operator-(const QuadraframeStatus &a,
                            const QuadraframeStatus &b) {
  const QuadraframeStatus difference =
      QuadraframeStatus{((a.servo_0_angle_rad) - (b.servo_0_angle_rad)),
                        ((a.servo_1_angle_rad) - (b.servo_1_angle_rad)),
                        ((a.servo_2_angle_rad) - (b.servo_2_angle_rad)),
                        ((a.servo_3_angle_rad) - (b.servo_3_angle_rad))};
  return difference;
}
QuadraframeStatus operator+(const QuadraframeStatus &a,
                            const QuadraframeStatusDelta &grp_b) {
  const QuadraframeStatus out = QuadraframeStatus{
      ((a.servo_0_angle_rad) + (grp_b.servo_0_angle_rad_error)),
      ((a.servo_1_angle_rad) + (grp_b.servo_1_angle_rad_error)),
      ((a.servo_2_angle_rad) + (grp_b.servo_2_angle_rad_error)),
      ((a.servo_3_angle_rad) + (grp_b.servo_3_angle_rad_error))};
  return out;
}
VecNd<4> QuadraframeStatus::compute_delta(const QuadraframeStatus &a,
                                          const QuadraframeStatus &b) {
  const QuadraframeStatus difference = a - b;
  const double servo_3_angle_rad_error = difference.servo_3_angle_rad;
  const double servo_2_angle_rad_error = difference.servo_2_angle_rad;
  const double servo_0_angle_rad_error = difference.servo_0_angle_rad;
  const double servo_1_angle_rad_error = difference.servo_1_angle_rad;
  const QuadraframeStatusDelta delta =
      QuadraframeStatusDelta{servo_0_angle_rad_error, servo_1_angle_rad_error,
                             servo_2_angle_rad_error, servo_3_angle_rad_error};
  const VecNd<4> out_vec = QuadraframeStatusDelta::to_vector(delta);
  return out_vec;
}
QuadraframeStatus QuadraframeStatus::apply_delta(const QuadraframeStatus &a,
                                                 const VecNd<4> &delta) {
  const QuadraframeStatusDelta grp_b =
      QuadraframeStatusDelta::from_vector(delta);
  const QuadraframeStatus out = a + grp_b;
  return out;
}
} // namespace control
} // namespace jet