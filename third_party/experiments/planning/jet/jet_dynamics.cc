#include "planning/jet/jet_dynamics.hh"

namespace planning {
namespace jet {
VecNd<13> to_vector(const StateDelta &in_grp) {
  const VecNd<13> out =
      (VecNd<13>() << ((in_grp.R_world_from_body_error_log)[0]),
       ((in_grp.R_world_from_body_error_log)[1]),
       ((in_grp.R_world_from_body_error_log)[2]), (in_grp.throttle_pct_error),
       ((in_grp.x_error)[0]), ((in_grp.x_error)[1]), ((in_grp.x_error)[2]),
       ((in_grp.w_error)[0]), ((in_grp.w_error)[1]), ((in_grp.w_error)[2]),
       ((in_grp.v_error)[0]), ((in_grp.v_error)[1]), ((in_grp.v_error)[2]))
          .finished();
  return out;
}
State operator-(const State &a, const State &b) {
  const State difference =
      State{((a.R_world_from_body) * ((b.R_world_from_body).inverse())),
            ((a.throttle_pct) - (b.throttle_pct)), ((a.x) - (b.x)),
            ((a.w) - (b.w)), ((a.v) - (b.v))};
  return difference;
}
VecNd<13> compute_delta(const State &a, const State &b) {
  const State difference = a - b;
  const SO3 R_world_from_body_error = difference.R_world_from_body;
  const VecNd<3> R_world_from_body_error_log =
      SO3::log(R_world_from_body_error);
  const double throttle_pct_error = difference.throttle_pct;
  const VecNd<3> x_error = difference.x;
  const VecNd<3> w_error = difference.w;
  const VecNd<3> v_error = difference.v;
  const StateDelta delta =
      StateDelta{R_world_from_body_error_log, throttle_pct_error, x_error,
                 w_error, v_error};
  const VecNd<13> out_vec = to_vector(delta);
  return out_vec;
}
double force_from_throttle(const double throttle) {
  const double out = throttle;
  return out;
}
StateDot compute_qdot(const State &Q, const Controls &U, const Parameters &Z) {
  const double mass = Z.mass;
  const double inv_mass = (1.0 / mass);
  const SO3 R_world_from_body = Q.R_world_from_body;
  const double throttle_pct = Q.throttle_pct;
  const double thrust = force_from_throttle(throttle_pct);
  const VecNd<3> unit_z = Z.unit_z;
  const VecNd<3> body_force = thrust * unit_z;
  const VecNd<3> force_world = R_world_from_body * body_force;
  const VecNd<3> external_force = Z.external_force;
  const VecNd<3> net_force_world = force_world + external_force;
  const VecNd<3> a = inv_mass * net_force_world;
  const VecNd<3> q = U.q;
  const double throttle_dot = U.throttle_dot;
  const VecNd<3> w = Q.w;
  const VecNd<3> v = Q.v;
  const StateDot Qdot = StateDot{w, throttle_dot, v, q, a};
  return Qdot;
}
StateDot operator*(const double h, const StateDot &K1) {
  const StateDot anon_c5ec58 =
      StateDot{(h * (K1.w)), (h * (K1.throttle_dot)), (h * (K1.v)),
               (h * (K1.q)), (h * (K1.a))};
  return anon_c5ec58;
}
State operator+(const State &Q, const StateDot &anon_763fee) {
  const State Q2 = State{((SO3::exp((anon_763fee.w))) * (Q.R_world_from_body)),
                         ((Q.throttle_pct) + (anon_763fee.throttle_dot)),
                         ((Q.x) + (anon_763fee.v)), ((Q.w) + (anon_763fee.q)),
                         ((Q.v) + (anon_763fee.a))};
  return Q2;
}
StateDot operator+(const StateDot &anon_c5ec58, const StateDot &anon_dd9cdd) {
  const StateDot anon_38882d = StateDot{
      ((anon_c5ec58.w) + (anon_dd9cdd.w)),
      ((anon_c5ec58.throttle_dot) + (anon_dd9cdd.throttle_dot)),
      ((anon_c5ec58.v) + (anon_dd9cdd.v)), ((anon_c5ec58.q) + (anon_dd9cdd.q)),
      ((anon_c5ec58.a) + (anon_dd9cdd.a))};
  return anon_38882d;
}
State rk4_integrate(const State &Q, const Controls &U, const Parameters &Z,
                    const double h) {
  const double half = 0.5;
  const double half_h = h * half;
  const StateDot K1 = compute_qdot(Q, U, Z);
  const State Q2 = Q + (half_h * (h * K1));
  const StateDot K2 = compute_qdot(Q2, U, Z);
  const State Q3 = Q + (half_h * (h * K2));
  const StateDot K3 = compute_qdot(Q3, U, Z);
  const State Q4 = Q + (h * (h * K3));
  const StateDot K4 = compute_qdot(Q4, U, Z);
  const double two = 2.0;
  const double sixth = 0.166666666667;
  const State Qn =
      Q + (sixth * (((h * K1) + (h * K4)) + (two * ((h * K2) + (h * K3)))));
  return Qn;
}
State operator+(const State &a, const StateDelta &grp_b) {
  const State out = State{
      ((SO3::exp((grp_b.R_world_from_body_error_log))) * (a.R_world_from_body)),
      ((a.throttle_pct) + (grp_b.throttle_pct_error)),
      ((a.x) + (grp_b.x_error)), ((a.w) + (grp_b.w_error)),
      ((a.v) + (grp_b.v_error))};
  return out;
}
StateDelta from_vector(const VecNd<13> &in_vec) {
  const VecNd<3> anon_aa324e =
      (VecNd<3>() << (in_vec[0]), (in_vec[1]), (in_vec[2])).finished();
  const VecNd<3> anon_5ea7ce =
      (VecNd<3>() << (in_vec[10]), (in_vec[11]), (in_vec[12])).finished();
  const VecNd<3> anon_32f58b =
      (VecNd<3>() << (in_vec[4]), (in_vec[5]), (in_vec[6])).finished();
  const VecNd<3> anon_8cd6c7 =
      (VecNd<3>() << (in_vec[7]), (in_vec[8]), (in_vec[9])).finished();
  const StateDelta out = StateDelta{anon_aa324e, (in_vec[3]), anon_32f58b,
                                    anon_8cd6c7, anon_5ea7ce};
  return out;
}
State apply_delta(const State &a, const VecNd<13> &delta) {
  const StateDelta grp_b = from_vector(delta);
  const State out = a + grp_b;
  return out;
}
Controls from_vector(const VecNd<4> &in_vec) {
  const VecNd<3> anon_6245a5 =
      (VecNd<3>() << (in_vec[0]), (in_vec[1]), (in_vec[2])).finished();
  const Controls out = Controls{anon_6245a5, (in_vec[3])};
  return out;
}
VecNd<4> to_vector(const Controls &in_grp) {
  const VecNd<4> out = (VecNd<4>() << ((in_grp.q)[0]), ((in_grp.q)[1]),
                        ((in_grp.q)[2]), (in_grp.throttle_dot))
                           .finished();
  return out;
}
} // namespace jet
} // namespace planning