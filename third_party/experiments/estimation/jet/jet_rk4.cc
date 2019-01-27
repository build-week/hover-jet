#include "estimation/jet/jet_rk4.hh"

namespace estimation {
namespace jet_filter {
VecNd<24> StateDelta::to_vector(const StateDelta &in_grp) {
  const VecNd<24> out =
      (VecNd<24>() << ((in_grp.accel_bias_error)[0]),
       ((in_grp.accel_bias_error)[1]), ((in_grp.accel_bias_error)[2]),
       ((in_grp.eps_ddot_error)[0]), ((in_grp.eps_ddot_error)[1]),
       ((in_grp.eps_ddot_error)[2]), ((in_grp.eps_ddot_error)[3]),
       ((in_grp.eps_ddot_error)[4]), ((in_grp.eps_ddot_error)[5]),
       ((in_grp.eps_dot_error)[0]), ((in_grp.eps_dot_error)[1]),
       ((in_grp.eps_dot_error)[2]), ((in_grp.eps_dot_error)[3]),
       ((in_grp.eps_dot_error)[4]), ((in_grp.eps_dot_error)[5]),
       ((in_grp.gyro_bias_error)[0]), ((in_grp.gyro_bias_error)[1]),
       ((in_grp.gyro_bias_error)[2]), ((in_grp.T_body_from_world_error_log)[0]),
       ((in_grp.T_body_from_world_error_log)[1]),
       ((in_grp.T_body_from_world_error_log)[2]),
       ((in_grp.T_body_from_world_error_log)[3]),
       ((in_grp.T_body_from_world_error_log)[4]),
       ((in_grp.T_body_from_world_error_log)[5]))
          .finished();
  return out;
}
StateDelta StateDelta::from_vector(const VecNd<24> &in_vec) {
  const VecNd<6> anon_e66f16 =
      (VecNd<6>() << (in_vec[9]), (in_vec[10]), (in_vec[11]), (in_vec[12]),
       (in_vec[13]), (in_vec[14]))
          .finished();
  const VecNd<3> anon_4e53c2 =
      (VecNd<3>() << (in_vec[0]), (in_vec[1]), (in_vec[2])).finished();
  const VecNd<6> anon_116ef6 =
      (VecNd<6>() << (in_vec[3]), (in_vec[4]), (in_vec[5]), (in_vec[6]),
       (in_vec[7]), (in_vec[8]))
          .finished();
  const VecNd<3> anon_b3b450 =
      (VecNd<3>() << (in_vec[15]), (in_vec[16]), (in_vec[17])).finished();
  const VecNd<6> anon_105771 =
      (VecNd<6>() << (in_vec[18]), (in_vec[19]), (in_vec[20]), (in_vec[21]),
       (in_vec[22]), (in_vec[23]))
          .finished();
  const StateDelta out = StateDelta{anon_4e53c2, anon_116ef6, anon_e66f16,
                                    anon_b3b450, anon_105771};
  return out;
}
Parameters operator-(const Parameters &a, const Parameters &b) {
  const Parameters difference =
      Parameters{((a.T_imu_from_vehicle) * ((b.T_imu_from_vehicle).inverse()))};
  return difference;
}
Parameters operator+(const Parameters &a, const ParametersDelta &grp_b) {
  const Parameters out =
      Parameters{((SE3::exp((grp_b.T_imu_from_vehicle_error_log))) *
                  (a.T_imu_from_vehicle))};
  return out;
}
VecNd<6> Parameters::compute_delta(const Parameters &a, const Parameters &b) {
  const Parameters difference = a - b;
  const SE3 T_imu_from_vehicle_error = difference.T_imu_from_vehicle;
  const VecNd<6> T_imu_from_vehicle_error_log =
      SE3::log(T_imu_from_vehicle_error);
  const ParametersDelta delta = ParametersDelta{T_imu_from_vehicle_error_log};
  const VecNd<6> out_vec = ParametersDelta::to_vector(delta);
  return out_vec;
}
Parameters Parameters::apply_delta(const Parameters &a, const VecNd<6> &delta) {
  const ParametersDelta grp_b = ParametersDelta::from_vector(delta);
  const Parameters out = a + grp_b;
  return out;
}
VecNd<3> GyroMeasurementDelta::to_vector(const GyroMeasurementDelta &in_grp) {
  const VecNd<3> out =
      (VecNd<3>() << ((in_grp.observed_w_error)[0]),
       ((in_grp.observed_w_error)[1]), ((in_grp.observed_w_error)[2]))
          .finished();
  return out;
}
GyroMeasurementDelta GyroMeasurementDelta::from_vector(const VecNd<3> &in_vec) {
  const VecNd<3> anon_a45706 =
      (VecNd<3>() << (in_vec[0]), (in_vec[1]), (in_vec[2])).finished();
  const GyroMeasurementDelta out = GyroMeasurementDelta{anon_a45706};
  return out;
}
GyroMeasurement operator-(const GyroMeasurement &a, const GyroMeasurement &b) {
  const GyroMeasurement difference =
      GyroMeasurement{((a.observed_w) - (b.observed_w))};
  return difference;
}
GyroMeasurement operator+(const GyroMeasurement &a,
                          const GyroMeasurementDelta &grp_b) {
  const GyroMeasurement out =
      GyroMeasurement{((a.observed_w) + (grp_b.observed_w_error))};
  return out;
}
VecNd<3> GyroMeasurement::compute_delta(const GyroMeasurement &a,
                                        const GyroMeasurement &b) {
  const GyroMeasurement difference = a - b;
  const VecNd<3> observed_w_error = difference.observed_w;
  const GyroMeasurementDelta delta = GyroMeasurementDelta{observed_w_error};
  const VecNd<3> out_vec = GyroMeasurementDelta::to_vector(delta);
  return out_vec;
}
GyroMeasurement GyroMeasurement::apply_delta(const GyroMeasurement &a,
                                             const VecNd<3> &delta) {
  const GyroMeasurementDelta grp_b = GyroMeasurementDelta::from_vector(delta);
  const GyroMeasurement out = a + grp_b;
  return out;
}
State operator-(const State &a, const State &b) {
  const State difference =
      State{((a.accel_bias) - (b.accel_bias)), ((a.eps_ddot) - (b.eps_ddot)),
            ((a.eps_dot) - (b.eps_dot)), ((a.gyro_bias) - (b.gyro_bias)),
            ((a.T_body_from_world) * ((b.T_body_from_world).inverse()))};
  return difference;
}
State operator+(const State &a, const StateDelta &grp_b) {
  const State out = State{((a.accel_bias) + (grp_b.accel_bias_error)),
                          ((a.eps_ddot) + (grp_b.eps_ddot_error)),
                          ((a.eps_dot) + (grp_b.eps_dot_error)),
                          ((a.gyro_bias) + (grp_b.gyro_bias_error)),
                          ((SE3::exp((grp_b.T_body_from_world_error_log))) *
                           (a.T_body_from_world))};
  return out;
}
VecNd<24> State::compute_delta(const State &a, const State &b) {
  const State difference = a - b;
  const VecNd<3> accel_bias_error = difference.accel_bias;
  const VecNd<6> eps_ddot_error = difference.eps_ddot;
  const VecNd<6> eps_dot_error = difference.eps_dot;
  const VecNd<3> gyro_bias_error = difference.gyro_bias;
  const SE3 T_body_from_world_error = difference.T_body_from_world;
  const VecNd<6> T_body_from_world_error_log =
      SE3::log(T_body_from_world_error);
  const StateDelta delta =
      StateDelta{accel_bias_error, eps_ddot_error, eps_dot_error,
                 gyro_bias_error, T_body_from_world_error_log};
  const VecNd<24> out_vec = StateDelta::to_vector(delta);
  return out_vec;
}
State State::apply_delta(const State &a, const VecNd<24> &delta) {
  const StateDelta grp_b = StateDelta::from_vector(delta);
  const State out = a + grp_b;
  return out;
}
AccelMeasurement operator-(const AccelMeasurement &a,
                           const AccelMeasurement &b) {
  const AccelMeasurement difference =
      AccelMeasurement{((a.observed_acceleration) - (b.observed_acceleration))};
  return difference;
}
AccelMeasurement operator+(const AccelMeasurement &a,
                           const AccelMeasurementDelta &grp_b) {
  const AccelMeasurement out = AccelMeasurement{
      ((a.observed_acceleration) + (grp_b.observed_acceleration_error))};
  return out;
}
VecNd<3> AccelMeasurement::compute_delta(const AccelMeasurement &a,
                                         const AccelMeasurement &b) {
  const AccelMeasurement difference = a - b;
  const VecNd<3> observed_acceleration_error = difference.observed_acceleration;
  const AccelMeasurementDelta delta =
      AccelMeasurementDelta{observed_acceleration_error};
  const VecNd<3> out_vec = AccelMeasurementDelta::to_vector(delta);
  return out_vec;
}
AccelMeasurement AccelMeasurement::apply_delta(const AccelMeasurement &a,
                                               const VecNd<3> &delta) {
  const AccelMeasurementDelta grp_b = AccelMeasurementDelta::from_vector(delta);
  const AccelMeasurement out = a + grp_b;
  return out;
}
VecNd<6> ParametersDelta::to_vector(const ParametersDelta &in_grp) {
  const VecNd<6> out =
      (VecNd<6>() << ((in_grp.T_imu_from_vehicle_error_log)[0]),
       ((in_grp.T_imu_from_vehicle_error_log)[1]),
       ((in_grp.T_imu_from_vehicle_error_log)[2]),
       ((in_grp.T_imu_from_vehicle_error_log)[3]),
       ((in_grp.T_imu_from_vehicle_error_log)[4]),
       ((in_grp.T_imu_from_vehicle_error_log)[5]))
          .finished();
  return out;
}
ParametersDelta ParametersDelta::from_vector(const VecNd<6> &in_vec) {
  const VecNd<6> anon_372276 =
      (VecNd<6>() << (in_vec[0]), (in_vec[1]), (in_vec[2]), (in_vec[3]),
       (in_vec[4]), (in_vec[5]))
          .finished();
  const ParametersDelta out = ParametersDelta{anon_372276};
  return out;
}
VecNd<3> AccelMeasurementDelta::to_vector(const AccelMeasurementDelta &in_grp) {
  const VecNd<3> out = (VecNd<3>() << ((in_grp.observed_acceleration_error)[0]),
                        ((in_grp.observed_acceleration_error)[1]),
                        ((in_grp.observed_acceleration_error)[2]))
                           .finished();
  return out;
}
AccelMeasurementDelta
AccelMeasurementDelta::from_vector(const VecNd<3> &in_vec) {
  const VecNd<3> anon_30033b =
      (VecNd<3>() << (in_vec[0]), (in_vec[1]), (in_vec[2])).finished();
  const AccelMeasurementDelta out = AccelMeasurementDelta{anon_30033b};
  return out;
}
StateDot compute_qdot(const State &Q, const Parameters &Z) {
  const VecNd<3> gyro_bias_dot = VecNd<3>::Zero();
  const VecNd<6> eps_dot = Q.eps_dot;
  const VecNd<6> eps_ddot = Q.eps_ddot;
  const VecNd<3> accel_bias_dot = VecNd<3>::Zero();
  const VecNd<6> eps_ddot_dot = VecNd<6>::Zero();
  const StateDot Qdot =
      StateDot{accel_bias_dot, eps_ddot_dot, eps_ddot, gyro_bias_dot, eps_dot};
  return Qdot;
}
StateDot operator*(const double h, const StateDot &K1) {
  const StateDot anon_997f0c = StateDot{
      (h * (K1.accel_bias_dot)), (h * (K1.eps_ddot_dot)), (h * (K1.eps_ddot)),
      (h * (K1.gyro_bias_dot)), (h * (K1.eps_dot))};
  return anon_997f0c;
}
State operator+(const State &Q, const StateDot &anon_106530) {
  const State Q2 =
      State{((Q.accel_bias) + (anon_106530.accel_bias_dot)),
            ((Q.eps_ddot) + (anon_106530.eps_ddot_dot)),
            ((Q.eps_dot) + (anon_106530.eps_ddot)),
            ((Q.gyro_bias) + (anon_106530.gyro_bias_dot)),
            ((SE3::exp((anon_106530.eps_dot))) * (Q.T_body_from_world))};
  return Q2;
}
StateDot operator+(const StateDot &anon_997f0c, const StateDot &anon_56ce14) {
  const StateDot anon_1f1812 =
      StateDot{((anon_997f0c.accel_bias_dot) + (anon_56ce14.accel_bias_dot)),
               ((anon_997f0c.eps_ddot_dot) + (anon_56ce14.eps_ddot_dot)),
               ((anon_997f0c.eps_ddot) + (anon_56ce14.eps_ddot)),
               ((anon_997f0c.gyro_bias_dot) + (anon_56ce14.gyro_bias_dot)),
               ((anon_997f0c.eps_dot) + (anon_56ce14.eps_dot))};
  return anon_1f1812;
}
State rk4_integrate(const State &Q, const Parameters &Z, const double h) {
  const double half = 0.5;
  const double half_h = h * half;
  const StateDot K1 = compute_qdot(Q, Z);
  const State Q2 = Q + (half_h * (h * K1));
  const StateDot K2 = compute_qdot(Q2, Z);
  const State Q3 = Q + (half_h * (h * K2));
  const StateDot K3 = compute_qdot(Q3, Z);
  const State Q4 = Q + (h * (h * K3));
  const StateDot K4 = compute_qdot(Q4, Z);
  const double two = 2.0;
  const double sixth = 0.166666666667;
  const State Qn =
      Q + (sixth * (((h * K1) + (h * K4)) + (two * ((h * K2) + (h * K3)))));
  return Qn;
}
GyroMeasurement observe_gyro(const State &state, const Parameters &parameters) {
  const VecNd<6> eps_dot = state.eps_dot;
  const VecNd<3> w = eps_dot.block<3, 1>(3, 0);
  const SE3 imu_from_vehicle = parameters.T_imu_from_vehicle;
  const VecNd<3> gyro_bias = state.gyro_bias;
  const SO3 R_sensor_from_vehicle = imu_from_vehicle.so3();
  const VecNd<3> observed_w = gyro_bias - (R_sensor_from_vehicle * w);
  const GyroMeasurement gyro_meas = GyroMeasurement{observed_w};
  return gyro_meas;
}
AccelMeasurement observe_accel(const State &state,
                               const Parameters &parameters) {
  const VecNd<6> eps_dot = state.eps_dot;
  const SE3 imu_from_vehicle = parameters.T_imu_from_vehicle;
  const SE3 vehicle_from_world = state.T_body_from_world;
  const SO3 R_sensor_from_world = (imu_from_vehicle * vehicle_from_world).so3();
  const double g_mpss = 9.81;
  const VecNd<3> unit_z = VecNd<3>::UnitZ();
  const MatNd<6, 6> adj = imu_from_vehicle.Adj();
  const VecNd<3> w_imu = (adj * eps_dot).block<3, 1>(3, 0);
  const VecNd<3> v_imu = (adj * eps_dot).block<3, 1>(0, 0);
  const VecNd<3> g_world = unit_z * g_mpss;
  const VecNd<6> eps_ddot = state.eps_ddot;
  const VecNd<3> a_imu = (adj * eps_ddot).block<3, 1>(0, 0);
  const VecNd<3> clean = (w_imu.cross(v_imu)) - a_imu;
  const VecNd<3> g_imu = R_sensor_from_world * g_world;
  const VecNd<3> accel_bias = state.accel_bias;
  const VecNd<3> observed_acceleration = (clean + g_imu) + accel_bias;
  const AccelMeasurement accel_meas = AccelMeasurement{observed_acceleration};
  return accel_meas;
}
VecNd<3> observe_accel_error_model(const State &state,
                                   const AccelMeasurement &meas,
                                   const Parameters &parameters) {
  const AccelMeasurement expected = observe_accel(state, parameters);
  const VecNd<3> error = AccelMeasurement::compute_delta(meas, expected);
  return error;
}
VecNd<3> observe_gyro_error_model(const State &state,
                                  const GyroMeasurement &meas,
                                  const Parameters &parameters) {
  const GyroMeasurement expected = observe_gyro(state, parameters);
  const VecNd<3> error = GyroMeasurement::compute_delta(meas, expected);
  return error;
}
} // namespace jet_filter
} // namespace estimation