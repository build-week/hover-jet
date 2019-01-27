#include "estimation/jet/jet_optimizer.hh"

#include "numerics/set_diag_to_value.hh"

namespace estimation {
namespace jet_filter {

JetOptimizer::JetOptimizer() {
  MatNd<AccelMeasurement::DIM, AccelMeasurement::DIM> accel_cov;
  accel_cov.setZero();
  {
    numerics::set_diag_to_value<AccelMeasurementDelta::observed_acceleration_error_dim,
                                AccelMeasurementDelta::observed_acceleration_error_ind>(
        accel_cov, 0.01);
  }

  MatNd<FiducialMeasurement::DIM, FiducialMeasurement::DIM> fiducial_cov;
  fiducial_cov.setZero();
  {
    fiducial_cov.block<3, 3>(0, 0) = MatNd<3, 3>::Identity() * 0.0001;
    fiducial_cov.block<3, 3>(3, 3) = MatNd<3, 3>::Identity() * 0.00001;
  }
  MatNd<State::DIM, State::DIM> state_cov;
  {
    state_cov.setZero();
    numerics::set_diag_to_value<StateDelta::accel_bias_error_dim,
                                StateDelta::accel_bias_error_ind>(state_cov, 0.001);
    numerics::set_diag_to_value<StateDelta::gyro_bias_error_dim,
                                StateDelta::gyro_bias_error_ind>(state_cov, 0.001);
    numerics::set_diag_to_value<StateDelta::eps_dot_error_dim,
                                StateDelta::eps_dot_error_ind>(state_cov, 0.01);
    numerics::set_diag_to_value<StateDelta::eps_ddot_error_dim,
                                StateDelta::eps_ddot_error_ind>(state_cov, 12.0);

    constexpr int T_error_dim = StateDelta::T_body_from_world_error_log_dim;
    constexpr int T_error_ind = StateDelta::T_body_from_world_error_log_ind;
    numerics::set_diag_to_value<T_error_dim, T_error_ind>(state_cov, 0.01);
  }

  const MatNd<GyroMeasurement::DIM, GyroMeasurement::DIM> gyro_cov = accel_cov;

  imu_id_ =
      pose_opt_.add_error_model<AccelMeasurement>(observe_accel_error_model, accel_cov);
  gyro_id_ =
      pose_opt_.add_error_model<GyroMeasurement>(observe_gyro_error_model, gyro_cov);
  fiducial_id_ =
      pose_opt_.add_error_model<FiducialMeasurement>(fiducial_error_model, fiducial_cov);

  pose_opt_.set_dynamics_cov(state_cov);
}

void JetOptimizer::measure_imu(const AccelMeasurement& meas, const TimePoint& t) {
  pose_opt_.add_measurement(meas, t, imu_id_);
}

void JetOptimizer::measure_fiducial(const FiducialMeasurement& meas, const TimePoint& t) {
  pose_opt_.add_measurement(meas, t, fiducial_id_);
}

void JetOptimizer::measure_gyro(const GyroMeasurement& meas, const TimePoint& t) {
  pose_opt_.add_measurement(meas, t, gyro_id_);
}

JetPoseOptimizer::Solution JetOptimizer::solve(const std::vector<State> x,
                                               const Parameters& p,
                                               const JetPoseOptimizer::Visitor& visitor) {
  return pose_opt_.solve({x, p}, visitor);
}

}  // namespace jet_filter
}  // namespace estimation
