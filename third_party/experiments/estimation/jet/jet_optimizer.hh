#pragma once

#include "estimation/jet/jet_pose_opt.hh"
#include "estimation/jet/jet_rk4.hh"

// TODO
#include "estimation/jet/fiducial_measurement.hh"

namespace estimation {
namespace jet_filter {

class JetOptimizer {
 public:
  JetOptimizer();

  void measure_imu(const AccelMeasurement& meas, const TimePoint& t);

  void measure_fiducial(const FiducialMeasurement& meas, const TimePoint& t);

  void measure_gyro(const GyroMeasurement& meas, const TimePoint& t);

  JetPoseOptimizer::Solution solve(const std::vector<State> x,
                                   const Parameters& p,
                                   const JetPoseOptimizer::Visitor& visitor = {});

 private:
  JetPoseOptimizer pose_opt_{rk4_integrate};
  int imu_id_ = -1;
  int gyro_id_ = -1;
  int fiducial_id_ = -1;
};
}  // namespace jet_filter
}  // namespace estimation
