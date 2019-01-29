#pragma once

#include "estimation/jet/jet_ekf.hh"

namespace estimation {
namespace jet_filter {

class JetFilter {
  using Update = FilterStateUpdate<State>;
  using JetFilterState = FilterState<State>;

 public:
  static JetFilterState reasonable_initial_state();

  JetFilter(const JetFilterState& xp);
  JetFilter();

  void reset(const JetFilterState& xp) {
    xp_ = xp;
    initialized_ = true;
  }

  void measure_imu(const AccelMeasurement& meas, const TimePoint& t);

  void measure_gyro(const GyroMeasurement& meas, const TimePoint& t);

  void measure_fiducial(const FiducialMeasurement& meas, const TimePoint& t);

  State free_run();
  jcc::Optional<State> next_measurement();

  State view(const TimePoint& t) const;

  const Parameters& parameters() const {
    return parameters_;
  }

  const JetFilterState& state() const {
    return xp_;
  }

 private:
  void setup_models();

  JetFilterState xp_;
  JetEkf ekf_;
  Parameters parameters_;

  bool initialized_ = false;

  int imu_id_ = -1;
  int fiducial_id_ = -1;
  int gyro_id_ = -1;
};

}  // namespace jet_filter
}  // namespace estimation
