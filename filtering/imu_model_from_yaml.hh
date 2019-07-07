#pragma once

#include "filtering/yaml_matrix.hh"

#include "third_party/experiments/sophus.hh"
//%deps(estimate_imu_intrinsics)
#include "third_party/experiments/estimation/calibration/estimate_imu_intrinsics.hh"

namespace jet {

struct ImuExtrinsicsIntrinsics {
  estimation::ImuModel imu_model;
  int imu_id;
};

ImuExtrinsicsIntrinsics imu_model_from_yaml(const YAML::Node& node);
}  // namespace jet