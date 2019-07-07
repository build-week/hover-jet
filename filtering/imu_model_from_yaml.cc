#include "filtering/imu_model_from_yaml.hh"

namespace jet {

ImuExtrinsicsIntrinsics imu_model_from_yaml(const YAML::Node& node) {
  estimation::ImuIntrinsics intrinsics;
  {
    read_matrix(node["accelerometer_gains_p0"], intrinsics.imu_gains.p0);
    read_matrix(node["accelerometer_gains_scaling"], intrinsics.imu_gains.cholesky_factor);

    read_matrix(node["magnetometer_gains_p0"], intrinsics.magnetometer_gains.p0);
    read_matrix(node["magnetometer_gains_scaling"], intrinsics.magnetometer_gains.cholesky_factor);
  }
  const estimation::ImuModel model(intrinsics);
  const ImuExtrinsicsIntrinsics calibration{.imu_model = model, .imu_id = node["id"].as<int>()};
  return calibration;
}

}  // namespace jet