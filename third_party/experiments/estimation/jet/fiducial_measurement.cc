#include "estimation/jet/fiducial_measurement.hh"

namespace estimation {
namespace jet_filter {

const SE3 fiducial_1_from_world;

FiducialMeasurement observe_fiducial(const State& x, const Parameters& p) {
  FiducialMeasurement meas;
  // const SE3 T_camera_from_world = p.T_camera_from_body * x.T_body_from_world;
  const SE3 T_camera_from_body = SE3();
  const SE3 T_camera_from_world = T_camera_from_body * x.T_body_from_world;

  meas.T_fiducial_from_camera = fiducial_1_from_world * T_camera_from_world.inverse();
  return meas;
}

VecNd<6> fiducial_error_model(const State& x,
                              const FiducialMeasurement& z,
                              const Parameters& p) {
  /*  const SE3 measured_body_from_fiducial =
        (z.T_fiducial_from_camera * p.T_camera_from_body).inverse();
    const SE3 expected_body_from_fiducial =
        x.T_body_from_world * fiducial_1_from_world.inverse();
    const SE3 error = measured_body_from_fiducial * expected_body_from_fiducial;
  */

  const auto expected_fiducial = observe_fiducial(x, p);
  const SE3 error =
      z.T_fiducial_from_camera * expected_fiducial.T_fiducial_from_camera.inverse();

  return SE3::log(error);
}
}  // namespace jet_filter
}  // namespace estimation