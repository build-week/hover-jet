//%deps(jet_filter)
//%deps(jet_optimizer)
//%deps(nonlinear_camera_model)
//%deps(warn_sensor_rates)
//%deps(make_interpolator)
#include "third_party/experiments/estimation/sensors/make_interpolator.hh"

#include "third_party/experiments/estimation/calibration/nonlinear_camera_model.hh"
#include "third_party/experiments/estimation/calibration/warn_sensor_rates.hh"
#include "third_party/experiments/estimation/jet/jet_filter.hh"
#include "third_party/experiments/estimation/jet/jet_optimizer.hh"

//%deps(calibrate_single_imu)
//%deps(robust_pnp)
//%deps(visualize_calibration)
//%deps(visualize_camera_calibration)
#include "third_party/experiments/estimation/calibration/calibrate_single_imu.hh"
#include "third_party/experiments/estimation/vision/robust_pnp.hh"
#include "third_party/experiments/estimation/visualization/visualize_calibration.hh"
#include "third_party/experiments/estimation/visualization/visualize_camera_calibration.hh"

//%deps(form_coordinate_frame)
#include "third_party/experiments/geometry/spatial/form_coordinate_frame.hh"

//%deps(simple_geometry)
//%deps(ui2d)
//%deps(window_3d)
#include "third_party/experiments/viewer/interaction/ui2d.hh"
#include "third_party/experiments/viewer/primitives/simple_geometry.hh"
#include "third_party/experiments/viewer/window_3d.hh"

//%deps(jet_model)
#include "third_party/experiments/planning/jet/jet_model.hh"
//%deps(log)
#include "third_party/experiments/logging/log.hh"

#include "filtering/extract_data_from_log.hh"

#include "camera/camera_manager.hh"
#include "vision/fiducial_detection_and_pose.hh"

// - Validate reprojection of fiducial detection using the packaged calibration
// - Validate opencv calibration
// x - Estimate IMU intrinsics
// x - Estimate direction of gravity
// - 2D imageview

namespace jet {
namespace filtering {

namespace {
constexpr int IMU_1 = 78;
constexpr int IMU_2 = 36;

estimation::ProjectionCoefficients proj_coeffs_from_opencv(const Calibration& cal) {
  estimation::ProjectionCoefficients model;
  model.fx = cal.camera_matrix.at<double>(0, 0);
  model.fy = cal.camera_matrix.at<double>(1, 1);
  model.cx = cal.camera_matrix.at<double>(0, 2);
  model.cy = cal.camera_matrix.at<double>(1, 2);
  model.k1 = cal.distortion_coefficients.at<double>(0);
  model.k2 = cal.distortion_coefficients.at<double>(1);
  model.p1 = cal.distortion_coefficients.at<double>(2);
  model.p2 = cal.distortion_coefficients.at<double>(3);
  model.k3 = cal.distortion_coefficients.at<double>(4);

  model.rows = cal.rows;
  model.cols = cal.cols;

  return model;
}

std::vector<estimation::ObjectImageAssociations> object_image_assoc_from_board_point(
    const std::vector<BoardPointImagePointAssociation>& obj_pt_associations) {
  std::vector<estimation::ObjectImageAssociations> obj_image;

  for (const auto& assoc : obj_pt_associations) {
    const jcc::Vec2 pt_board_surface = jcc::Vec2(assoc.point_board_space);
    const jcc::Vec3 observed_pt_board_frame = jcc::Vec3(pt_board_surface.x(), pt_board_surface.y(), 0.0);
    const jcc::Vec2 observed_pt_image = assoc.point_image_space;
    obj_image.push_back({observed_pt_image, observed_pt_board_frame});
  }
  return obj_image;
}

void align_crappy_jake(const std::shared_ptr<viewer::SimpleGeometry>& geo,
                       const std::shared_ptr<viewer::Ui2d>& ui2d,
                       const estimation::NonlinearCameraModel& model,
                       const SE3& fiducial_from_camera_init,
                       const std::vector<BoardPointImagePointAssociation>& obj_pt_associations) {
  std::vector<jcc::Vec2> observed;
  std::vector<jcc::Vec3> object;

  const auto associations = object_image_assoc_from_board_point(obj_pt_associations);

  const auto pnp_visitor = estimation::make_pnp_visitor(model, object, observed);
  const auto pnp_result = estimation::robust_pnp(model, fiducial_from_camera_init, object, observed, pnp_visitor);
}

}  // namespace

const estimation::CreateSingleImuModelConfig imu_cal_cfg{
    .visualize_imu_model = false,    //
    .visualize_gyro = false,         //
    .visualize_magnetometer = false  //
};

const estimation::CameraCalibrationConfig camera_cal_cfg{
    .visualize_camera = false,             //
    .visualize_camera_distortion = false,  //
    .visualize_camera_frustum = false      //
};

bool visualize() {
  return (camera_cal_cfg.visualize_camera || camera_cal_cfg.visualize_camera_distortion ||
          camera_cal_cfg.visualize_camera_frustum || imu_cal_cfg.visualize_imu_model || imu_cal_cfg.visualize_gyro ||
          imu_cal_cfg.visualize_magnetometer);
}

void setup() {
  if (visualize()) {
    const auto view = viewer::get_window3d("Calibration");
    view->set_continue_time_ms(1);
    const auto background = view->add_primitive<viewer::SimpleGeometry>();
    const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
    background->add_plane({ground, 0.1});
    background->flip();
  }
}

struct JetModel {
  std::map<int, estimation::SingleImuCalibration> imu_calibration_from_imu_id;
};

jcc::Optional<ejf::FiducialMeasurement> find_nearest_fiducial(
    const estimation::calibration::CalibrationMeasurements& cal_meas, const estimation::TimePoint& t) {
  JASSERT_FALSE(cal_meas.fiducial_meas.empty(), "Cannot calibrate with empty fiducial measurements");
  for (const auto& fiducial_meas : cal_meas.fiducial_meas) {
    if (t < fiducial_meas.timestamp) {
      return {fiducial_meas.measurement};
    }
  }
  return {};
}

SE3 compute_world_from_camera(const estimation::SingleImuCalibration& imu_cal) {
  const geometry::Unit3 g_camera_frame = imu_cal.camera_from_gyro * imu_cal.g_estimate.direction;
  const SO3 world_from_camera = geometry::spatial::form_coordinate_frame_from_zhat(g_camera_frame);
  return SE3(world_from_camera, jcc::Vec3::Zero());
}

ejf::Parameters compute_parameters(const estimation::calibration::CalibrationMeasurements& cal_meas,
                                   const JetModel& jet_model) {
  const auto& imu_cal_1 = jet_model.imu_calibration_from_imu_id.at(IMU_1);
  const auto& imu_cal_2 = jet_model.imu_calibration_from_imu_id.at(IMU_2);

  const SO3 world_from_camera_1 = compute_world_from_camera(imu_cal_1).so3();
  const SO3 world_from_camera_2 = compute_world_from_camera(imu_cal_2).so3();
  jcc::Warning() << "[Filter Setup] IMU->IMU Alignment Error: "
                 << (world_from_camera_1 * world_from_camera_2.inverse()).log().norm() << std::endl;

  const auto maybe_nearest_fiducial = find_nearest_fiducial(cal_meas, imu_cal_1.g_estimate.time);
  assert(maybe_nearest_fiducial);
  const auto fiducial_at_g_estimate = *maybe_nearest_fiducial;
  const SO3 fiducial_from_camera = fiducial_at_g_estimate.T_fiducial_from_camera.so3();
  const SO3 world_from_fiducial = world_from_camera_1 * fiducial_from_camera.inverse();

  auto p = ejf::JetFilter::reasonable_parameters();
  {
    p.T_world_from_fiducial = SE3(world_from_fiducial, jcc::Vec3::Zero());
    p.T_imu1_from_vehicle = SE3(imu_cal_1.camera_from_gyro.inverse(), jcc::Vec3::Zero());
    p.T_imu2_from_vehicle = SE3(imu_cal_2.camera_from_gyro.inverse(), jcc::Vec3::Zero());
  }

  return p;
}

void test_filter(const estimation::calibration::CalibrationMeasurements& cal_meas, const JetModel& jet_model) {
  const auto view = viewer::get_window3d("Calibration");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  const auto geo_stable = view->add_primitive<viewer::SimpleGeometry>();
  const auto jet_tree = view->add_primitive<viewer::SceneTree>();

  const jcc::Vec3 jet_origin(1.0, 0.0, 0.0);

  const planning::jet::JetModel jet_3dmodel;
  constexpr bool DRAW_VEHICLE = true;
  const SO3 jetmodel_from_body = jcc::exp_x(M_PI * 0.5).so3();
  if constexpr (DRAW_VEHICLE) {
    jet_3dmodel.insert(*jet_tree);
  }

  const auto& imu_1_meas = cal_meas.imu_cal.at(IMU_1);
  const auto& imu_1_cal = jet_model.imu_calibration_from_imu_id.at(IMU_1);
  const auto& imu_2_cal = jet_model.imu_calibration_from_imu_id.at(IMU_2);

  const auto& imu_1_model = imu_1_cal.imu_model;
  const auto accel_interp = estimation::make_accel_interpolator(imu_1_meas.accel_meas, imu_1_model);

  const auto t0 = cal_meas.first();
  // const auto t0 = imu_1_meas.accel_meas.front().timestamp;

  auto xp0 = ejf::JetFilter::reasonable_initial_state(t0);
  {  //
    xp0.x.R_world_from_body = compute_world_from_camera(jet_model.imu_calibration_from_imu_id.at(IMU_2)).so3();
  }
  const auto p = compute_parameters(cal_meas, jet_model);
  ejf::JetFilter jf(xp0, p);

  for (const auto& fiducial_meas : cal_meas.fiducial_meas) {
    jf.measure_fiducial(fiducial_meas.measurement, fiducial_meas.timestamp);
  }

  for (const auto& accel : cal_meas.imu_cal.at(IMU_1).accel_meas) {
    const jcc::Vec3 corrected_accel = imu_1_model.correct_measured_accel(accel.measurement.observed_acceleration);
    // jf.measure_imu({corrected_accel}, accel.timestamp);
  }

  for (const auto& gyro : cal_meas.imu_cal.at(IMU_1).gyro_meas) {
    jf.measure_gyro(gyro.measurement, gyro.timestamp);
  }

  const auto& imu_2_model = imu_2_cal.imu_model;
  for (const auto& accel : cal_meas.imu_cal.at(IMU_2).accel_meas) {
    const jcc::Vec3 corrected_accel = imu_2_model.correct_measured_accel(accel.measurement.observed_acceleration);
    // jf.measure_imu({corrected_accel}, accel.timestamp, true);
  }

  for (const auto& gyro : cal_meas.imu_cal.at(IMU_2).gyro_meas) {
    // jf.measure_gyro(gyro.measurement, gyro.timestamp, true);
  }

  int k = 0;
  while (jf.next_measurement()) {
    ++k;
    if (k % 2 != 0) {
      continue;
    }
    const auto state = jf.state().x;
    // geo->add_axes({get_world_from_body(state)});

    const SE3 T_world_from_body = get_world_from_body(state);
    constexpr bool PRINT_STATES = false;
    if (PRINT_STATES) {
      std::cout << "States:       " << std::endl;
      std::cout << "\taccel_bias: " << state.accel_bias.transpose() << std::endl;
      std::cout << "\tgyro_bias: " << state.gyro_bias.transpose() << std::endl;
      std::cout << "\teps_ddot:   " << state.eps_ddot.transpose() << std::endl;
      std::cout << "\teps_dot:    " << state.eps_dot.transpose() << std::endl;
      std::cout << "\tr:          " << T_world_from_body.so3().log().transpose() << std::endl;
      std::cout << "\tx:          " << T_world_from_body.translation().transpose() << std::endl;
    }

    // Fix jitter: Covariances plz bby
    geo->add_axes({SE3(T_world_from_body.so3(), jet_origin)});

    const jcc::Vec3 accel_imu_frame = *accel_interp(jf.state().time_of_validity);

    const SE3 imu_from_vehicle = jf.parameters().T_imu1_from_vehicle;
    const SO3 world_from_imu = state.R_world_from_body * imu_from_vehicle.so3().inverse();

    geo->add_line(
        {jet_origin, jet_origin + (world_from_imu * accel_imu_frame) / 9.81, jcc::Vec4(0.2, 0.2, 1.0, 1.0), 5.0});

    geo->add_line({jcc::Vec3::Zero(), accel_imu_frame, jcc::Vec4(0.2, 0.2, 1.0, 1.0), 5.0});
    // Velocity
    geo->add_line({jcc::Vec3::Zero(), state.eps_dot.head<3>(), jcc::Vec4(1.0, 0.0, 0.0, 1.0), 5.0});
    // Angular Velocity
    geo->add_line({jcc::Vec3::Zero(), -state.eps_dot.tail<3>(), jcc::Vec4(0.0, 1.0, 0.0, 1.0), 5.0});

    jet_tree->set_world_from_root(SE3(T_world_from_body.so3() * jetmodel_from_body.inverse(), jet_origin));

    geo->flip();
    view->spin_until_step();
  }
}

void go() {
  jcc::Success() << "Preparing to calibrate" << std::endl;

  //
  // Grab the calibration data from the log
  //

  // const std::string path = "/jet/logs/calibration-log-jul4-2/";

  // const std::string path = "/jet/logs/calibration-log-jul6-6";
  const std::string path = "/jet/logs/calibration-log-jul6-7";

  const TimeRange range{};
  const ExtractionConfiguration default_cfg{};
  const auto cal_measurements = extract_data_from_log(path, range, default_cfg);

  estimation::warn_sensor_rates(cal_measurements);

  const double total_time = estimation::to_seconds(cal_measurements.last() - cal_measurements.first());
  jcc::Success() << "Done reading log." << std::endl;

  if (total_time < 30.0) {
    jcc::Warning() << "Total time (" << total_time << " seconds) less than 30.0 sec" << std::endl;
  } else {
    jcc::Success() << "Total time: " << total_time << " seconds" << std::endl;
  }

  //
  // Set up the viewer
  //
  setup();

  jcc::Success() << "Calibrating..." << std::endl;

  //
  // Calibrate the IMU intrinsics / Estrinsics
  //

  JetModel jet_model;
  for (const auto& imu_measurements : cal_measurements.imu_cal) {
    const auto one_imu = estimation::create_single_imu_model(cal_measurements, imu_measurements.second, imu_cal_cfg);
    jet_model.imu_calibration_from_imu_id[imu_measurements.first] = one_imu;
  }

  //
  // Validate the fiducial reprojections
  //

  jcc::Success() << "[Camera] Validating fiducial measurements" << std::endl;
  ImageStream image_stream(path, range);
  jcc::Success() << "[Camera] Parsing images..." << std::endl;

  const CameraManager cam_mgr;
  const auto view = viewer::get_window3d("Calibration");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  const auto ui2d = view->add_primitive<viewer::Ui2d>();

  bool set = false;
  geometry::Unit3 permanent_g_fiducial_frame;

  std::vector<estimation::calibration::TimedMeasurement<ejf::FiducialMeasurement>> collected_fiducial_meas;

  int i = 0;
  while (true) {
    const auto image = image_stream.next();

    ++i;
    if (!image) {
      break;
    }

    const auto calibration = cam_mgr.get_calibration(image->serial_number);
    const auto proj = proj_coeffs_from_opencv(calibration);
    const auto model = estimation::NonlinearCameraModel(proj);

    const auto ids_corners = get_ids_and_corners(camera_frame);
    const auto obj_pt_associations = obj_points_img_points_from_image(ids_corners);
    const auto fiducial_from_camera = estimate_board_bottom_left_from_camera(ids_corners, calibration);

    const auto t = image->time;
    if (fiducial_from_camera) {
      ejf::FiducialMeasurement fiducial_measurement;
      fiducial_measurement.T_fiducial_from_camera = *fiducial_from_camera;
      collected_fiducial_meas.push_back({fiducial_measurement, t});
    }

    // TODO: NEXT, FILL THIS IN
    if (camera_cal_cfg.visualize_camera) {
      const auto associations = object_image_assoc_from_board_point(obj_pt_associations);
      estimation::visualize_single_camera_frame(
          model, fiducial_from_camera, associations, *image, ui2d, geo, camera_cal_cfg);
    }
  }

  jcc::Success() << "[Filter] Testing filter..." << std::endl;
  {
    estimation::calibration::CalibrationMeasurements new_cal_measurements = cal_measurements;
    new_cal_measurements.fiducial_meas = collected_fiducial_meas;
    test_filter(new_cal_measurements, jet_model);
  }
  jcc::Success() << "Done." << std::endl;
}

}  // namespace filtering
}  // namespace jet

int main() {
  jet::filtering::go();
}