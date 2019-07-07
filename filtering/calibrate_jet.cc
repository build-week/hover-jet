//%deps(jet_filter)
//%deps(jet_optimizer)
//%deps(nonlinear_camera_model)
//%deps(warn_sensor_rates)
//%deps(make_interpolator)
#include "third_party/experiments/estimation/calibration/nonlinear_camera_model.hh"
#include "third_party/experiments/estimation/calibration/warn_sensor_rates.hh"
#include "third_party/experiments/estimation/jet/jet_filter.hh"
#include "third_party/experiments/estimation/jet/jet_optimizer.hh"
#include "third_party/experiments/estimation/sensors/make_interpolator.hh"

//%deps(create_static_jet_model)
//%deps(robust_pnp)
//%deps(visualize_calibration)
//%deps(visualize_camera_calibration)
#include "third_party/experiments/estimation/jet/create_static_jet_model.hh"
#include "third_party/experiments/estimation/vision/robust_pnp.hh"
#include "third_party/experiments/estimation/visualization/visualize_calibration.hh"
#include "third_party/experiments/estimation/visualization/visualize_camera_calibration.hh"

//%deps(form_coordinate_frame)
//%deps(put_transform_network)
#include "third_party/experiments/geometry/spatial/form_coordinate_frame.hh"
#include "third_party/experiments/geometry/visualization/put_transform_network.hh"

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

#include "filtering/transform_network_from_yaml.hh"
#include "filtering/yaml_matrix.hh"

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

bool visualize_filter = true;

bool visualize() {
  return (camera_cal_cfg.visualize_camera || camera_cal_cfg.visualize_camera_distortion ||
          camera_cal_cfg.visualize_camera_frustum || imu_cal_cfg.visualize_imu_model || imu_cal_cfg.visualize_gyro ||
          imu_cal_cfg.visualize_magnetometer || visualize_filter);
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

void emit_yaml_for_imu_calibrations(const estimation::jet::JetModel& jet_model, const std::string& source_log_name) {
  YAML::Node node;
  YAML::Node imus_node = node["imus"];

  for (const auto& imu_pair : jet_model.imu_calibration_from_imu_id) {
    const auto& single_cal = imu_pair.second;

    const std::string imu_id_str = estimation::jet::make_imu_id_string(single_cal.imu_id);
    {
      YAML::Node single_imu_node = imus_node[single_cal.imu_id];
      const auto intrinsics = single_cal.imu_model.intrinsics();

      set_matrix(single_imu_node, "accelerometer_gains_scaling", intrinsics.imu_gains.cholesky_factor);
      set_matrix(single_imu_node, "accelerometer_gains_p0", intrinsics.imu_gains.p0);
      set_matrix(single_imu_node, "magnetometer_gains_scaling", intrinsics.magnetometer_gains.cholesky_factor);
      set_matrix(single_imu_node, "magnetometer_gains_p0", intrinsics.magnetometer_gains.p0);

      single_imu_node["id"] = single_cal.imu_id;
      single_imu_node["name"] = imu_id_str;
      single_imu_node["source_log"] = source_log_name;
    }
  }

  transform_network_to_yaml(node, jet_model.transform_network);

  std::cout << node << std::endl;
}

jcc::Optional<ejf::FiducialMeasurement> find_nearest_fiducial_in_time(
    const std::vector<estimation::TimedMeasurement<ejf::FiducialMeasurement>> fiducial_measurements,
    const estimation::TimePoint& t) {
  JASSERT_FALSE(fiducial_measurements.empty(), "Cannot calibrate with empty fiducial measurements");
  for (const auto& fiducial_measurements : fiducial_measurements) {
    if (t < fiducial_measurements.timestamp) {
      return {fiducial_measurements.measurement};
    }
  }
  return {};
}

SE3 compute_world_from_camera(const estimation::SingleImuCalibration& imu_cal) {
  const geometry::Unit3 g_camera_frame = imu_cal.camera_from_gyro * imu_cal.g_estimate.direction;
  const SO3 world_from_camera = geometry::spatial::form_coordinate_frame_from_zhat(g_camera_frame);
  return SE3(world_from_camera, jcc::Vec3::Zero());
}

ejf::Parameters compute_filter_fixed_parameters(
    const std::vector<estimation::TimedMeasurement<ejf::FiducialMeasurement>> fiducial_measurements,
    const estimation::jet::JetModel& jet_model) {
  const auto& imu_cal_1 = jet_model.imu_calibration_from_imu_id.at(IMU_1);
  const auto& imu_cal_2 = jet_model.imu_calibration_from_imu_id.at(IMU_2);

  const SO3 world_from_camera_1 = compute_world_from_camera(imu_cal_1).so3();
  const SO3 world_from_camera_2 = compute_world_from_camera(imu_cal_2).so3();
  jcc::Warning() << "[Filter Setup] IMU->IMU Alignment Error: "
                 << (world_from_camera_1 * world_from_camera_2.inverse()).log().norm() << " Radians" << std::endl;

  const auto maybe_nearest_fiducial = find_nearest_fiducial_in_time(fiducial_measurements, imu_cal_1.g_estimate.time);
  assert(maybe_nearest_fiducial);
  const auto fiducial_at_g_estimate = *maybe_nearest_fiducial;
  const SO3 fiducial_from_camera = fiducial_at_g_estimate.T_fiducial_from_camera.so3();
  const SO3 world_from_fiducial = world_from_camera_1 * fiducial_from_camera.inverse();

  auto p = ejf::JetFilter::reasonable_parameters();
  {
    p.T_world_from_fiducial = SE3(world_from_fiducial, jcc::Vec3::Zero());

    p.T_imu1_from_vehicle = jet_model.transform_network.find_source_from_destination("imu_78", "vehicle");
    p.T_imu2_from_vehicle = jet_model.transform_network.find_source_from_destination("imu_36", "vehicle");
    p.T_camera_from_vehicle = jet_model.transform_network.find_source_from_destination("camera", "vehicle");
  }

  return p;
}

void test_filter(const estimation::CalibrationMeasurements& cal_meas, const estimation::jet::JetModel& jet_model) {
  const auto view = viewer::get_window3d("Calibration");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  const auto geo_stable = view->add_primitive<viewer::SimpleGeometry>();
  const auto jet_tree = view->add_primitive<viewer::SceneTree>();

  const jcc::Vec3 jet_origin(1.0, 0.0, 0.0);

  const planning::jet::JetModel jet_3dmodel;
  constexpr bool DRAW_VEHICLE = true;
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

  const auto p = compute_filter_fixed_parameters(cal_meas.fiducial_meas, jet_model);

  auto xp0 = ejf::JetFilter::reasonable_initial_state(t0);
  {
    const SE3 world_from_vehicle =
        compute_world_from_camera(jet_model.imu_calibration_from_imu_id.at(IMU_2)) * p.T_camera_from_vehicle;
    xp0.x.R_world_from_body = world_from_vehicle.so3();
    xp0.x.x_world = world_from_vehicle.translation();
  }

  ejf::JetFilter jf(xp0, p);

  for (const auto& fiducial_meas : cal_meas.fiducial_meas) {
    jf.measure_fiducial(fiducial_meas.measurement, fiducial_meas.timestamp);
  }

  constexpr bool USE_ACCELEROMETER = false;
  if (USE_ACCELEROMETER) {
    for (const auto& accel : cal_meas.imu_cal.at(IMU_1).accel_meas) {
      const jcc::Vec3 corrected_accel = imu_1_model.correct_measured_accel(accel.measurement.observed_acceleration);
      jf.measure_imu({corrected_accel}, accel.timestamp);
    }
  }

  for (const auto& gyro : cal_meas.imu_cal.at(IMU_1).gyro_meas) {
    jf.measure_gyro(gyro.measurement, gyro.timestamp);
  }

  constexpr bool USE_IMU_2 = false;
  if (USE_IMU_2) {
    const auto& imu_2_model = imu_2_cal.imu_model;
    for (const auto& accel : cal_meas.imu_cal.at(IMU_2).accel_meas) {
      const jcc::Vec3 corrected_accel = imu_2_model.correct_measured_accel(accel.measurement.observed_acceleration);
      jf.measure_imu({corrected_accel}, accel.timestamp, true);
    }

    for (const auto& gyro : cal_meas.imu_cal.at(IMU_2).gyro_meas) {
      jf.measure_gyro(gyro.measurement, gyro.timestamp, true);
    }
  }

  int k = 0;
  while (jf.next_measurement()) {
    ++k;
    if (k % 2 != 0) {
      continue;
    }
    const auto state = jf.state().x;

    const SE3 T_world_from_body = get_world_from_body(state);
    constexpr bool PRINT_STATES = false;
    if (PRINT_STATES) {
      std::cout << "States:       " << std::endl;
      std::cout << "\taccel_bias: " << state.accel_bias.transpose() << std::endl;
      std::cout << "\tgyro_bias:  " << state.gyro_bias.transpose() << std::endl;
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

    auto tfn2 = jet_model.transform_network;
    const auto maybe_nearest_fiducial =
        find_nearest_fiducial_in_time(cal_meas.fiducial_meas, jf.state().time_of_validity);

    // const SE3 visualized_world_from_vehicle = SE3(T_world_from_body.so3(), jet_origin);
    const SE3 visualized_world_from_vehicle = T_world_from_body;

    tfn2.update_edge("world", "vehicle", visualized_world_from_vehicle);
    if (maybe_nearest_fiducial) {
      tfn2.update_edge("fiducial", "camera", maybe_nearest_fiducial->T_fiducial_from_camera);
    }

    geometry::put_transform_network(*geo, tfn2, "world");
    const SE3 vehicle_from_model = jcc::exp_z(-M_PI * 0.5);
    jet_tree->set_world_from_root(visualized_world_from_vehicle * vehicle_from_model);

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
  // Validate the fiducial reprojections
  //

  jcc::Success() << "[Camera] Validating fiducial measurements" << std::endl;
  ImageStream image_stream(path, range);
  jcc::Success() << "[Camera] Re-running Fiducial Detection on images..." << std::endl;

  const CameraManager cam_mgr;
  const auto view = viewer::get_window3d("Calibration");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  const auto ui2d = view->add_primitive<viewer::Ui2d>();

  // Compute new fiducial measurements
  std::vector<estimation::TimedMeasurement<ejf::FiducialMeasurement>> collected_fiducial_meas;

  int i = 0;
  while (true) {
    const auto image = image_stream.next();

    ++i;
    if (!image) {
      break;
    }
    const auto calibration = cam_mgr.get_calibration(image->serial_number);
    const auto ids_corners = get_ids_and_corners(image->image);
    const auto fiducial_from_camera = estimate_board_bottom_left_from_camera(ids_corners, calibration);

    const auto t = image->time;
    if (fiducial_from_camera) {
      ejf::FiducialMeasurement fiducial_measurement;
      fiducial_measurement.T_fiducial_from_camera = *fiducial_from_camera;
      collected_fiducial_meas.push_back({fiducial_measurement, t});
    }

    // TODO: NEXT, FILL THIS IN
    if (camera_cal_cfg.visualize_camera) {
      const auto obj_pt_associations = obj_points_img_points_from_image(ids_corners);
      const auto proj = proj_coeffs_from_opencv(calibration);
      const auto model = estimation::NonlinearCameraModel(proj);
      const auto associations = object_image_assoc_from_board_point(obj_pt_associations);
      estimation::visualize_single_camera_frame(
          model, fiducial_from_camera, associations, *image, ui2d, geo, camera_cal_cfg);
    }
  }

  jcc::Success() << "[Filter] Testing filter..." << std::endl;
  estimation::CalibrationMeasurements new_cal_measurements = cal_measurements;
  { new_cal_measurements.fiducial_meas = collected_fiducial_meas; }

  const auto jet_model = estimation::jet::create_static_jet_model(new_cal_measurements, imu_cal_cfg);

  //
  // Calibrate the IMU intrinsics / Estrinsics
  //

  emit_yaml_for_imu_calibrations(jet_model, path);

  test_filter(new_cal_measurements, jet_model);

  jcc::Success() << "Done." << std::endl;
}

}  // namespace filtering
}  // namespace jet

int main() {
  jet::filtering::go();
}