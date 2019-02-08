#include <queue>

//%deps(opencv)
#include <opencv2/opencv.hpp>

#include "camera/camera_image_message.hh"
#include "embedded/imu_driver/imu_message.hh"
#include "infrastructure/logging/log_reader.hh"

//%deps(fiducial_pose)
//%deps(rotation_to)
#include "third_party/experiments/estimation/vision/fiducial_pose.hh"
#include "third_party/experiments/geometry/rotation_to.hh"

//%deps(jet_filter)
//%deps(simple_geometry)
//%deps(jet_optimizer)
//%deps(window_3d)
#include "third_party/experiments/estimation/jet/jet_filter.hh"
#include "third_party/experiments/estimation/jet/jet_optimizer.hh"
#include "third_party/experiments/viewer/primitives/simple_geometry.hh"
#include "third_party/experiments/viewer/window_3d.hh"

//%deps(interpolator)
#include "third_party/experiments/geometry/spatial/interpolator.hh"

//%deps(fit_ellipse)
#include "third_party/experiments/geometry/shapes/fit_ellipse.hh"

//
#include "vision/fiducial_detection_and_pose.hh"
#include "vision/fiducial_detection_message.hh"

namespace jet {
namespace filtering {
namespace {
// TODO FACTOR
void draw_states(viewer::SimpleGeometry& geo, const std::vector<estimation::jet_filter::State>& states, bool truth) {
  const int n_states = static_cast<int>(states.size());
  for (int k = 0; k < n_states; ++k) {
    auto& state = states.at(k);
    const SE3 T_world_from_body = state.T_body_from_world.inverse();
    if (truth) {
      geo.add_axes({T_world_from_body, 0.1});
    } else {
      geo.add_axes({T_world_from_body, 0.01, 1.0, true});
      if (k < n_states - 1) {
        const auto& next_state = states.at(k + 1);
        const SE3 T_world_from_body_next = next_state.T_body_from_world.inverse();
        geo.add_line({T_world_from_body.translation(), T_world_from_body_next.translation()});
      }
    }
  }
}

void setup() {
  const auto view = viewer::get_window3d("Filter Debug");
  // view->set_azimuth(0.0);
  // view->set_elevation(0.0);
  // view->set_zoom(1.0);
  view->set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), jcc::Vec3(-1.0, 0.0, -1.0)));
  view->set_continue_time_ms(10);
  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground, 0.1});
  background->flip();
}
estimation::TimePoint to_time_point(const Timestamp& ts) {
  const auto epoch_offset = std::chrono::nanoseconds(uint64_t(ts));
  const estimation::TimePoint time_point = estimation::TimePoint{} + epoch_offset;
  return time_point;
}

class Calibrator {
 public:
  Calibrator() {
    const auto view = viewer::get_window3d("Filter Debug");
    geo_ = view->add_primitive<viewer::SimpleGeometry>();
  }

  void maybe_add_imu(const ImuMessage& msg) {
    const auto time_of_validity = to_time_point(msg.timestamp);
    if (estimation::to_seconds(time_of_validity - earliest_camera_time_) > 25.0) {
      return;
    }
    // if (timestamp > earliest_camera_time_) {
    if (true) {
      // std::cout << "Accel:"  << uint64_t(msg.timestamp) << std::endl;
      estimation::jet_filter::AccelMeasurement accel_meas;
      const jcc::Vec3 accel_mpss(msg.accel_mpss_x, msg.accel_mpss_y, msg.accel_mpss_z);
      last_accel_ = accel_mpss;
      got_imu_ = true;

      accel_meas.observed_acceleration = accel_mpss;
      jf_.measure_imu(accel_meas, time_of_validity);
      jet_opt_.measure_imu(accel_meas, time_of_validity);

      accel_meas_.push_back({accel_meas, time_of_validity});

      // const jcc::Vec3 gyro_radps(msg.gyro_radps_x, msg.gyro_radps_y, msg.gyro_radps_z);
      // estimation::jet_filter::GyroMeasurement gyro_meas;
      // gyro_meas.observed_w = gyro_radps;
      // jf_.measure_gyro(gyro_meas, time_of_validity + estimation::to_duration(0.0001));
      // jet_opt_.measure_gyro(gyro_meas, time_of_validity + estimation::to_duration(0.0001));

      const jcc::Vec3 mag_utesla(msg.mag_utesla_x, msg.mag_utesla_y, msg.mag_utesla_z);
      mag_utesla_.push_back({mag_utesla, time_of_validity});
      geo_->add_point({(mag_utesla)});
      geo_->flush();
    }
    // Otherwise, ignore it
  }

  void add_fiducial(const Timestamp& ts, const SE3& world_from_camera) {
    const auto time_of_validity = to_time_point(ts);
    if (!got_camera_) {
      auto xp0 = estimation::jet_filter::JetFilter::reasonable_initial_state();

      xp0.x.T_body_from_world = world_from_camera.inverse();
      xp0.time_of_validity = time_of_validity;
      jf_.reset(xp0);

      earliest_camera_time_ = time_of_validity;
    }
    // std::cout << "cam: " << uint64_t(ts) << std::endl;
    if (estimation::to_seconds(time_of_validity - earliest_camera_time_) > 25.0) {
      return;
    }

    got_camera_ = true;

    estimation::jet_filter::FiducialMeasurement fiducial_meas;
    fiducial_meas.T_fiducial_from_camera = world_from_camera;

    jf_.measure_fiducial(fiducial_meas, time_of_validity);
    jet_opt_.measure_fiducial(fiducial_meas, time_of_validity);
    fiducial_meas_.push_back({fiducial_meas, time_of_validity});

    geo_->add_axes({world_from_camera, 0.025, 3.0});
    if (got_imu_) {
      got_imu_ = false;

      const SE3 imu_from_vehicle = jf_.parameters().T_imu_from_vehicle;

      const SE3 T_camera_from_body = SE3();
      const SE3 world_from_body = world_from_camera * T_camera_from_body;

      const SE3 world_from_imu = world_from_body * imu_from_vehicle.inverse();
      const jcc::Vec3 accel_world = world_from_imu * last_accel_;
      // geo_->add_line({world_from_imu.translation(), accel_world});
      // geo_->add_line({jcc::Vec3::Zero(), world_from_imu.so3() * last_accel_});
    }
    const auto view = viewer::get_window3d("Filter Debug");

    geo_->flush();
  }

  geometry::spatial::LinearInterpolator fit_mag(const estimation::TimePoint& start_t) const {
    std::vector<jcc::Vec3> measurements;
    const auto view = viewer::get_window3d("Filter Debug");
    for (const auto& pt : mag_utesla_) {
      measurements.push_back(pt.first);
    }

    const auto ell_geo = view->add_primitive<viewer::SimpleGeometry>();
    const auto visitor = [&ell_geo, &view](const geometry::shapes::EllipseFit& fit) {
      ell_geo->add_ellipsoid({fit.ellipse, jcc::Vec4(0.4, 0.6, 0.4, 0.7), 2.0});
      ell_geo->flip();
      view->spin_until_step();
    };
    const auto result = geometry::shapes::fit_ellipse(measurements, visitor);

    ell_geo->add_ellipsoid({result.ellipse, jcc::Vec4(0.2, 1.0, 0.2, 1.0), 4.0});
    ell_geo->flip();
    view->spin_until_step();

    std::vector<geometry::spatial::ControlPoint> control_points;
    for (const auto& pt : mag_utesla_) {
      const auto pt_time = pt.second;
      const double delta_t = estimation::to_seconds(pt_time - start_t);

      const jcc::Vec3 pt_mag_utesla = pt.first;

      const jcc::Vec3 pt_mag_corrected =
          (result.ellipse.cholesky_factor.transpose().inverse() * (pt_mag_utesla - result.ellipse.p0));

      control_points.push_back({delta_t, pt_mag_corrected});
    }

    const geometry::spatial::LinearInterpolator interpolator(control_points);
    return interpolator;
  }

  geometry::spatial::LinearInterpolator make_accel_interpolator() const {
    std::vector<geometry::spatial::ControlPoint> points;
    const auto first_time = accel_meas_.front().second;
    for (const auto& measurement : accel_meas_) {
      const double delta_t = estimation::to_seconds(measurement.second - first_time);
      points.push_back({delta_t, measurement.first.observed_acceleration});
    }
    const geometry::spatial::LinearInterpolator interp(sort_control_points(points));
    return interp;
  }

  void prepare() {
    const auto view = viewer::get_window3d("Filter Debug");
    const auto first_time = accel_meas_.front().second;

    const auto mag_interpolator = fit_mag(first_time);
    const auto accel_interpolator = make_accel_interpolator();

    const SE3 imu_from_vehicle = jf_.parameters().T_imu_from_vehicle;

    for (const auto& fiducial_meas : fiducial_meas_) {
      const SE3 world_from_vehicle = fiducial_meas.first.T_fiducial_from_camera;
      const SE3 imu_from_world = imu_from_vehicle * world_from_vehicle.inverse();

      // geo_->add_axes({imu_from_world.inverse(), 0.01});

      const double t = estimation::to_seconds(fiducial_meas.second - first_time);

      const auto maybe_interp_at_t = accel_interpolator(t);
      if (!maybe_interp_at_t) {
        std::cout << "Failed at " << t << std::endl;
        continue;
      }
      const jcc::Vec3 accel_meas_imu_at_t = *maybe_interp_at_t;
      const jcc::Vec3 accel_meas_vehicle_frame = imu_from_vehicle.so3().inverse() * accel_meas_imu_at_t;

      const jcc::Vec3 mag_meas_vehicle_frame = imu_from_vehicle.so3().inverse() * (*mag_interpolator(t));

      constexpr double M_PER_MPSS = 0.01;
      const jcc::Vec4 imu_obs_color(0.1, 0.7, 0.7, 0.4);
      // geo_->add_line({world_from_vehicle.translation(), world_from_vehicle * (accel_meas_vehicle_frame * M_PER_MPSS),
      // imu_obs_color});

      const jcc::Vec3 g_vehicle_frame = (world_from_vehicle.so3().inverse() * (jcc::Vec3::UnitZ() * 9.81));
      const jcc::Vec3 accel_g_subtracted_vehicle = accel_meas_vehicle_frame - g_vehicle_frame;

      const jcc::Vec3 direction = world_from_vehicle.so3() * (accel_g_subtracted_vehicle).normalized();
      const double length = accel_g_subtracted_vehicle.norm() * M_PER_MPSS;
      geo_->add_ray({world_from_vehicle.translation(), direction, length, imu_obs_color, 3.0});

      const jcc::Vec4 mag_obs_color(1.0, 0.3, 0.3, 0.4);
      geo_->add_ray({world_from_vehicle.translation(), world_from_vehicle.so3() * mag_meas_vehicle_frame.normalized(), 1.0,
                     mag_obs_color, 3.0});

      view->spin_until_step();
      geo_->flush();
    }
  }

  void run() {
    prepare();

    std::vector<estimation::jet_filter::State> est_states;
    const auto view = viewer::get_window3d("Filter Debug");

    while (true) {
      const auto xx = jf_.next_measurement();
      if (!xx) {
        break;
      }
      est_states.push_back(*xx);
    }

    std::cout << "Calibrating" << std::endl;
    const auto visitor = make_visitor();
    const auto solution = jet_opt_.solve(est_states, jf_.parameters(), visitor);
  }

 private:
  estimation::jet_filter::JetPoseOptimizer::Visitor make_visitor() {
    const auto view = viewer::get_window3d("Filter Debug");
    const auto visitor_geo = view->add_primitive<viewer::SimpleGeometry>();
    const auto visitor = [view, visitor_geo](const estimation::jet_filter::JetPoseOptimizer::Solution& soln) {
      visitor_geo->clear();
      draw_states(*visitor_geo, soln.x, false);
      visitor_geo->flip();
      // std::cout << "\tOptimized g: " << soln.p.g_world.transpose() << std::endl;
      std::cout << "\tOptimized T_imu_from_vehicle: " << soln.p.T_imu_from_vehicle.translation().transpose() << "; "
                << soln.p.T_imu_from_vehicle.so3().log().transpose() << std::endl;
      view->spin_until_step();
    };
    return visitor;
  }

  bool got_imu_ = false;
  jcc::Vec3 last_accel_ = jcc::Vec3::Zero();

  estimation::TimePoint earliest_camera_time_ = estimation::TimePoint::max();
  bool got_camera_ = false;

  std::vector<std::pair<estimation::jet_filter::AccelMeasurement, estimation::TimePoint>> accel_meas_;
  std::vector<std::pair<estimation::jet_filter::FiducialMeasurement, estimation::TimePoint>> fiducial_meas_;
  // std::vector<std::Pair<GyroMeasurement, estimation::TimePoint>> gyro_meas_;

  std::vector<std::pair<jcc::Vec3, estimation::TimePoint>> mag_utesla_;

  estimation::jet_filter::JetFilter jf_;
  estimation::jet_filter::JetOptimizer jet_opt_;

  // temp
  std::shared_ptr<viewer::SimpleGeometry> geo_;
};

}  // namespace

void go() {
  setup();
  const auto view = viewer::get_window3d("Filter Debug");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  const std::vector<std::string> channel_names = {"imu", "fiducial_detection_channel", "camera_image_channel"};

  // const std::string path = "/jet/logs/calibration-log-jan26-1";
  const std::string path = "/jet/logs/calibration-log-jan31-1";

  Calibrator calibrator;
  jet::LogReader reader(path, channel_names);

  bool accepted_any = false;
  SE3 last_world_from_camera;
  // const uint64_t start_t = 1548989056740894609;
  // const uint64_t end_t = 1548989075101073423;

  const uint64_t start_t = 1548989056142134014;
  const uint64_t end_t = 1548989091635950250;

  constexpr bool USE_CAMERA_IMAGES = true;
  constexpr bool USE_FIDUCIAL_DETECTIONS = false;

  int imu_ct = 0;
  for (int k = 0; k < 3000; ++k) {
    {
      ImuMessage imu_msg;
      if (reader.read_next_message("imu", imu_msg)) {
        imu_ct++;
        if (imu_ct % 1 == 0) {
          const uint64_t ts = imu_msg.timestamp;
          if (((ts > start_t) && (ts < end_t))) {
            calibrator.maybe_add_imu(imu_msg);
          }
        }
      } else {
        std::cout << "Breaking at : " << k << std::endl;
        break;
      }
    }

    if (USE_CAMERA_IMAGES) {
      CameraImageMessage cam_msg;
      if (reader.read_next_message("camera_image_channel", cam_msg)) {
        const auto image = get_image_mat(cam_msg);

        const uint64_t ts = cam_msg.timestamp;
        if (!((ts > start_t) && (ts < end_t))) {
          continue;
        }

        const auto result = detect_board(image);
        if (result) {
          const SE3 world_from_camera = *result;

          if (accepted_any) {
            const SE3 camera_from_last_camera = world_from_camera.inverse() * last_world_from_camera;
            constexpr double MAX_OUTLIER_DIST_M = 0.7;
            if (camera_from_last_camera.translation().norm() > MAX_OUTLIER_DIST_M) {
              continue;
            }
          }

          accepted_any = true;
          last_world_from_camera = world_from_camera;

          calibrator.add_fiducial(cam_msg.timestamp, world_from_camera);
        }
      }
    }

    if (USE_FIDUCIAL_DETECTIONS) {
      FiducialDetectionMessage fiducial_msg;
      if (reader.read_next_message("fiducial_detection_channel", fiducial_msg)) {
        const SE3 world_from_camera = fiducial_msg.fiducial_from_camera();

        if (accepted_any) {
          const SE3 camera_from_last_camera = world_from_camera.inverse() * last_world_from_camera;
          constexpr double MAX_OUTLIER_DIST_M = 0.7;
          if (camera_from_last_camera.translation().norm() > MAX_OUTLIER_DIST_M) {
            continue;
          }
        }

        const uint64_t ts = fiducial_msg.timestamp;
        if (((ts > start_t) && (ts < end_t))) {
          calibrator.add_fiducial(fiducial_msg.timestamp, world_from_camera);
          accepted_any = true;
          last_world_from_camera = world_from_camera;
        }
      }
    }
    // cv::imshow("Image", image);
    // cv::waitKey(0);
  }

  geo->flush();
  view->spin_until_step();
  std::cout << "Done, preparing to calibrate" << std::endl;

  calibrator.run();
  view->spin_until_step();
}

}  // namespace filtering
}  // namespace jet

int main() {
  jet::filtering::go();
}
