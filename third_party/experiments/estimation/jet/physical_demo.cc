#include "util/environment.hh"
#include "viewer/primitives/image.hh"
#include "viewer/primitives/scene_tree.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include <opencv2/opencv.hpp>
#include "estimation/vision/fiducial_pose.hh"

#include "estimation/jet/jet_filter.hh"
#include "estimation/jet/jet_optimizer.hh"
#include "estimation/time_point.hh"

// TODO: Factor
#include "numerics/set_diag_to_value.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace estimation {
namespace jet_filter {
namespace {

// TODO FACTOR
void draw_states(viewer::SimpleGeometry& geo,
                 const std::vector<State>& states,
                 bool truth) {
  const int n_states = static_cast<int>(states.size());
  for (int k = 0; k < n_states; ++k) {
    auto& state = states.at(k);
    const SE3 T_world_from_body = state.T_body_from_world.inverse();
    if (truth) {
      geo.add_axes({T_world_from_body, 0.1});
    } else {
      geo.add_axes({T_world_from_body, 0.05, 2.0, true});

      if (k < n_states - 1) {
        const auto& next_state = states.at(k + 1);
        const SE3 T_world_from_body_next = next_state.T_body_from_world.inverse();
        geo.add_line(
            {T_world_from_body.translation(), T_world_from_body_next.translation()});
      }
    }
  }
}

struct ProjectionModel {
  double fx;
  double fy;
  double cx;
  double cy;

  double p1;
  double p2;

  double k1;
  double k2;
  double k3;
};

jcc::Vec2 distort(const ProjectionModel& proj, const jcc::Vec3& world_point) {
  const jcc::Vec2 distorted_point = world_point.head<2>() / world_point.z();

  const double r2 = distorted_point.squaredNorm();
  const double x = distorted_point.x();
  const double y = distorted_point.y();
  const double xy = x * y;

  const double x_tan_offset = (2.0 * proj.p1 * xy) + (proj.p2 * (r2 + (2.0 * x * x)));
  const double y_tan_offset = (2.0 * proj.p2 * xy) + (proj.p1 * (r2 + (2.0 * y * y)));

  const double r4 = r2 * r2;
  const double r6 = r2 * r4;

  const double radial_distortion = (proj.k1 * r2) + (proj.k2 * r4) + (proj.k3 * r6);
  const double x_prime = x * (1.0 + radial_distortion) + x_tan_offset;
  const double y_prime = y * (1.0 + radial_distortion) + y_tan_offset;
  return jcc::Vec2(x_prime, y_prime);
}

void setup() {
  const auto view = viewer::get_window3d("FiducialDebug");

  view->set_azimuth(0.0);
  view->set_elevation(0.0);
  view->set_zoom(1.0);

  // view->set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)),
  // jcc::Vec3(-1.0, 0.0, -1.0)));
  view->set_continue_time_ms(10);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};

  background->add_plane({ground, 0.1});
  background->flip();
}

void setup2() {
  const auto view = viewer::get_window3d("ImageView");

  // 1920, 1080

  view->set_azimuth(0.0);
  view->set_elevation(0.0);
  view->set_zoom(1.0);

  // view->set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)),
  // jcc::Vec3(-1.0, 0.0, -1.0)));
  view->set_continue_time_ms(10);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};

  background->add_plane({ground, 0.1});
  background->flip();
}

}  // namespace

void run() {
  setup();
  setup2();

  const auto view = viewer::get_window3d("FiducialDebug");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  const std::string fiducial_path = jcc::Environment::asset_path() + "fiducial.jpg";
  const cv::Mat fiducial_tag = cv::imread(fiducial_path);
  const auto fiducial_image = view->add_primitive<viewer::Image>(fiducial_tag, 0.28, 1);

  const auto im_view = viewer::get_window3d("ImageView");
  cv::Mat camera_frame = cv::Mat::zeros(cv::Size(640, 640), CV_8UC3);
  const auto image = im_view->add_primitive<viewer::Image>(camera_frame);

  auto cap = cv::VideoCapture(0);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('B', 'G', 'R', '8'));
  cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
  cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
  constexpr double LETS_USE_THIS_EXPOSURE = 0.1;
  cap.set(cv::CAP_PROP_EXPOSURE, LETS_USE_THIS_EXPOSURE);

  //
  // Set up filters & optimizers
  //

  FilterState<State> xp0;
  {
    MatNd<State::DIM, State::DIM> state_cov;
    state_cov.setZero();
    numerics::set_diag_to_value<StateDelta::accel_bias_error_dim,
                                StateDelta::accel_bias_error_ind>(state_cov, 0.0001);
    numerics::set_diag_to_value<StateDelta::gyro_bias_error_dim,
                                StateDelta::gyro_bias_error_ind>(state_cov, 0.0001);
    numerics::set_diag_to_value<StateDelta::eps_dot_error_dim,
                                StateDelta::eps_dot_error_ind>(state_cov, 0.01);
    numerics::set_diag_to_value<StateDelta::eps_ddot_error_dim,
                                StateDelta::eps_ddot_error_ind>(state_cov, 0.1);

    xp0.P = state_cov;
    xp0.time_of_validity = jcc::now();
  }

  JetFilter jf(xp0);
  JetOptimizer jet_opt;
  std::vector<State> est_states;

  const auto obs_geo = view->add_primitive<viewer::SimpleGeometry>();
  constexpr double sphere_size_m = 0.05;

  while (!im_view->should_close()) {
    const TimePoint current_time = jcc::now();
    cap.set(cv::CAP_PROP_EXPOSURE, LETS_USE_THIS_EXPOSURE);
    if (cap.read(camera_frame)) {
      cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);

      cv::Mat cf_blurred;
      // blur = cv2.blur(img,(5,5))
      cv::GaussianBlur(camera_frame, cf_blurred, cv::Size(5, 5), 1.0, 1.0);

      const auto result = vision::detect_markers(cf_blurred);

      for (const auto& detection : result) {
        const SE3 world_from_camera = detection.marker_center_from_camera;
        geo->add_axes({world_from_camera});

        for (const auto& pt : detection.image_points) {
          cv::circle(camera_frame, cv::Point2f(pt.x(), pt.y()), 5.0,
                     cv::Scalar(255, 15, 15), 5.0);
        }
      }

      if (!result.empty()) {
        const FiducialMeasurement fiducial_meas = {
            -1, result.at(0u).marker_center_from_camera};
        jf.measure_fiducial(fiducial_meas, current_time);
        jet_opt.measure_fiducial(fiducial_meas, current_time);

        jf.free_run();
        est_states.push_back(jf.state().x);
        assert(jf.state().time_of_validity == current_time);

        const SE3 world_from_jet = jf.state().x.T_body_from_world.inverse();
        obs_geo->add_sphere({world_from_jet.translation(), sphere_size_m,
                             jcc::Vec4(0.0, 1.0, 0.0, 1.0), world_from_jet.so3()});
      } else {
        const SE3 world_from_jet = jf.view(current_time).T_body_from_world.inverse();
        obs_geo->add_sphere({world_from_jet.translation(), sphere_size_m,
                             jcc::Vec4(1.0, 0.6, 0.0, 1.0), world_from_jet.so3()});
      }

      geo->flip();
      obs_geo->flip();

      image->update_image(camera_frame);
    }
  }

  std::cout << "Done, optimizing" << std::endl;
  {
    const auto visitor_geo = view->add_primitive<viewer::SimpleGeometry>();
    const auto visitor = [&view, &visitor_geo](const JetPoseOptimizer::Solution& soln) {
      visitor_geo->clear();
      draw_states(*visitor_geo, soln.x, false);
      visitor_geo->flip();
      // std::cout << "\tOptimized g: " << soln.p.g_world.transpose() << std::endl;
      std::cout << "\tOptimized T_imu_from_vehicle: "
                << soln.p.T_imu_from_vehicle.translation().transpose() << "; "
                << soln.p.T_imu_from_vehicle.so3().log().transpose() << std::endl;
      view->spin_until_step();
    };

    const auto solution = jet_opt.solve(est_states, jf.parameters(), visitor);
  }

  view->spin_until_step();
}

}  // namespace jet_filter
}  // namespace estimation

int main() {
  estimation::jet_filter::run();
}