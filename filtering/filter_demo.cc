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

//
#include "vision/fiducial_detection_and_pose.hh"

namespace jet {
namespace filtering {
namespace {
// TODO FACTOR
void draw_states(viewer::SimpleGeometry& geo,
                 const std::vector<estimation::jet_filter::State>& states,
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

void setup() {
  const auto view = viewer::get_window3d("Filter Debug");
  // view->set_azimuth(0.0);
  // view->set_elevation(0.0);
  // view->set_zoom(1.0);
  view->set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)),
                                  jcc::Vec3(-1.0, 0.0, -1.0)));
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
  Calibrator() = default;

  void maybe_add_imu(const ImuMessage& msg) {
    const auto timestamp = to_time_point(msg.timestamp);
    if (uint64_t(msg.timestamp) > 1547103182284210843) {
      return;
    }

    if (timestamp > earliest_camera_time_) {
      const jcc::Vec3 accel_mpss(msg.accel_mpss_x, msg.accel_mpss_y, msg.accel_mpss_z);

      estimation::jet_filter::AccelMeasurement accel_meas;
      accel_meas.observed_acceleration = accel_mpss;
      jf_.measure_imu(accel_meas, timestamp);
      jet_opt_.measure_imu(accel_meas, timestamp);
      const jcc::Vec3 gyro_radps(msg.gyro_radps_x, msg.gyro_radps_y, msg.gyro_radps_z);

      // estimation::jet_filter::GyroMeasurement gyro_meas;
      // gyro_meas.observed_w = gyro_radps;
      // jf_.measure_gyro(gyro_meas, timestamp + estimation::to_duration(0.0001));
      // jet_opt_.measure_gyro(gyro_meas, timestamp + estimation::to_duration(0.0001));
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
    got_camera_ = true;

    estimation::jet_filter::FiducialMeasurement fiducial_meas;
    fiducial_meas.T_fiducial_from_camera = world_from_camera;

    jf_.measure_fiducial(fiducial_meas,
                         time_of_validity - estimation::to_duration(0.0001));
    jet_opt_.measure_fiducial(fiducial_meas,
                              time_of_validity - estimation::to_duration(0.0001));
  }

  void run() {
    std::vector<estimation::jet_filter::State> est_states;
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
    const auto visitor =
        [&view,
         &visitor_geo](const estimation::jet_filter::JetPoseOptimizer::Solution& soln) {
          visitor_geo->clear();
          draw_states(*visitor_geo, soln.x, false);
          visitor_geo->flip();
          std::cout << "\tOptimized g: " << soln.p.g_world.transpose() << std::endl;
          std::cout << "\tOptimized T_imu_from_vehicle: "
                    << soln.p.T_imu_from_vehicle.translation().transpose() << "; "
                    << soln.p.T_imu_from_vehicle.so3().log().transpose() << std::endl;
          view->spin_until_step();
        };
    return visitor;
  }

  estimation::TimePoint earliest_camera_time_ = estimation::TimePoint::max();
  bool got_camera_ = false;

  estimation::jet_filter::JetFilter jf_;
  estimation::jet_filter::JetOptimizer jet_opt_;
};

}  // namespace

void go() {
  setup();
  const auto view = viewer::get_window3d("Filter Debug");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  const std::vector<std::string> channel_names = {"imu", "camera_image_channel"};
  const std::string path = "/jet/logs/20190110065258";

  Calibrator calibrator;
  jet::LogReader reader(path, channel_names);

  for (int k = 0; k < 3000; ++k) {
    ImuMessage msg;
    CameraImageMessage cam_msg;
    if (reader.read_next_message("imu", msg)) {
      calibrator.maybe_add_imu(msg);
    } else {
      std::cout << "Breaking at : " << k << std::endl;
      break;
    }
    if (reader.read_next_message("camera_image_channel", cam_msg)) {
      const auto image = get_image_mat(cam_msg);

      const auto result = detect_board(image);
      if (result) {
        const SE3 world_from_camera = *result;
        calibrator.add_fiducial(msg.timestamp, world_from_camera);

        geo->add_axes({world_from_camera});
        geo->flush();
        // view->spin_until_step();
      }

      // cv::imshow("Image", image);
      // cv::waitKey(1);
    }
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
