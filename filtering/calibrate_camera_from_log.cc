//%deps(simple_geometry)
//%deps(ui2d)
//%deps(window_3d)
//%deps(time_point)
#include "third_party/experiments/estimation/time_point.hh"
#include "third_party/experiments/logging/log.hh"
#include "third_party/experiments/viewer/interaction/ui2d.hh"
#include "third_party/experiments/viewer/primitives/simple_geometry.hh"
#include "third_party/experiments/viewer/window_3d.hh"

#include "filtering/extract_data_from_log.hh"

#include "vision/fiducial_detection_and_pose.hh"

//%deps(yaml-cpp)
#include <yaml-cpp/yaml.h>

namespace jet {
namespace filtering {

namespace {

bool visualize() {
  return true;
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
}  // namespace

void calibrate_camera(const std::string& log_path) {
  jcc::Success() << "Preparing to calibrate" << std::endl;

  //
  // Set up the viewer
  //

  setup();

  //
  // Grab images
  //

  jcc::Success() << "[Camera] Validating fiducial measurements" << std::endl;
  const TimeRange range{};
  ImageStream image_stream(log_path, range);
  jcc::Success() << "[Camera] Parsing images" << std::endl;

  //
  // View stuff
  //

  const auto view = viewer::get_window3d("Calibration");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  const auto ui2d = view->add_primitive<viewer::Ui2d>();

  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;

  std::string serial_number = "";
  int cols, rows;

  constexpr int DECIMATION = 25;

  int i = 0;
  while (true) {
    const auto image = image_stream.next();
    ++i;
    if (i % DECIMATION != 0) {
      continue;
    }
    if (!image) {
      break;
    }


    const auto ids_corners = get_ids_and_corners(image->image);
    const auto obj_pt_associations = obj_points_img_points_from_image(ids_corners);

    serial_number = image->serial_number;
    cols = image->image.cols;
    rows = image->image.rows;

    if (visualize()) {
      ui2d->clear();
      geo->clear();
      ui2d->add_image(image->image, 1.0);

      if (!obj_pt_associations.empty()) {
        //
        // Make OpenCV points for calibration
        //

        std::vector<cv::Point3f> this_object_points;
        std::vector<cv::Point2f> this_image_points;
        for (const auto& assoc : obj_pt_associations) {
          const jcc::Vec2 pt_board_surface = jcc::Vec2(assoc.point_board_space);
          const jcc::Vec3 obj_pt_dbl = jcc::Vec3(pt_board_surface.x(), pt_board_surface.y(), 0.0);
          const jcc::Vec2 image_pt_dbl = assoc.point_image_space;

          const Eigen::Vector2f image_pt = image_pt_dbl.cast<float>();
          const Eigen::Vector3f obj_pt = obj_pt_dbl.cast<float>();
          this_image_points.push_back({image_pt.x(), image_pt.y()});
          this_object_points.push_back({obj_pt.x(), obj_pt.y(), obj_pt.z()});

          const jcc::Vec4 blue(0.0, 0.0, 1.0, 1.0);
          constexpr double PT_SIZE = 5.0;
          ui2d->add_point({image_pt_dbl / image->image.rows, blue, PT_SIZE});
        }
        image_points.push_back(this_image_points);
        object_points.push_back(this_object_points);
      }

      ui2d->flip();
      geo->flip();

      view->spin_until_step();
    }
  }

  cv::Mat K;
  cv::Mat D;
  std::vector<cv::Mat> rvecs, tvecs;
  jcc::Warning() << "Calibrating camera, this might take a while..." << std::endl;
  int flag = 0;
  const auto t0 = jcc::now();
  cv::Size resolution(480, 270);
  cv::calibrateCamera(object_points, image_points, resolution, K, D, rvecs, tvecs, flag);
  const auto t1 = jcc::now();
  jcc::Success() << "Done. Took: " << estimation::to_seconds(t1 - t0) << std::endl;
  jcc::Success() << "Generating yaml..." << std::endl;

  YAML::Node node;
  node["serial_number"] = serial_number;

  {
    std::stringstream ss;
    ss << "usb-046d_Logitech_Webcam_C930e_" << serial_number << "-video-index0";
    node["v4l_path"] = ss.str();
  }

  for (int i = 0; i < D.cols; ++i) {
    node["distortion_coefficients"].push_back(D.at<double>(0, i));
  }
  for (int i = 0; i < K.cols; ++i) {
    for (int j = 0; j < K.rows; ++j) {
      node["camera_matrix"].push_back(K.at<double>(i, j));
    }
  }
  node["resolution"].push_back(cols);
  node["resolution"].push_back(rows);

  node["source_log"] = log_path;

  std::cout << "\n\n\n" << std::endl;
  std::cout << node << std::endl;
  std::cout << "\n\n\n" << std::endl;
}

}  // namespace filtering
}  // namespace jet

int main(int argc, char** argv) {
  assert(argc > 1);
  const std::string path(argv[1]);
  jet::filtering::calibrate_camera(path);
}