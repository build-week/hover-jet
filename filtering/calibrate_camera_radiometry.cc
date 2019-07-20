#include "camera/camera_manager.hh"
#include "camera/webcam_interface.hh"

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

//%deps(ui2d)
#include "third_party/experiments/viewer/interaction/ui2d.hh"
//%deps(simple_geometry)
#include "third_party/experiments/viewer/primitives/simple_geometry.hh"
//%deps(window_3d)
#include "third_party/experiments/viewer/window_3d.hh"

#include <cstddef>
#include <iostream>
#include <sstream>

namespace jet {

void capture_frames(cv::VideoCapture& cap, int gamma) {
  const auto view = viewer::get_window3d("Radiometry");
  const auto ui2d = view->add_primitive<viewer::Ui2d>();

  viewer::LinePlotBuilder builder("Radiometry");
  auto& red = builder.make_subplot("red", jcc::Vec4(1.0, 0.0, 0.0, 1.0), 3.0, false, viewer::LinePlotStyle::Scatter);
  auto& green =
      builder.make_subplot("green", jcc::Vec4(0.0, 1.0, 0.0, 1.0), 3.0, false, viewer::LinePlotStyle::Scatter);
  auto& blue = builder.make_subplot("blue", jcc::Vec4(0.0, 0.0, 1.0, 1.0), 3.0, false, viewer::LinePlotStyle::Scatter);
  auto& yellow =
      builder.make_subplot("yellow", jcc::Vec4(1.0, 1.0, 0.0, 1.0), 3.0, false, viewer::LinePlotStyle::Scatter);
  auto& teal = builder.make_subplot("teal", jcc::Vec4(0.0, 1.0, 1.0, 1.0), 3.0, false, viewer::LinePlotStyle::Scatter);

  int frames_per_exposure = 20;
  cv::Mat camera_frame;
  for (int exp = 1; exp <= 2048; exp *= 2) {
    // Update exposure settings and clear buffer
    cap.set(cv::CAP_PROP_EXPOSURE, exp);
    for (int i = 0; i < 50; ++i) {
      cap.read(camera_frame);
    }

    for (int frame_num = 0; frame_num < frames_per_exposure; ++frame_num) {
      if (cap.read(camera_frame)) {
        // cv::imshow("banana", camera_frame);

        const int exposure_real = cap.get(cv::CAP_PROP_EXPOSURE);

        const cv::Vec3b bgr_pixel_1 = camera_frame.at<cv::Vec3b>(200, 200);
        const cv::Vec3b bgr_pixel_2 = camera_frame.at<cv::Vec3b>(100, 200);
        const cv::Vec3b bgr_pixel_3 = camera_frame.at<cv::Vec3b>(200, 100);
        const cv::Vec3b bgr_pixel_4 = camera_frame.at<cv::Vec3b>(30, 30);
        const cv::Vec3b bgr_pixel_5 = camera_frame.at<cv::Vec3b>(230, 430);

        const int b_1 = static_cast<int>(bgr_pixel_1[0]);
        const int b_2 = static_cast<int>(bgr_pixel_2[0]);
        const int b_3 = static_cast<int>(bgr_pixel_3[0]);

        const int b_4 = static_cast<int>(bgr_pixel_4[0]);
        const int b_5 = static_cast<int>(bgr_pixel_5[0]);

        std::cout << "[" << exposure_real << ", " << b_1 << ", " << b_2 << ", " << b_3 << ", " << b_4 << ", " << b_5
                  << "]," << std::endl;

        // red << jcc::Vec2(exposure_real, static_cast<double>(bgr_pixel[0]));
        // green << jcc::Vec2(exposure_real, static_cast<double>(bgr_pixel[1]));
        // blue << jcc::Vec2(exposure_real, static_cast<double>(bgr_pixel[2]));
        red << jcc::Vec2(exposure_real, static_cast<double>(b_1));
        green << jcc::Vec2(exposure_real, static_cast<double>(b_2));
        blue << jcc::Vec2(exposure_real, static_cast<double>(b_3));
        yellow << jcc::Vec2(exposure_real, static_cast<double>(b_4));
        teal << jcc::Vec2(exposure_real, static_cast<double>(b_5));

        cv::waitKey(10);
      }
    }
    ui2d->add_lineplot(builder);
    ui2d->flip();
    // std::cout << "Exposure: " << exp << std::endl;
  }
  view->spin_until_step();
}

void go() {
  const CameraManager camera_manager;

  const std::string serial_number = "CA38DB5E";
  const auto camera = camera_manager.get_camera(serial_number);

  CameraConfiguration camera_config;

  camera_config.frames_per_second = 10;
  camera_config.exposure = 3;
  camera_config.auto_focus = 0;
  camera_config.auto_exposure = 1;
  camera_config.width_pixels = 480;
  camera_config.height_pixels = 270;

  auto cap = cv::VideoCapture(camera.v4l_path);
  initialize_camera_hardware(camera_config, cap);

  // CAP_PROP_GAMMA
  // CAP_PROP_GAIN

  int gamma = 1;
  capture_frames(cap, gamma);
}

}  // namespace jet

int main() {
  jet::go();
}