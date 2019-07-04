#include "infrastructure/config/config.hh"
#include "infrastructure/time/timestamp.hh"
// #include "infrastructure/time/duration.hh"
#include "infrastructure/balsa_queue/balsa_queue.hh"

//%deps(opencv)
#include <opencv2/opencv.hpp>

namespace jet {
struct CameraConfiguration {
  int frames_per_second;
  double exposure;

  int auto_focus;
  double auto_exposure;

  int width_pixels;
  int height_pixels;
};

CameraConfiguration generate_capture_config(const Config& config);

void initialize_camera_hardware(CameraConfiguration camera_config, cv::VideoCapture cap);


}  // namespace jet