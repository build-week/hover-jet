
#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "camera/camera_image_message.hh"
#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "camera/camera_manager.hh"
#include "camera/webcam_interface.hh"

#include "infrastructure/comms/shared_struct/shared_struct_publisher.hh"

//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

namespace jet {

// A C++ form of the yaml config
//
// Stored this way to avoid parsing overhead when re-reading the
// config each frame
//
// The fields are left intentionally uninitialized, so that
// their use before initialization can be detected with Valgrind

class CameraBq : public BalsaQ {
 public:
  CameraBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  // Use this to convert a capture time to a wall time
  //
  // @param msec_monotonic The timestamp in int milliseconds reported by "the" monotonic clock
  //
  // The camera capture times are reported in millseconds from monotonic clock start
  // This function reconstructs a a reasonable approximation of the *vehicle* time
  // associated with that monotonic time
  static const int CAMERA_FPS = 10;
  PublisherPtr publisher_;
  cv::VideoCapture cap_;
  Timestamp last_msg_recvd_timestamp_;

  Camera camera_;
  CameraConfiguration camera_config_;
  CameraManager camera_manager_ = CameraManager();
  std::unique_ptr<SharedStructPublisher> camera_shmem_publisher_;


};

}  // namespace jet
