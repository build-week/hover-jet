#include <infrastructure/balsa_queue/balsa_queue.hh>
//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "camera/camera_image_message.hh"
#include "vision/fiducial_detection_and_pose.hh"
#include "vision/fiducial_detection_message.hh"

#include "camera/camera_manager.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {

std::optional<FiducialDetectionMessage> create_detection_message(const cv::Mat& camera_frame,
                                                                 const Calibration& camera_calibration,
                                                                 const Timestamp timestamp);

class FiducialDetectionBq : public BalsaQ {
 public:
  FiducialDetectionBq() = default;
  void init(const Config& config) override;
  void loop() override;
  void shutdown() override;
  const static uint loop_delay_microseconds = 50000;

 private:
  PublisherPtr publisher_;
  SubscriberPtr subscriber_;
  static constexpr bool OPEN_DEBUG_WINDOWS = false;
  CameraManager camera_manager_ = CameraManager();
  Timestamp last_msg_recvd_timestamp_;
};

}  // namespace jet
