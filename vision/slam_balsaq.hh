//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include "infrastructure/balsa_queue/balsa_queue.hh"

#include "camera/camera_image_message.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {



class SlamBq : public BalsaQ {
 public:
  SlamBq() = default;
  void init(const Config& config) override;
  void loop() override;
  void shutdown() override;

 private:
  PublisherPtr publisher_;
  SubscriberPtr subscriber_;
  static constexpr bool OPEN_DEBUG_WINDOWS = false;
  Timestamp last_msg_recvd_timestamp_;
  cv::Mat old_image;
  // ImageWithLandmarks old_image_w_landmarks;
};

}  // namespace jet