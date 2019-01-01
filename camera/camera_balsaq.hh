#include <infrastructure/balsa_queue/balsa_queue.hh>
//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "camera/camera_image_message.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"


namespace jet {

class CameraBq : public BalsaQ {
 public:
  CameraBq();
  void init();
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
  cv::VideoCapture cap;
};

}  // namespace jet
