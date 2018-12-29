#include <infrastructure/balsa_queue/balsa_queue.hh>
//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

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
