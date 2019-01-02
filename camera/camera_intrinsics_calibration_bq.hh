#include <infrastructure/balsa_queue/balsa_queue.hh>
//%deps(opencv)
#include <opencv2/opencv.hpp>

#include "camera/camera_image_message.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {

constexpr int BOARD_WIDTH = 9;
constexpr int BOARD_HEIGHT = 6;
constexpr float SQUARE_SIZE = .1;

struct CameraIntrinsics{
	cv::Mat K;
	cv::Mat D;
};


class CameraIntrincicsCalibrationBq : public BalsaQ {
 public:
  CameraIntrincicsCalibrationBq();
  void init() override;
  void loop() override;
  void shutdown() override;
  void add_camera_image(const cv::Mat image);
  int num_images_collected() const;
  CameraIntrinsics calibrate() const;

 private:
  PublisherPtr publisher_;
  SubscriberPtr subscriber_;
  std::vector<cv::Mat> all_camera_images_;

};

}  // namespace jet
