#pragma once

#include "infrastructure/time/timestamp.hh"
#include "infrastructure/comms/schemas/message.hh"

//%deps(opencv)
#include <opencv2/opencv.hpp>

namespace jet {

struct CameraImageMessage : Message {
  std::vector<uchar> image_data;
  Timestamp timestamp;
  uint width, height;

  MESSAGE(CameraImageMessage, image_data, timestamp, width, height);
};

cv::Mat get_image_mat(CameraImageMessage message);

}  //  namespace jet
