#pragma once

#include "infrastructure/comms/schemas/message.hh"

//%deps(opencv)
#include <opencv2/opencv.hpp>

namespace jet {

struct CameraImageMessage : Message {
  std::vector<uchar> image_data;
  uint64_t timestamp_ns;
  uint width, height;

  MESSAGE(CameraImageMessage, image_data, timestamp_ns, width, height);
};

cv::Mat get_image_mat(CameraImageMessage message);

}  //  namespace jet
