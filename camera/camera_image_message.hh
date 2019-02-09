#pragma once

#include "infrastructure/time/timestamp.hh"
#include "infrastructure/comms/schemas/message.hh"

//%deps(opencv)
#include <opencv2/opencv.hpp>

namespace jet {

struct CameraImageMessage : Message {
  std::vector<uchar> image_data;
  uint width, height;
  std::string camera_serial_number;

  MESSAGE(CameraImageMessage, 
          image_data, 
          width, 
          height,
          camera_serial_number);
};

cv::Mat get_image_mat(CameraImageMessage message);

}  //  namespace jet
