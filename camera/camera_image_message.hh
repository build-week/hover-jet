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
  std::string camera_serial_number;

  MESSAGE(CameraImageMessage,
          image_data,
          timestamp,
          width,
          height,
          camera_serial_number);
};

cv::Mat get_image_mat(const CameraImageMessage& message);


CameraImageMessage create_camera_image_message(const cv::Mat& camera_frame,
                                               const std::string& serial_number,
                                               const Timestamp cap_time_vehicle);


}  //  namespace jet
