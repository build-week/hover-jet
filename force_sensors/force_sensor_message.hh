#pragma once

#include "infrastructure/time/timestamp.hh"
#include "infrastructure/comms/schemas/message.hh"

//%deps(opencv)
#include <opencv2/opencv.hpp>

namespace jet {

struct ForceSensorMessage : Message {
  Timestamp timestamp;
  int force_sensor_index;

  MESSAGE(ForceSensorMessage, timestamp, force_sensor_index);
};

cv::Mat get_image_mat(ForceSensorMessage message);

}  //  namespace jet
