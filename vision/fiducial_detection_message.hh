#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "vision/fiducial_detection_and_pose.hh"

//%deps(opencv)
#include <opencv2/opencv.hpp>

#include <string>

namespace jet {

struct FiducialDetectionMessage : Message {
  int marker_id;
  MESSAGE(FiducialDetectionMessage, marker_id);
};

}  //  namespace jet
