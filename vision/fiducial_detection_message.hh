#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "vision/fiducial_detection_and_pose.hh"
#include "third_party/experiments/eigen.hh"

//%deps(opencv)
#include <opencv2/opencv.hpp>

#include <string>

namespace jet {

struct FiducialDetectionMessage : Message {
  std::array<double, 6> fiducial_from_camera_log;
  MESSAGE(FiducialDetectionMessage, fiducial_from_camera_log);
};

}  //  namespace jet
