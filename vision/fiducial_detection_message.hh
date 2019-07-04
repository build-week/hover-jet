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

  // Timestamp of the image that was used to generate this
  Timestamp timestamp;
  std::vector<BoardPointImagePointAssociation> board_points_image_points;
  SE3 fiducial_from_camera() const;

  MESSAGE(FiducialDetectionMessage, fiducial_from_camera_log, timestamp,board_points_image_points);
};

std::optional<FiducialDetectionMessage> create_fiducial_detection_message(const cv::Mat& camera_frame,
                                                              const Calibration& camera_calibration,
                                                              const Timestamp timestamp);

}  //  namespace jet
