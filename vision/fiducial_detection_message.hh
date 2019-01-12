#pragma once

#include "infrastructure/comms/schemas/message.hh"

#include <string>

namespace jet {

struct FiducialDetectionMessage : Message {
  std::array<double, 6> fiducial_from_camera_log;

  // Timestamp of the image that was used to generate this
  Timestamp timestamp;

  SE3 fiducial_from_camera() const;

  MESSAGE(FiducialDetectionMessage, fiducial_from_camera_log, timestamp);
};

}  //  namespace jet
