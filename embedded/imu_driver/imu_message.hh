#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/time/timestamp.hh"
#include "infrastructure/comms/serialization/serialization_macros.hh"

#include <string>

namespace jet {

// Intentionally uninitialized, for detectability via valgrind
struct ImuMessage : Message {
  // TODO(jake): Implement a serializable matrix type
  double accel_mpss_x;
  double accel_mpss_y;
  double accel_mpss_z;

  double gyro_radps_x;
  double gyro_radps_y;
  double gyro_radps_z;

  // Timestamp that the observation was *generated*
  Timestamp timestamp;

  MESSAGE(ImuMessage,
          accel_mpss_x,
          accel_mpss_y,
          accel_mpss_z,
          gyro_radps_x,
          gyro_radps_y,
          gyro_radps_z,
          timestamp);
};

}  // namespace jet
