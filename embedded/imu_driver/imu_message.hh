#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/comms/serialization/serialization_macros.hh"
#include "infrastructure/time/timestamp.hh"

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

  double mag_utesla_x;
  double mag_utesla_y;
  double mag_utesla_z;

  // Timestamp that the observation was *generated*
  Timestamp timestamp;

  // Unique identifier: Leading byte of serial number
  uint8_t imu_id_leading_byte;

  MESSAGE(ImuMessage,
          accel_mpss_x,
          accel_mpss_y,
          accel_mpss_z,
          gyro_radps_x,
          gyro_radps_y,
          gyro_radps_z,
          mag_utesla_x,
          mag_utesla_y,
          mag_utesla_z,
          timestamp,
          imu_id_leading_byte);
};

}  // namespace jet
