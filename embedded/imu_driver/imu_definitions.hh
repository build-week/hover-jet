#pragma once

#include <cstdint>
#include "third_party/experiments/eigen.hh"


namespace jet {
namespace embedded {

// clang-format off
struct GyroConfig0 {
  static constexpr uint8_t DPS_2000    = 0b000;
  static constexpr uint8_t DPS_1000    = 0b001;
  static constexpr uint8_t DPS_500     = 0b010;
  static constexpr uint8_t DPS_250     = 0b011;
  static constexpr uint8_t DPS_125     = 0b100;
  static constexpr uint8_t HZ_523   = 0b000000;
  static constexpr uint8_t HZ_230   = 0b001000;
  static constexpr uint8_t HZ_116   = 0b010000;
};

struct AccConfig {
  static constexpr uint8_t g_2         = 0b00;
  static constexpr uint8_t g_4         = 0b01;
  static constexpr uint8_t g_8         = 0b10;
  static constexpr uint8_t g_16        = 0b11;

  static constexpr uint8_t HZ_7_81  = 0b00000;
  static constexpr uint8_t HZ_15_63 = 0b00100;
  static constexpr uint8_t HZ_62_5  = 0b01100;
  static constexpr uint8_t HZ_125   = 0b10000;
  static constexpr uint8_t HZ_250   = 0b10100;
};

struct ConfigAdresses {
  static constexpr uint8_t GYR_CONFIG_0 = 0x38;
  static constexpr uint8_t GYR_CONFIG_1 = 0x00;
  static constexpr uint8_t ACC_CONFIG   = 0x0D;
  static constexpr uint8_t MAG_CONFIG   = 0x6D;
};
// clang-format on

struct ImuMeasurements {
  jcc::Vec3 accel_mpss;
  jcc::Vec3 angvel_radps;
  jcc::Vec3 mag_utesla;
};

}  // namespace embedded
}  // namespace jet
