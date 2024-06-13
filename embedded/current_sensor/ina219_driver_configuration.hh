#pragma once

#include <stdint.h>

namespace ina219 {

struct DriverConfiguration {
  /// @brief  Bus voltage ranges
  enum class BusVoltageRange : uint16_t {
    RANGE_16V = (0x0000),
    RANGE_32V = (0x2000),
  };

  /// @brief  PGA gains
  enum class PGAGain : uint16_t {
    GAIN_1_40MV = (0x0000),
    GAIN_2_80MV = (0x0800),
    GAIN_4_160MV = (0x1000),
    GAIN_8_320MV = (0x1800),
  };

  /// @brief  Bus ADC resolutions
  enum class BusADCResolution : uint16_t {
    RES_9BIT = (0x0000),
    RES_10BIT = (0x0080),
    RES_11BIT = (0x0100),
    RES_12BIT = (0x0180),
  };

  /// @brief  Shunt ADC resolutions
  enum class ShuntADCResolution : uint8_t {
    RES_9BIT_1S_84US = (0x0000),
    RES_10BIT_1S_148US = (0x0008),
    RES_11BIT_1S_276US = (0x0010),
    RES_12BIT_1S_532US = (0x0018),
    RES_12BIT_2S_1060US = (0x0048),
    RES_12BIT_4S_2130US = (0x0050),
    RES_12BIT_8S_4260US = (0x0058),
    RES_12BIT_16S_8510US = (0x0060),
    RES_12BIT_32S_17MS = (0x0068),
    RES_12BIT_64S_34MS = (0x0070),
    RES_12BIT_128S_69MS = (0x0078),
  };

  /// @brief  Operating modes
  enum class OperatingMode : uint8_t {
    POWERDOWN = (0x0000),
    SVOLT_TRIGGERED = (0x0001),
    BVOLT_TRIGGERED = (0x0002),
    SANDBVOLT_TRIGGERED = (0x0003),
    ADCOFF = (0x0004),
    SVOLT_CONTINUOUS = (0x0005),
    BVOLT_CONTINUOUS = (0x0006),
    SANDBVOLT_CONTINUOUS = (0x0007),
  };

  /// @brief Convenience functions for generating common configurations
  static DriverConfiguration make_32V_2A();
  static DriverConfiguration make_32V_1A();
  static DriverConfiguration make_16V_400mA();

  uint32_t current_divider_mA{0};
  float power_multiplier_mW{0};
  uint32_t calibration_value{0};
  uint16_t ina219_configuration{0};

 private:
  DriverConfiguration(uint32_t current_divider_mA,
                      float power_multiplier_mW,
                      uint32_t calibration_value,
                      BusVoltageRange bus_voltage_range,
                      PGAGain pga_gain,
                      BusADCResolution bus_adc_resolution,
                      ShuntADCResolution shunt_adc_resolution,
                      OperatingMode operating_mode);

  uint16_t calculate_ina219_configuration_value(BusVoltageRange bus_voltage_range,
                                                PGAGain pga_gain,
                                                BusADCResolution bus_adc_resolution,
                                                ShuntADCResolution shunt_adc_resolution,
                                                OperatingMode operating_mode) const;
};

}  // namespace ina219
