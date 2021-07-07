#include "ina219_driver_configuration.hh"

namespace ina219 {

DriverConfiguration DriverConfiguration::make_32V_2A() {
  return DriverConfiguration(10,
                             2,
                             4096,
                             BusVoltageRange::RANGE_32V,
                             PGAGain::GAIN_8_320MV,
                             BusADCResolution::RES_12BIT,
                             ShuntADCResolution::RES_12BIT_1S_532US,
                             OperatingMode::SANDBVOLT_CONTINUOUS);
}

DriverConfiguration DriverConfiguration::make_32V_1A() {
  return DriverConfiguration(25,
                             0.8,
                             10240,
                             BusVoltageRange::RANGE_32V,
                             PGAGain::GAIN_8_320MV,
                             BusADCResolution::RES_12BIT,
                             ShuntADCResolution::RES_12BIT_1S_532US,
                             OperatingMode::SANDBVOLT_CONTINUOUS);
}

DriverConfiguration DriverConfiguration::make_16V_400mA() {
  return DriverConfiguration(20,
                             1.0,
                             8192,
                             BusVoltageRange::RANGE_16V,
                             PGAGain::GAIN_1_40MV,
                             BusADCResolution::RES_12BIT,
                             ShuntADCResolution::RES_12BIT_1S_532US,
                             OperatingMode::SANDBVOLT_CONTINUOUS);
}

DriverConfiguration::DriverConfiguration(uint32_t current_divider_mA,
                                         float power_multiplier_mW,
                                         uint32_t calibration_value,
                                         BusVoltageRange bus_voltage_range,
                                         PGAGain pga_gain,
                                         BusADCResolution bus_adc_resolution,
                                         ShuntADCResolution shunt_adc_resolution,
                                         OperatingMode operating_mode)
    : current_divider_mA(current_divider_mA),
      power_multiplier_mW(power_multiplier_mW),
      calibration_value(calibration_value),
      ina219_configuration(calculate_ina219_configuration_value(
          bus_voltage_range, pga_gain, bus_adc_resolution, shunt_adc_resolution, operating_mode)) {
}

uint16_t DriverConfiguration::calculate_ina219_configuration_value(BusVoltageRange bus_voltage_range,
                                                                   PGAGain pga_gain,
                                                                   BusADCResolution bus_adc_resolution,
                                                                   ShuntADCResolution shunt_adc_resolution,
                                                                   OperatingMode operating_mode) const {
  return static_cast<uint16_t>(bus_voltage_range) | static_cast<uint16_t>(pga_gain) |
         static_cast<uint16_t>(bus_adc_resolution) | static_cast<uint16_t>(shunt_adc_resolution) |
         static_cast<uint16_t>(operating_mode);
}

}  // namespace ina219
