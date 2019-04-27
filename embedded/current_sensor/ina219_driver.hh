#pragma once
//%deps(i2c)

#include "ina219_driver_configuration.hh"

#include "third_party/i2c/i2c.h"

#include <stdint.h>

#include <optional>

namespace ina219 {

static constexpr float MILLIVOLTS_PER_MICROVOLT{0.01};
static constexpr float VOLTS_PER_MILLIVOLT{0.001};
static constexpr int32_t I2C_ADDR_BYTES{1};
static constexpr int32_t I2C_PAGE_BYTES{16};

/// @brief  Register IDs
enum class Register : uint8_t {
  CONFIG = 0x00,
  SHUNT_VOLTAGE = 0x01,
  BUS_VOLTAGE = 0x02,
  POWER = 0x03,
  CURRENT = 0x04,
  CALIBRATION = 0x05
};

/// @brief Class for interacting with the INA219 I2C current sensor
class INA219Driver {
 public:
  /// @brief  INA219Driver constructor
  /// @param  i2c_handle - The I2C bus file descriptor
  /// @param  i2c_address - The I2C address of the INA219 device
  /// @param  config - The configuration for this INA219
  INA219Driver(int i2c_handle, int i2c_address, DriverConfiguration config);

  /// @brief  Gets the shunt voltage in volts
  /// @return Bus voltage in volts
  std::optional<float> get_bus_voltage_V() const;

  /// @brief  Gets the shunt voltage in mV
  /// @return Shunt voltage in millivolts
  std::optional<float> get_shunt_voltage_mV() const;

  /// @brief  Gets the current value in mA
  /// @return Current in milliamps
  std::optional<float> get_current_mA() const;

  /// @brief  Gets the measured power in mW
  /// @return Power in milliwatts
  std::optional<float> get_power_mW() const;

 private:
  const DriverConfiguration config_;

  /// @brief  Writes two bytes to an INA219 register
  /// @param  reg - ID of the register to write to
  /// @param  value - The value to write
  /// @return Returns true on success, false on failure
  [[nodiscard]] bool write_register(Register reg, uint16_t value) const;

  /// @brief  Reads 16 bits from an INA219 register
  /// @param  reg - ID of the register to read from
  /// @param  value - Buffer to write the value to
  /// @return Returns true on success, false on failure
  [[nodiscard]] bool read_register(Register reg, uint16_t& value) const;

  /// @brief  Gets the raw bus voltage value
  /// @return The value read rom the INA219's bus voltage register
  std::optional<int16_t> get_raw_bus_voltage() const;

  /// @brief  Gets the raw shunt voltage value
  /// @return The value read rom the INA219's shunt voltage register
  std::optional<int16_t> get_raw_shunt_voltage() const;

  /// @brief  Gets the raw current value
  /// @return The value read rom the INA219's current register
  std::optional<int16_t> get_raw_current() const;

  /// @brief  Gets the raw power value
  /// @return The value read rom the INA219's power register
  std::optional<int16_t> get_raw_power() const;

  i2c_device i2c_device_;
};

}  // namespace ina219
