#include "ina219_driver.hh"

#include <string.h>  // memset

namespace ina219 {

[[nodiscard]] bool INA219Driver::write_register(Register reg, uint16_t value) const {
  uint16_t write_buffer = ((value & 0xFF) << 8) + (value >> 8);
  if (i2c_write(&i2c_device_, static_cast<uint8_t>(reg), &write_buffer, 2) == -1) {
    return false;
  }
  return true;
}

    [[nodiscard]] bool INA219Driver::read_register(Register reg, uint16_t& value) const {
  uint16_t read_buffer;
  if (i2c_read(&i2c_device_, static_cast<uint8_t>(reg), &read_buffer, 2) == -1) {
    return false;
  }
  value = ((read_buffer & 0xFF) << 8) + (read_buffer >> 8);
  return true;
}

INA219Driver::INA219Driver(int i2c_handle, int i2c_address, DriverConfiguration config) : config_(config) {
  memset(&i2c_device_, 0, sizeof(i2c_device_));
  i2c_device_.bus = i2c_handle;
  i2c_device_.addr = i2c_address;
  i2c_device_.iaddr_bytes = I2C_ADDR_BYTES;
  i2c_device_.page_bytes = I2C_PAGE_BYTES;
}

std::optional<int16_t> INA219Driver::get_raw_bus_voltage() const {
  uint16_t raw_bus_voltage_buffer;
  if (!read_register(Register::BUS_VOLTAGE, raw_bus_voltage_buffer)) {
    return {};
  }

  return (int16_t)((raw_bus_voltage_buffer >> 3) * 4);
}

std::optional<int16_t> INA219Driver::get_raw_shunt_voltage() const {
  uint16_t raw_shunt_voltage_buffer;
  if (!read_register(Register::SHUNT_VOLTAGE, raw_shunt_voltage_buffer)) {
    return {};
  }
  return static_cast<int16_t>(raw_shunt_voltage_buffer);
}

std::optional<int16_t> INA219Driver::get_raw_current() const {
  if (!write_register(Register::CALIBRATION, config_.calibration_value)) {
    return {};
  }

  uint16_t raw_current_buffer;
  if (!read_register(Register::CURRENT, raw_current_buffer)) {
    return {};
  }

  return static_cast<int16_t>(raw_current_buffer);
}

std::optional<int16_t> INA219Driver::get_raw_power() const {
  if (!write_register(Register::CALIBRATION, config_.calibration_value)) {
    return {};
  }

  uint16_t raw_power_buffer;
  if (!read_register(Register::POWER, raw_power_buffer)) {
    return {};
  }

  return static_cast<int16_t>(raw_power_buffer);
}

std::optional<float> INA219Driver::get_shunt_voltage_mV() const {
  std::optional<int16_t> raw_shunt_voltage_value = get_raw_shunt_voltage();
  if (!raw_shunt_voltage_value) {
    return {};
  }
  return raw_shunt_voltage_value.value() * MILLIVOLTS_PER_MICROVOLT;
}

std::optional<float> INA219Driver::get_bus_voltage_V() const {
  std::optional<int16_t> raw_bus_voltage_value = get_raw_bus_voltage();
  if (!raw_bus_voltage_value) {
    return {};
  }
  return raw_bus_voltage_value.value() * VOLTS_PER_MILLIVOLT;
}

std::optional<float> INA219Driver::get_current_mA() const {
  std::optional<float> raw_current_value = get_raw_current();
  if (!raw_current_value) {
    return {};
  }
  return raw_current_value.value() / config_.current_divider_mA;
}

std::optional<float> INA219Driver::get_power_mW() const {
  std::optional<float> raw_power_value = get_raw_power();
  if (!raw_power_value) {
    return {};
  }
  return raw_power_value.value() * config_.power_multiplier_mW;
}

}  // namespace ina219
