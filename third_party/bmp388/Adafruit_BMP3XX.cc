/*!
 * @file Adafruit_BMP3XX.cpp
 *
 * @mainpage Adafruit BMP3XX temperature & barometric pressure sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's BMP3XX driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BMP388 breakout: https://www.adafruit.com/products/3966
 *
 * These sensors use I2C or SPI to communicate
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_BMP3XX.hh"
#include <iostream>
#include <math.h>
#include <unistd.h> // usleep

// Our hardware interface functions
static void delay_msec(uint32_t ms);

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates sensor with Hardware SPI or I2C.
    @param  cspin SPI chip select. If not passed in, I2C will be used
*/
/**************************************************************************/
Adafruit_BMP3XX::Adafruit_BMP3XX(std::string i2cBus)
  : _meas_end(0),
    _i2cBus(i2cBus) {
  _filterEnabled = _tempOSEnabled = _presOSEnabled = false;
}

/**************************************************************************/
/*!
    @brief Initializes the sensor

    Hardware ss initialized, verifies it is in the I2C or SPI bus, then reads
    calibration data in preparation for sensor reads.

    @param  addr Optional parameter for the I2C address of BMP3. Default is 0x77
    @param  theWire Optional parameter for the I2C device we will use. Default is "Wire"
    @return True on sensor initialization success. False on failure.
*/
/**************************************************************************/
bool Adafruit_BMP3XX::begin(uint8_t addr) {
  _i2caddr = addr;

  _i2cDevice = {};

  _i2cDevice.bus = i2c_open(static_cast<const char*>(_i2cBus.c_str()));
  if (_i2cDevice.bus == -1) {
    return false;
  }

  _i2cDevice.addr = addr;
  _i2cDevice.iaddr_bytes = 1;
  _i2cDevice.page_bytes = 16;

  the_sensor.dev_id = addr;
  the_sensor.intf = BMP3_I2C_INTF;
  the_sensor.read = &i2c_read_wrapper;
  the_sensor.write = &i2c_write_wrapper;
  the_sensor.delay_ms = delay_msec;

  int8_t rslt = BMP3_OK;
  rslt = bmp3_init(&the_sensor);
#ifdef BMP3XX_DEBUG
  std::cout << "Result: " << rslt << std::endl;
#endif

  if (rslt != BMP3_OK)
    return false;

#ifdef BMP3XX_DEBUG
  std::cout << "T1 = " << the_sensor.calib_data.reg_calib_data.par_t1 << std::endl;
  std::cout << "T2 = " << the_sensor.calib_data.reg_calib_data.par_t2 << std::endl;
  std::cout << "T3 = " << the_sensor.calib_data.reg_calib_data.par_t3 << std::endl;
  std::cout << "P1 = " << the_sensor.calib_data.reg_calib_data.par_p1 << std::endl;
  std::cout << "P2 = " << the_sensor.calib_data.reg_calib_data.par_p2 << std::endl;
  std::cout << "P3 = " << the_sensor.calib_data.reg_calib_data.par_p3 << std::endl;
  std::cout << "P4 = " << the_sensor.calib_data.reg_calib_data.par_p4 << std::endl;
  std::cout << "P5 = " << the_sensor.calib_data.reg_calib_data.par_p5 << std::endl;
  std::cout << "P6 = " << the_sensor.calib_data.reg_calib_data.par_p6 << std::endl;
  std::cout << "P7 = " << the_sensor.calib_data.reg_calib_data.par_p7 << std::endl;
  std::cout << "P8 = " << the_sensor.calib_data.reg_calib_data.par_p8 << std::endl;
  std::cout << "P9 = " << the_sensor.calib_data.reg_calib_data.par_p9 << std::endl;
  std::cout << "P10 = " << the_sensor.calib_data.reg_calib_data.par_p10 << std::endl;
  std::cout << "P11 = " << the_sensor.calib_data.reg_calib_data.par_p11 << std::endl;
#endif

  setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  setPressureOversampling(BMP3_NO_OVERSAMPLING);
  setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);

  // don't do anything till we request a reading
  the_sensor.settings.op_mode = BMP3_FORCED_MODE;

  return true;
}


/**************************************************************************/
/*!
    @brief Performs a reading and returns the ambient temperature.
    @return Temperature in degrees Centigrade
*/
/**************************************************************************/
float Adafruit_BMP3XX::readTemperature(void) {
  performReading();
  return temperature;
}


/**************************************************************************/
/*!
    @brief Performs a reading and returns the barometric pressure.
    @return Barometic pressure in Pascals
*/
/**************************************************************************/
float Adafruit_BMP3XX::readPressure(void) {
  performReading();
  return pressure;
}



/**************************************************************************/
/*!
    @brief Calculates the altitude (in meters).

    Reads the current atmostpheric pressure (in hPa) from the sensor and calculates
    via the provided sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @return Altitude in meters
*/
/**************************************************************************/
float Adafruit_BMP3XX::readAltitude(float seaLevel)
{
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude. See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    float atmospheric = readPressure() / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
    @brief Performs a full reading of all sensors in the BMP3XX.

    Assigns the internal Adafruit_BMP3XX#temperature & Adafruit_BMP3XX#pressure member variables

    @return True on success, False on failure
*/
/**************************************************************************/
bool Adafruit_BMP3XX::performReading(void) {
  int8_t rslt;
  /* Used to select the settings user needs to change */
  uint16_t settings_sel = 0;
  /* Variable used to select the sensor component */
  uint8_t sensor_comp = 0;

  /* Select the pressure and temperature sensor to be enabled */
  the_sensor.settings.temp_en = BMP3_ENABLE;
  settings_sel |= BMP3_TEMP_EN_SEL;
  sensor_comp |= BMP3_TEMP;
  if (_tempOSEnabled) {
     settings_sel |= BMP3_TEMP_OS_SEL;
  }

  the_sensor.settings.press_en = BMP3_ENABLE;
  settings_sel |= BMP3_PRESS_EN_SEL;
  sensor_comp |= BMP3_PRESS;
  if (_presOSEnabled) {
    settings_sel |= BMP3_PRESS_OS_SEL ;
  }

  if (_filterEnabled) {
    settings_sel |= BMP3_IIR_FILTER_SEL;
  }

  if (_ODREnabled) {
    settings_sel |= BMP3_ODR_SEL;
  }

  // set interrupt to data ready
  //settings_sel |= BMP3_DRDY_EN_SEL | BMP3_LEVEL_SEL | BMP3_LATCH_SEL;

  /* Set the desired sensor configuration */
#ifdef BMP3XX_DEBUG
  std::cout << "Setting sensor settings" << std::endl;
#endif
  rslt = bmp3_set_sensor_settings(settings_sel, &the_sensor);
  if (rslt != BMP3_OK)
    return false;

  /* Set the power mode */
  the_sensor.settings.op_mode = BMP3_FORCED_MODE;
#ifdef BMP3XX_DEBUG
  std::cout << "Setting power mode" << std::endl;
#endif
  rslt = bmp3_set_op_mode(&the_sensor);
  if (rslt != BMP3_OK)
    return false;

  /* Variable used to store the compensated data */
  struct bmp3_data data;

  /* Temperature and Pressure data are read and stored in the bmp3_data instance */
  rslt = bmp3_get_sensor_data(sensor_comp, &data, &the_sensor);

  /* Save the temperature and pressure data */
  temperature = data.temperature;
  pressure = data.pressure;
  if (rslt != BMP3_OK)
    return false;

  return true;
}


/**************************************************************************/
/*!
    @brief  Setter for Temperature oversampling
    @param  oversample Oversampling setting, can be BMP3_NO_OVERSAMPLING, BMP3_OVERSAMPLING_2X, BMP3_OVERSAMPLING_4X, BMP3_OVERSAMPLING_8X, BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_32X
    @return True on success, False on failure
*/
/**************************************************************************/

bool Adafruit_BMP3XX::setTemperatureOversampling(uint8_t oversample) {
  if (oversample > BMP3_OVERSAMPLING_32X) return false;

  the_sensor.settings.odr_filter.temp_os = oversample;

  if (oversample == BMP3_NO_OVERSAMPLING)
    _tempOSEnabled = false;
  else
    _tempOSEnabled = true;

  return true;
}


/**************************************************************************/
/*!
    @brief  Setter for Pressure oversampling
    @param  oversample Oversampling setting, can be BMP3_NO_OVERSAMPLING, BMP3_OVERSAMPLING_2X, BMP3_OVERSAMPLING_4X, BMP3_OVERSAMPLING_8X, BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_32X
    @return True on success, False on failure
*/
/**************************************************************************/
bool Adafruit_BMP3XX::setPressureOversampling(uint8_t oversample) {
  if (oversample > BMP3_OVERSAMPLING_32X) return false;

  the_sensor.settings.odr_filter.press_os = oversample;

  if (oversample == BMP3_NO_OVERSAMPLING)
    _presOSEnabled = false;
  else
    _presOSEnabled = true;

  return true;
}

/**************************************************************************/
/*!
    @brief  Setter for IIR filter coefficient
    @param filtercoeff Coefficient of the filter (in samples). Can be BMP3_IIR_FILTER_DISABLE (no filtering), BMP3_IIR_FILTER_COEFF_1, BMP3_IIR_FILTER_COEFF_3, BMP3_IIR_FILTER_COEFF_7, BMP3_IIR_FILTER_COEFF_15, BMP3_IIR_FILTER_COEFF_31, BMP3_IIR_FILTER_COEFF_63, BMP3_IIR_FILTER_COEFF_127
    @return True on success, False on failure

*/
/**************************************************************************/
bool Adafruit_BMP3XX::setIIRFilterCoeff(uint8_t filtercoeff) {
  if (filtercoeff > BMP3_IIR_FILTER_COEFF_127) return false;

  the_sensor.settings.odr_filter.iir_filter = filtercoeff;

  if (filtercoeff == BMP3_IIR_FILTER_DISABLE)
    _filterEnabled = false;
  else
    _filterEnabled = true;

  return true;
}

/**************************************************************************/
/*!
    @brief  Setter for output data rate (ODR)
    @param odr Sample rate in Hz. Can be BMP3_ODR_200_HZ, BMP3_ODR_100_HZ, BMP3_ODR_50_HZ, BMP3_ODR_25_HZ, BMP3_ODR_12_5_HZ, BMP3_ODR_6_25_HZ, BMP3_ODR_3_1_HZ, BMP3_ODR_1_5_HZ, BMP3_ODR_0_78_HZ, BMP3_ODR_0_39_HZ, BMP3_ODR_0_2_HZ, BMP3_ODR_0_1_HZ, BMP3_ODR_0_05_HZ, BMP3_ODR_0_02_HZ, BMP3_ODR_0_01_HZ, BMP3_ODR_0_006_HZ, BMP3_ODR_0_003_HZ, or BMP3_ODR_0_001_HZ 
    @return True on success, False on failure

*/
/**************************************************************************/
bool Adafruit_BMP3XX::setOutputDataRate(uint8_t odr) {
  if (odr > BMP3_ODR_0_001_HZ) return false;

  the_sensor.settings.odr_filter.odr = odr;

  _ODREnabled = true;

  return true;
}

int8_t Adafruit_BMP3XX::i2c_write_wrapper(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  return i2c_write(&_i2cDevice, reg_addr, reg_data, len);
}

int8_t Adafruit_BMP3XX::i2c_read_wrapper(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  return i2c_read(&_i2cDevice, reg_addr, reg_data, len);
}

static void delay_msec(uint32_t ms) {
  usleep(1000*ms);
}
