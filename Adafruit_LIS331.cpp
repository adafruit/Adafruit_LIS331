/*!
 * @file Adafruit_LIS331.cpp
 */

#include "Arduino.h"

#include <Adafruit_LIS331.h>
#include <Wire.h>

/*!
 *  @brief  Instantiates a new LIS331 class in I2C
 *  @param  TheWire
 *          optional wire object
 */
Adafruit_LIS331::Adafruit_LIS331(TwoWire *TheWire) { (void)TheWire; }

/*!
 *   @brief  Instantiates a new LIS331 class using hardware SPI
 *   @param  cspin
 *           number of CSPIN (Chip Select)
 *   @param  *theSPI
 *           optional parameter contains spi object
 */
Adafruit_LIS331::Adafruit_LIS331(int8_t cspin, SPIClass *theSPI) {
  (void)cspin;
  (void)theSPI;
}

/*!
 *   @brief  Instantiates a new LIS331 class using software SPI
 *   @param  cspin
 *           number of CSPIN (Chip Select)
 *   @param  mosipin
 *           number of pin used for MOSI (Master Out Slave In))
 *   @param  misopin
 *           number of pin used for MISO (Master In Slave Out)
 *   @param  sckpin
 *           number of pin used for CLK (clock pin)
 */
Adafruit_LIS331::Adafruit_LIS331(int8_t cspin, int8_t mosipin, int8_t misopin,
                                 int8_t sckpin) {
  (void)cspin;
  (void)mosipin;
  (void)misopin;
  (void)sckpin;
}

/*!
 *  @brief  Get Device ID from LIS331_REG_WHOAMI
 *  @return WHO AM I value
 */
uint8_t Adafruit_LIS331::getDeviceID(void) {
  Adafruit_BusIO_Register _chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_WHOAMI, 1);

  return _chip_id.read();
}

/*!
 *  @brief  Reads x y z values at once
 */
void Adafruit_LIS331::read(void) {

  uint8_t register_address = LIS331_REG_OUT_X_L;
  if (i2c_dev) {
    register_address |= 0x80; // set [7] for auto-increment
  } else {
    register_address |= 0x40; // set [6] for auto-increment
    register_address |= 0x80; // set [7] for read
  }

  Adafruit_BusIO_Register xl_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, register_address, 6);

  uint8_t buffer[6];
  xl_data.read(buffer, 6);

  x = buffer[0];
  x |= ((uint16_t)buffer[1]) << 8;
  y = buffer[2];
  y |= ((uint16_t)buffer[3]) << 8;
  z = buffer[4];
  z |= ((uint16_t)buffer[5]) << 8;

  _scaleValues();
}

/**
 * @brief Setup  the INT1 or INT2 pin to trigger when new data is ready
 *
 * @param irqnum The interrupt number/pin to configure
 * @param activelow The polarity of the pin. true: active low false: active high
 * @param opendrain The pinmode for the given interrupt pin. true: open drain.
 * Connects to GND when activated false: push-pull: connects to VCC when
 * activated
 * @return true
 * @return false
 */
bool Adafruit_LIS331::configIntDataReady(uint8_t irqnum, bool activelow,
                                         bool opendrain) {
  Adafruit_BusIO_Register ctrl3_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_CTRL3, 1);

  Adafruit_BusIO_RegisterBits opendrain_and_polarity_bits =
      Adafruit_BusIO_RegisterBits(&ctrl3_reg, 2, 6);
  Adafruit_BusIO_RegisterBits int1_bits =
      Adafruit_BusIO_RegisterBits(&ctrl3_reg, 2, 0);
  Adafruit_BusIO_RegisterBits int2_bits =
      Adafruit_BusIO_RegisterBits(&ctrl3_reg, 2, 3);
  opendrain_and_polarity_bits.write((activelow << 1) | (opendrain));

  if (irqnum == 1) {
    int1_bits.write(0b10);
    int2_bits.write(0);
  } else {
    int2_bits.write(0b10);
    int1_bits.write(0);
  }

  return true;
}

/**
 * @brief configure the interrupt reset to be latched or not
 *
 * @param irqnum The interrupt number/pin to configure
 * @param latched Latch interrupt request on INT1/2_SRC register, with INT1/2_SRC register cleared by reading INT1/2_SRC register
 */
  void Adafruit_LIS331::setIntLatched(uint8_t irqnum, bool latched){
      Adafruit_BusIO_Register ctrl3_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_CTRL3, 1);

  Adafruit_BusIO_RegisterBits int1_latched_pin =
      Adafruit_BusIO_RegisterBits(&ctrl3_reg, 1, 1);
  Adafruit_BusIO_RegisterBits int2_latched_pin =
      Adafruit_BusIO_RegisterBits(&ctrl3_reg, 1, 5);

  if (irqnum == 1) {
    int1_latched_pin.write(latched);
  } else if(irqnum == 2){
    int2_latched_pin.write(latched);
  }

  }


/**
 * @brief Sets the threshold to fire an interrupt
 *
 * @param irqnum The interrupt number/pin to configure
 * @param threshold The threshold to set
 */
void Adafruit_LIS331::setIntThreshold(uint8_t irqnum, uint16_t threshold) {
  Adafruit_BusIO_Register _ths1 = Adafruit_BusIO_Register(i2c_dev, spi_dev,
      ADDRBIT8_HIGH_TOREAD, LIS331_REG_INT1THS, 1);
  Adafruit_BusIO_Register _ths2 = Adafruit_BusIO_Register(i2c_dev, spi_dev,
      ADDRBIT8_HIGH_TOREAD, LIS331_REG_INT2THS, 1);

  Adafruit_BusIO_RegisterBits ths1_bits = Adafruit_BusIO_RegisterBits(&_ths1,
      7, 0);
  Adafruit_BusIO_RegisterBits ths2_bits = Adafruit_BusIO_RegisterBits(&_ths2,
      7, 0);

  if (irqnum == 1) {
    ths1_bits.write(threshold);
  } else if (irqnum == 2) {
    ths2_bits.write(threshold);
  }

}

/**
 * @brief Sets the duration for wich a condition has to occur to fire an interrupt
 *
 * @param irqnum The interrupt number/pin to configure
 * @param duration the duration to set
 */
void Adafruit_LIS331::setIntDuration(uint8_t irqnum, uint16_t duration) {
  Adafruit_BusIO_Register _dur1 = Adafruit_BusIO_Register(i2c_dev, spi_dev,
      ADDRBIT8_HIGH_TOREAD, LIS331_REG_INT1DUR, 1);
  Adafruit_BusIO_Register _dur2 = Adafruit_BusIO_Register(i2c_dev, spi_dev,
      ADDRBIT8_HIGH_TOREAD, LIS331_REG_INT2DUR, 1);

  Adafruit_BusIO_RegisterBits dur1_bits = Adafruit_BusIO_RegisterBits(&_dur1,
      7, 0);
  Adafruit_BusIO_RegisterBits dur2_bits = Adafruit_BusIO_RegisterBits(&_dur2,
      7, 0);

  if (irqnum == 1) {
    dur1_bits.write(duration);
  } else if (irqnum == 2) {
    dur2_bits.write(duration);
  }
}

/**
 * @brief Configure INT1_CFG or INT2_CFG
 *
 * @param irqnum The interrupt number/pin to configure
 * @param AOI AND/OR combination of Interrupt events
 * @param _6D 6 direction detection function enable
 * @param 6 bits to enable individual interrupt generation on hi/lo threshold events for each axis.
 *        0b00 ZHIE ZLIE YHIE YLIE XHIE XLIE (see datasheet page 30 for LIS331HH)
 */
void Adafruit_LIS331::configInterrupt(uint8_t irqnum, bool AOI, bool _6D, uint8_t axisflags) {
  Adafruit_BusIO_Register _cfg1 = Adafruit_BusIO_Register(i2c_dev, spi_dev,
      ADDRBIT8_HIGH_TOREAD, LIS331_REG_INT1CFG, 1);
  Adafruit_BusIO_Register _cfg2 = Adafruit_BusIO_Register(i2c_dev, spi_dev,
      ADDRBIT8_HIGH_TOREAD, LIS331_REG_INT2CFG, 1);

  Adafruit_BusIO_RegisterBits cfg1_source_bits = Adafruit_BusIO_RegisterBits(
      &_cfg1, 2, 6);
  Adafruit_BusIO_RegisterBits cfg2_source_bits = Adafruit_BusIO_RegisterBits(
      &_cfg2, 2, 6);
  Adafruit_BusIO_RegisterBits cfg1_axis_enable_bits =
      Adafruit_BusIO_RegisterBits(&_cfg1, 6, 0);
  Adafruit_BusIO_RegisterBits cfg2_axis_enable_bits =
      Adafruit_BusIO_RegisterBits(&_cfg2, 6, 0);

  if (irqnum == 1) {
    cfg1_source_bits.write((AOI << 1) | (_6D));
    cfg1_axis_enable_bits.write(axisflags);

  } else if (irqnum == 2) {
    cfg2_source_bits.write((AOI << 1) | (_6D));
    cfg2_axis_enable_bits.write(axisflags);
  }
}


/**
 * @brief read the source of an interrupt (important for latched interrupt generation)
 *
 * @param irqnum The interrupt number/pin to configure
 * @return the content of the src register
 */
uint8_t Adafruit_LIS331::getInterruptSrc(uint8_t irqnum) {
  Adafruit_BusIO_Register _src1 = Adafruit_BusIO_Register(i2c_dev, spi_dev,
      ADDRBIT8_HIGH_TOREAD, LIS331_REG_INT1SRC, 1);
  Adafruit_BusIO_Register _src2 = Adafruit_BusIO_Register(i2c_dev, spi_dev,
      ADDRBIT8_HIGH_TOREAD, LIS331_REG_INT2SRC, 1);

  Adafruit_BusIO_RegisterBits int1_source_bits = Adafruit_BusIO_RegisterBits(
      &_src1, 7, 0);
  Adafruit_BusIO_RegisterBits int2_source_bits = Adafruit_BusIO_RegisterBits(
      &_src2, 7, 0);


  if (irqnum == 1) {
    return (uint8_t) int1_source_bits.read();

  } else if (irqnum == 2) {
    return (uint8_t) int2_source_bits.read();
  } else
    return 0;

}


/**************************************************************************/
/*!
    @brief Enables the high pass filter and/or slope filter
    @param filter_enabled Whether to enable the slope filter (see datasheet)
    @param cutoff The frequency below which signals will be filtered out
    @param use_reference Selects if the reference value set by `setReference`
   should be used

    See section **4** of the LIS331DLH application note for more information

    https://www.st.com/content/ccc/resource/technical/document/application_note/b5/8e/58/69/cb/87/45/55/CD00215823.pdf/files/CD00215823.pdf/jcr:content/translations/en.CD00215823.pdf
*/
void Adafruit_LIS331::enableHighPassFilter(bool filter_enabled,
                                           lis331_hpf_cutoff_t cutoff,
                                           bool use_reference) {
  Adafruit_BusIO_Register ctrl2_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_CTRL2);

  Adafruit_BusIO_RegisterBits HPF_mode =
      Adafruit_BusIO_RegisterBits(&ctrl2_reg, 1, 5);

  Adafruit_BusIO_RegisterBits HPF_internal_filter_en =
      Adafruit_BusIO_RegisterBits(&ctrl2_reg, 1, 4);

  Adafruit_BusIO_RegisterBits HPF_cuttoff =
      Adafruit_BusIO_RegisterBits(&ctrl2_reg, 2, 0);

  if (filter_enabled) {
    HPF_mode.write(use_reference);
    HPF_cuttoff.write(cutoff);
  }
  HPF_internal_filter_en.write(filter_enabled);
}

/**
 * @brief Set the reference value to offset measurements when using the
 High-pass filter.
 *
 * @param reference The offset amount. The conversion of `reference` to milli-g
 depends on the selected range:
 *
 *  * Full scale Reference mode LSB value (mg)
    6g/100g                     ~16mg
    12g/200g                    ~31mg
    24g/400g                    ~63mg
 */
void Adafruit_LIS331::setHPFReference(int8_t reference) {
  Adafruit_BusIO_Register reference_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_REFERENCE);
  reference_reg.write(reference);
}
/**
 * @brief Gets the current high-pass filter reference/offset
 * @return int8_t The current reference value.
 * See `enableHighPassFilter` and `setHPFReference` for more information.
 */
int8_t Adafruit_LIS331::getHPFReference(void) {
  Adafruit_BusIO_Register reference_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_REFERENCE);
  return reference_reg.read();
}

/**
 * @brief Zero the measurement offsets while the high-pass filter is enabled
 when not using a reference.
 *
 * See `enableHighPassFilter` and `setHPFReference` for more information.
 */
void Adafruit_LIS331::HPFReset(void) {
  Adafruit_BusIO_Register reference_reset_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_HP_FILTER_RESET);
  reference_reset_reg.read();
}

/**
 * @brief Scale the acceleration measuremets from their raw value to milli-g
 * depending on the current measurement range
 *
 */
void Adafruit_LIS331::_scaleValues(void) {}

/**
 * @brief Set the measurement range for the sensor
 *
 * @param range The measurement range to set
 */
void Adafruit_LIS331::writeRange(uint8_t range) {

  Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_CTRL4, 1);

  Adafruit_BusIO_RegisterBits range_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl4, 2, 4);
  range_bits.write(range);
  delay(15); // delay to let new setting settle
}

/**
 * @brief Get the measurement range
 *
 * @return uint8_t The measurement range
 */
uint8_t Adafruit_LIS331::readRange(void) {
  Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_CTRL4, 1);

  Adafruit_BusIO_RegisterBits range_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl4, 2, 4);

  return (uint8_t)range_bits.read();
}

/******** Power mode, data rate, and Low-pass filter methods ***********/
/*!
 *  @brief  Sets the data rate for the LIS331 (affects power consumption)
 *  @param  data_rate The new data rate to set.
 */
void Adafruit_LIS331::setDataRate(lis331_data_rate_t data_rate) {
  int8_t dr_value = 0;
  int8_t pm_value = 0;

  lis331_mode_t new_mode = getMode(data_rate);
  Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_CTRL1, 1);
  Adafruit_BusIO_RegisterBits pm_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl1, 3, 5);

  switch (new_mode) {
  case LIS331_MODE_SHUTDOWN:
    break;

  case LIS331_MODE_LOW_POWER: // ODR bits are in CTRL1[7:5] (PM)
    pm_value = ((data_rate & 0x1C)) >> 2;
    break;

  case LIS331_MODE_NORMAL: // ODR bits are in CTRL1[4:3] (DR)
    pm_value = ((data_rate & 0x1C)) >> 2;
    dr_value = (data_rate & 0x7);

    // only Normal mode uses DR to set ODR, so we can set it here
    Adafruit_BusIO_RegisterBits dr_bits =
        Adafruit_BusIO_RegisterBits(&_ctrl1, 2, 3);
    dr_bits.write(dr_value);
    break;
  }

  pm_bits.write(pm_value);
}

/*!
 *   @brief  Gets the data rate for the LIS331 (affects power consumption)
 *   @return Returns Data Rate value
 */
lis331_data_rate_t Adafruit_LIS331::getDataRate(void) {
  Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_CTRL1, 1);
  Adafruit_BusIO_RegisterBits pm_dr_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl1, 5, 3);
  return (lis331_data_rate_t)pm_dr_bits.read();
}

/**
 * @brief  Return the current power mode from the current data rate
 * @return lis331_mode_t The currently set power mode
 */
lis331_mode_t Adafruit_LIS331::getMode(void) {
  lis331_data_rate_t current_rate = getDataRate();
  return getMode(current_rate);
}

/**
 * @brief Return the current power mode from a given data rate value
 *
 * @param data_rate The `lis331_data_rate_t` to return the `lis331_mode_t` for
 * @return lis331_mode_t
 */
lis331_mode_t Adafruit_LIS331::getMode(lis331_data_rate_t data_rate) {
  uint8_t pm_value = (data_rate & 0x1C) >> 2;
  if (pm_value >= LIS331_MODE_LOW_POWER) {
    return LIS331_MODE_LOW_POWER;
  }
  return (lis331_mode_t)pm_value;
}
/**
 * @brief Set the Low Pass Filter cutoff frequency. Useful for removing high
 * frequency noise while sensing orientation using the acceleration from
 * gravity.
 *
 * **Will not work** when sensor is **in Normal mode** because the LPF cutoff
 * bits are used to set the ODR while in Normal mode.
 *
 * @param cutoff The frequency above which signals will be ignored.
 * @returns true: success false: cuttoff frequency was not set because the
 */
bool Adafruit_LIS331::setLPFCutoff(lis331_lpf_cutoff_t cutoff) {

  lis331_mode_t current_mode = getMode();
  if (current_mode == LIS331_MODE_NORMAL) {
    return false;
  }
  Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS331_REG_CTRL1, 1);
  Adafruit_BusIO_RegisterBits data_rate_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl1, 2, 3); // including LPen bit

  data_rate_bits.write(cutoff);
  return true;
}

/************************************************************************/
/*!
 *  @brief  Gets the most recent sensor event
 *  @param  *event
 *          sensor event that we want to read
 *  @return true if successful
 */
bool Adafruit_LIS331::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;

  read();

  event->acceleration.x = x_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = y_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = z_g * SENSORS_GRAVITY_STANDARD;

  return true;
}

/*!
 *   @brief  Gets the sensor_t data
 *   @param  *sensor
 *           sensor that we want to write data into
 */
void Adafruit_LIS331::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "LIS331", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->max_value = 0;
  sensor->min_value = 0;
  sensor->resolution = 0;
}
