/*
 * bh1750.c
 *
 *  Created on: 29 Jul 2023
 *      Author: admin
 */

#include "bh1750.h"
#include <stddef.h>
#include "sl_sleeptimer.h"

#define OPCODE_HIGH  0x0
#define OPCODE_HIGH2 0x1
#define OPCODE_LOW   0x3

#define OPCODE_CONT 0x10
#define OPCODE_OT   0x20

#define OPCODE_POWER_DOWN 0x00
#define OPCODE_POWER_ON   0x01
#define OPCODE_MT_HI      0x40
#define OPCODE_MT_LO      0x60

/***************************************************************************//**
 * Local Variables
 ******************************************************************************/
static sl_i2cspm_t *bh1750_i2cspm_instance;

/***************************************************************************//**
 * Local Function Prototypes
 ******************************************************************************/
static int8_t i2c_bus_read_nolock(uint8_t devAddr, uint8_t *Data, uint8_t count);
static int8_t i2c_bus_write_nolock(uint8_t devAddr, uint8_t *Data, uint8_t count);

/***************************************************************************//**
 *    Initializes the barometric pressure sensor chip
 ******************************************************************************/
sl_status_t bh1750_init(sl_i2cspm_t *i2cspm)
{
  /* The device needs 2 ms startup time */
  sl_sleeptimer_delay_millisecond(2);

  /* Update i2cspm instance */
  bh1750_i2cspm_instance = i2cspm;

  return SL_STATUS_OK;
}

/***************************************************************************//**
 *    Configure the light sensor
 ******************************************************************************/
sl_status_t bh1750_configure(sl_bh1750_config cfg)
{
  int8_t result;

  result = 0;

  uint8_t opcode = cfg.mode == BH1750_MODE_CONTINUOUS ? OPCODE_CONT : OPCODE_OT;
  switch (cfg.resolution)
  {
      case BH1750_RES_LOW:  opcode |= OPCODE_LOW;   break;
      case BH1750_RES_HIGH: opcode |= OPCODE_HIGH;  break;
      default:              opcode |= OPCODE_HIGH2; break;
  }

  result = i2c_bus_write_nolock(cfg.address, &opcode, 1);

  if (result != 0) {
    return SL_STATUS_FAIL;
  }
  return SL_STATUS_OK;
}

sl_status_t bh1750_measure_illuminance(sl_bh1750_dev* dev, sl_bh1750_config cfg)
{
  uint8_t buf[2];
  if (i2c_bus_read_nolock(cfg.address, buf, 2) != 0){
      return SL_STATUS_FAIL;
  }
  dev->level = buf[0] << 8 | buf[1];
  dev->level = (dev->level * 10) / 12; // convert to LUX
  return SL_STATUS_OK;
}

sl_status_t bh1750_power_down(sl_bh1750_config cfg)
{
  uint8_t opcode = OPCODE_POWER_DOWN;
  if (i2c_bus_write_nolock(cfg.address, &opcode, 1) != 0) {
      return SL_STATUS_FAIL;
  }
  return SL_STATUS_OK;
}

sl_status_t bh1750_power_on(sl_bh1750_config cfg)
{
  uint8_t opcode = OPCODE_POWER_ON;
  if (i2c_bus_write_nolock(cfg.address, &opcode, 1) != 0) {
      return SL_STATUS_FAIL;
  }
  return SL_STATUS_OK;
}

sl_status_t bh1750_set_measurement_time(sl_bh1750_config cfg, uint8_t time)
{
  uint8_t opcode;
  opcode = OPCODE_MT_HI | (time >> 5);
  if (i2c_bus_write_nolock(cfg.address, &opcode, 1) != 0) {
        return SL_STATUS_FAIL;
  }
  opcode = OPCODE_MT_LO | (time & 0x1f);
  if (i2c_bus_write_nolock(cfg.address, &opcode, 1) != 0) {
          return SL_STATUS_FAIL;
  }
  return SL_STATUS_OK;
}

/***************************************************************************//**
 * @brief
 *    Writes data to a device via the I2C bus
 *
 * @param[in] Data
 *    The data to write
 *
 * @param[in] count
 *    The number of data bytes to write
 *
 * @return
 *    *    Returns zero on OK, -1 on failure.
 ******************************************************************************/
static int8_t i2c_bus_write_nolock(uint8_t devAddr, uint8_t *Data, uint8_t count)
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;

  seq.addr = devAddr << 1;
  seq.flags = I2C_FLAG_WRITE;

  seq.buf[0].len  = count;
  seq.buf[0].data = Data;

  ret = I2CSPM_Transfer(bh1750_i2cspm_instance, &seq);
  if (ret != i2cTransferDone) {
    return -1;
  }

  return 0;
}

/***************************************************************************//**
 * @brief
 *    Reads data from a device via the I2C bus
 *
 * @param[in] devAddr
 *    The I2C address of the device
 *
 * @param[out] Data
 *    The data read from the device
 *
 * @param[in] count
 *    The number of data bytes to read
 *
 * @return
 *    Returns zero on OK, -1 on failure.
 ******************************************************************************/
static int8_t i2c_bus_read_nolock(uint8_t devAddr, uint8_t *Data, uint8_t count)
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;

  seq.addr = devAddr << 1;
  seq.flags = I2C_FLAG_READ;

  seq.buf[0].len  = count;
  seq.buf[0].data = Data;

  ret = I2CSPM_Transfer(bh1750_i2cspm_instance, &seq);

  if (ret != i2cTransferDone) {
    return -1;
  }

  return 0;
}
