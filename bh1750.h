/*
 * bh1750.h
 *
 *  Created on: 29 Jul 2023
 *      Author: admin
 */

#ifndef BH1750_H_
#define BH1750_H_

#include <stdint.h>
#include "sl_i2cspm.h"
#include "sl_status.h"

#define BH1750_ADDR_LO 0x23 //!< I2C address when ADDR pin floating/low
#define BH1750_ADDR_HI 0x5c //!< I2C address when ADDR pin high

/**
 * Measurement mode
 */
typedef enum
{
    BH1750_MODE_ONE_TIME = 0, //!< One time measurement
    BH1750_MODE_CONTINUOUS    //!< Continuous measurement
} bh1750_mode_t;

/**
 * Measurement resolution
 */
typedef enum
{
    BH1750_RES_LOW = 0,  //!< 4 lx resolution, measurement time is usually 16 ms
    BH1750_RES_HIGH,     //!< 1 lx resolution, measurement time is usually 120 ms
    BH1750_RES_HIGH2     //!< 0.5 lx resolution, measurement time is usually 120 ms
} bh1750_resolution_t;

typedef struct bh1750_config
{
  uint8_t address;
  bh1750_resolution_t resolution;
  bh1750_mode_t mode;
  uint8_t time;
} sl_bh1750_config;

typedef struct bh1750
{
  uint16_t level;
} sl_bh1750_dev;


/***************************************************************************//**
 * @brief
 *    Initialize the barometric pressure sensor chip.
 *
 * @param[in] i2cspm
 *   The I2CSPM instance to use.
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_FAIL Initialization failed
 ******************************************************************************/
sl_status_t bh1750_init(sl_i2cspm_t *i2cspm);

/***************************************************************************//**
 * @brief
 *    Configure the barometric pressure sensor.
 *
 * @param[in] cfg
 *    Structure, which holds the configuration parameters
 *
 * @note
 *   Make sure to pass by Sleep Mode before switching between Forced mode
 *   and normal mode. Refer to datasheet for more details.
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_FAIL Sensor configuration failed
 ******************************************************************************/
sl_status_t bh1750_configure(sl_bh1750_config cfg);

/***************************************************************************//**
 * @brief
 *    Power down the light sensor.
 *
 * @param[in] cfg
 *    Structure, which holds the configuration parameters
 *
 * @note
 *   Make sure to pass by Sleep Mode before switching between Forced mode
 *   and normal mode. Refer to datasheet for more details.
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_FAIL Sensor configuration failed
 ******************************************************************************/
sl_status_t bh1750_power_down(sl_bh1750_config cfg);

/***************************************************************************//**
 * @brief
 *    Power on the light sensor.
 *
 * @param[in] cfg
 *    Structure, which holds the configuration parameters
 *
 * @note
 *   Make sure to pass by Sleep Mode before switching between Forced mode
 *   and normal mode. Refer to datasheet for more details.
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_FAIL Sensor configuration failed
 ******************************************************************************/
sl_status_t bh1750_power_on(sl_bh1750_config cfg);

/***************************************************************************//**
 * @brief
 *    Set measurement time the light sensor.
 *
 * @param[in] cfg
 *    Structure, which holds the configuration parameters
 *
 * @note
 *   Make sure to pass by Sleep Mode before switching between Forced mode
 *   and normal mode. Refer to datasheet for more details.
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_FAIL Sensor configuration failed
 ******************************************************************************/
sl_status_t bh1750_set_measurement_time(sl_bh1750_config cfg, uint8_t time);

/***************************************************************************//**
 * @brief
 *    Initiate  illuminance measurement on the light sensor bh1750
 * @param[in] dev
 *   The sl_bh1750_dev instance to use.
 *
 * @param[out]  illuminance
 *    The measured  illuminance in degrees lux
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_FAIL Failure during  illuminance measurement/read
 ******************************************************************************/
sl_status_t bh1750_measure_illuminance(sl_bh1750_dev* dev, sl_bh1750_config cfg);

#endif /* BH1750_H_ */
