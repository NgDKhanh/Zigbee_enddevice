/***************************************************************************//**
 * @file
 * @brief I2C simple poll-based master mode driver instance initialilization
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "sl_i2cspm.h"
#include "em_cmu.h"
// Include instance config 
#include "sl_i2cspm_inst0_config.h"
#include "sl_i2cspm_mikroe_config.h"

sl_i2cspm_t *sl_i2cspm_inst0 = SL_I2CSPM_INST0_PERIPHERAL;
sl_i2cspm_t *sl_i2cspm_mikroe = SL_I2CSPM_MIKROE_PERIPHERAL;

#if SL_I2CSPM_INST0_SPEED_MODE == 0
#define SL_I2CSPM_INST0_HLR i2cClockHLRStandard
#define SL_I2CSPM_INST0_MAX_FREQ I2C_FREQ_STANDARD_MAX
#elif SL_I2CSPM_INST0_SPEED_MODE == 1
#define SL_I2CSPM_INST0_HLR i2cClockHLRAsymetric
#define SL_I2CSPM_INST0_MAX_FREQ I2C_FREQ_FAST_MAX
#elif SL_I2CSPM_INST0_SPEED_MODE == 2
#define SL_I2CSPM_INST0_HLR i2cClockHLRFast
#define SL_I2CSPM_INST0_MAX_FREQ I2C_FREQ_FASTPLUS_MAX
#endif

#if SL_I2CSPM_MIKROE_SPEED_MODE == 0
#define SL_I2CSPM_MIKROE_HLR i2cClockHLRStandard
#define SL_I2CSPM_MIKROE_MAX_FREQ I2C_FREQ_STANDARD_MAX
#elif SL_I2CSPM_MIKROE_SPEED_MODE == 1
#define SL_I2CSPM_MIKROE_HLR i2cClockHLRAsymetric
#define SL_I2CSPM_MIKROE_MAX_FREQ I2C_FREQ_FAST_MAX
#elif SL_I2CSPM_MIKROE_SPEED_MODE == 2
#define SL_I2CSPM_MIKROE_HLR i2cClockHLRFast
#define SL_I2CSPM_MIKROE_MAX_FREQ I2C_FREQ_FASTPLUS_MAX
#endif

I2CSPM_Init_TypeDef init_inst0 = { 
  .port = SL_I2CSPM_INST0_PERIPHERAL,
  .sclPort = SL_I2CSPM_INST0_SCL_PORT,
  .sclPin = SL_I2CSPM_INST0_SCL_PIN,
  .sdaPort = SL_I2CSPM_INST0_SDA_PORT,
  .sdaPin = SL_I2CSPM_INST0_SDA_PIN,
  .i2cRefFreq = 0,
  .i2cMaxFreq = SL_I2CSPM_INST0_MAX_FREQ,
  .i2cClhr = SL_I2CSPM_INST0_HLR
};

I2CSPM_Init_TypeDef init_mikroe = { 
  .port = SL_I2CSPM_MIKROE_PERIPHERAL,
  .sclPort = SL_I2CSPM_MIKROE_SCL_PORT,
  .sclPin = SL_I2CSPM_MIKROE_SCL_PIN,
  .sdaPort = SL_I2CSPM_MIKROE_SDA_PORT,
  .sdaPin = SL_I2CSPM_MIKROE_SDA_PIN,
  .i2cRefFreq = 0,
  .i2cMaxFreq = SL_I2CSPM_MIKROE_MAX_FREQ,
  .i2cClhr = SL_I2CSPM_MIKROE_HLR
};

void sl_i2cspm_init_instances(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  I2CSPM_Init(&init_inst0);
  I2CSPM_Init(&init_mikroe);
}
