/*
 * i2c_scan.h
 *
 *  Created on: 15 Jun 2023
 *      Author: admin
 */

#ifndef I2C_SCAN_H_
#define I2C_SCAN_H_

#include "sl_i2cspm_instances.h"

typedef struct bme280 {
  float temperature;
  float humidity;
  float pressure;
} sl_bme280_dev;

void i2c_scan_init(sl_i2cspm_t *i2cspm);

void i2c_scan_process(sl_bme280_dev *dev);


#endif /* I2C_SCAN_H_ */
