/*
 * i2c_scan.c
 *
 *  Created on: 15 Jun 2023
 *      Author: admin
 */

#include "sl_i2cspm_instances.h"
#include "sl_bmp280.h"
#include "sl_status.h"
#include "stdio.h"
#include "i2c_scan.h"

void i2c_scan_init(sl_i2cspm_t *i2cspm) {
  sl_status_t status = sl_bmp280_init(i2cspm);
  switch (status) {
    case SL_STATUS_NOT_FOUND:
      printf("Not found BME280");
      break;
    case SL_STATUS_OK:
      printf("Init OK");
      break;
    default:
      printf("Init Fail");
      break;
  }
}

void i2c_scan_process(sl_bme280_dev *bme) {
  float temperature;
  float pressure;
  float humidity;
  sl_status_t status = sl_bmp280_measure_temperature(sl_i2cspm_mikroe, &temperature) |
                       sl_bmp280_measure_pressure(sl_i2cspm_mikroe, &pressure)       |
                       sl_bmp280_measure_humidity(sl_i2cspm_mikroe, &humidity);
  bme->temperature = temperature;
  bme->humidity = humidity;
  bme->pressure = pressure;
  if (status == SL_STATUS_OK) {
      printf(" Temp: %.2f, Press: %.2f, Hum = %.2f\n\r", bme->temperature, bme->pressure, bme->humidity);
  }
  else {
      printf(" Read fail!");
  }
}


