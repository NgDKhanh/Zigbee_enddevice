/*
 * pms7003.c
 *
 *  Created on: 30 Jun 2023
 *      Author: admin
 */

#include "pms7003.h"
#include "em_chip.h"
#include "sl_sleeptimer.h"
#include "sl_iostream.h"
#include "sl_iostream_init_instances.h"
#include "sl_iostream_handles.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

#ifndef BUFSIZE
#define BUFSIZE    128
#endif

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/


/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

uint8_t buffer[BUFSIZE];

/***************************************************************************//**
 * Initialize example.
 ******************************************************************************/

void receive(uint8_t *resp, uint16_t len)
{
  uint64_t startTime = sl_sleeptimer_get_tick_count64();
  uint8_t index = 0;
  while ((index < len) && (sl_sleeptimer_get_tick_count64() - startTime < 1000))
  {
    char c;
    sl_iostream_getchar(sl_iostream_mikroe_handle, &c);
    resp[index++] = (uint8_t)c;
  }
}

uint8_t read_pms(pms_dev *dev){
  memset(buffer, 0, BUFSIZE);
  receive(buffer, BUFSIZE);
  for (size_t i = 0; i < 104; i++) {
      if (buffer[i] == 0x42 && buffer[i+1] == 0x4d) {
          uint8_t startByte = i;
          startByte += (dev->mode == indoor) ? 4 : 10;
          dev->pm1_0 = ((buffer[startByte] << 8) + buffer[startByte + 1]);
          dev->pm2_5 = ((buffer[startByte + 2] << 8) + buffer[startByte + 3]);
          dev->pm10 = ((buffer[startByte + 4] << 8) + buffer[startByte + 5]);
          return true;
      }
  }
  return false;
}

void pms7003_init(pms_dev *dev, pms_mode mode)
{
  /* Output on vcom usart instance */
  //const char str1[] = "PMS7003 init\n\r";
  //sl_iostream_write(sl_iostream_vcom_handle, str1, strlen(str1));
  printf("PMS7003 init\n\r");
  dev->mode = mode;

  int i = 0;
  while ((read_pms (dev)) != true && i < 10)
  {
    printf ("Read PMS fail!\n\r");
    i++;
    sl_sleeptimer_delay_millisecond (100);
  }
  printf(" pm1 = %d, pm2.5 = %d, pm10 = %d\n\r", dev->pm1_0, dev->pm2_5, dev->pm10);
}

/***************************************************************************//**
 * Example ticking function.
 ******************************************************************************/

void pms7003_process(pms_dev *dev)
{
  if (read_pms(dev)) {
      printf(" pm1 = %d, pm2.5 = %d, pm10 = %d\n\r", dev->pm1_0, dev->pm2_5, dev->pm10);
  }
  else {
      printf("Read PMS fail!\n\r");
  }
  //sl_sleeptimer_delay_millisecond(5000);
}
