/*
 * pms7003.h
 *
 *  Created on: 29 Jun 2023
 *      Author: admin
 */

#ifndef PMS7003_H_
#define PMS7003_H_

#include <stdint.h>

typedef enum Mode {
  indoor = 0,
  outdoor = 1,
}pms_mode;

typedef struct pms {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
  pms_mode mode;
} pms_dev;

void pms7003_init(pms_dev *dev, pms_mode mode);

void pms7003_process(pms_dev *dev);

#endif /* PMS7003_H_ */
