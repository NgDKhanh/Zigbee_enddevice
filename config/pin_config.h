#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// $[CMU]
// [CMU]$

// $[LFXO]
// [LFXO]$

// $[PRS.ASYNCH0]
// [PRS.ASYNCH0]$

// $[PRS.ASYNCH1]
// [PRS.ASYNCH1]$

// $[PRS.ASYNCH2]
// [PRS.ASYNCH2]$

// $[PRS.ASYNCH3]
// [PRS.ASYNCH3]$

// $[PRS.ASYNCH4]
// [PRS.ASYNCH4]$

// $[PRS.ASYNCH5]
// [PRS.ASYNCH5]$

// $[PRS.ASYNCH6]
// [PRS.ASYNCH6]$

// $[PRS.ASYNCH7]
// [PRS.ASYNCH7]$

// $[PRS.ASYNCH8]
// [PRS.ASYNCH8]$

// $[PRS.ASYNCH9]
// [PRS.ASYNCH9]$

// $[PRS.ASYNCH10]
// [PRS.ASYNCH10]$

// $[PRS.ASYNCH11]
// [PRS.ASYNCH11]$

// $[PRS.ASYNCH12]
// [PRS.ASYNCH12]$

// $[PRS.ASYNCH13]
// [PRS.ASYNCH13]$

// $[PRS.ASYNCH14]
// [PRS.ASYNCH14]$

// $[PRS.ASYNCH15]
// [PRS.ASYNCH15]$

// $[PRS.SYNCH0]
// [PRS.SYNCH0]$

// $[PRS.SYNCH1]
// [PRS.SYNCH1]$

// $[PRS.SYNCH2]
// [PRS.SYNCH2]$

// $[PRS.SYNCH3]
// [PRS.SYNCH3]$

// $[GPIO]
// GPIO SWV on PA03
#define GPIO_SWV_PORT                            gpioPortA
#define GPIO_SWV_PIN                             3

// [GPIO]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// [TIMER1]$

// $[TIMER2]
// [TIMER2]$

// $[TIMER3]
// [TIMER3]$

// $[TIMER4]
// [TIMER4]$

// $[USART0]
// USART0 CTS on PA09
#define USART0_CTS_PORT                          gpioPortA
#define USART0_CTS_PIN                           9

// USART0 RTS on PA08
#define USART0_RTS_PORT                          gpioPortA
#define USART0_RTS_PIN                           8

// USART0 RX on PA06
#define USART0_RX_PORT                           gpioPortA
#define USART0_RX_PIN                            6

// USART0 TX on PA05
#define USART0_TX_PORT                           gpioPortA
#define USART0_TX_PIN                            5

// [USART0]$

// $[I2C1]
// [I2C1]$

// $[EUSART1]
// EUSART1 RX on PD05
#define EUSART1_RX_PORT                          gpioPortD
#define EUSART1_RX_PIN                           5

// EUSART1 TX on PD04
#define EUSART1_TX_PORT                          gpioPortD
#define EUSART1_TX_PIN                           4

// [EUSART1]$

// $[KEYSCAN]
// [KEYSCAN]$

// $[LETIMER0]
// [LETIMER0]$

// $[ACMP0]
// [ACMP0]$

// $[ACMP1]
// [ACMP1]$

// $[VDAC0]
// [VDAC0]$

// $[VDAC1]
// [VDAC1]$

// $[PCNT0]
// [PCNT0]$

// $[HFXO0]
// [HFXO0]$

// $[I2C0]
// I2C0 SCL on PB04
#define I2C0_SCL_PORT                            gpioPortB
#define I2C0_SCL_PIN                             4

// I2C0 SDA on PB05
#define I2C0_SDA_PORT                            gpioPortB
#define I2C0_SDA_PIN                             5

// [I2C0]$

// $[EUSART0]
// [EUSART0]$

// $[PTI]
// PTI DFRAME on PC07
#define PTI_DFRAME_PORT                          gpioPortC
#define PTI_DFRAME_PIN                           7

// PTI DOUT on PC06
#define PTI_DOUT_PORT                            gpioPortC
#define PTI_DOUT_PIN                             6

// [PTI]$

// $[MODEM]
// [MODEM]$

// $[CUSTOM_PIN_NAME]
// [CUSTOM_PIN_NAME]$

#endif // PIN_CONFIG_H

