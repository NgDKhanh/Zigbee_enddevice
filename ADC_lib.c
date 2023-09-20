/*
 * ADC_lib.c
 *
 *  Created on: 12 Aug 2023
 *      Author: admin
 */

#include <stdio.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_iadc.h"
#include "sl_component_catalog.h"
#include "sl_system_init.h"
#include "ADC_lib.h"


/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

// Set CLK_ADC to 40 MHz - this will be adjusted to HFXO frequency in the
// initialization process
#define CLK_SRC_ADC_FREQ        40000000  // CLK_SRC_ADC - 40 MHz max
#define CLK_ADC_FREQ            10000000  // CLK_ADC - 10 MHz max in normal mode

// Number of scan channels
#define NUM_INPUTS 2


/*
 * Specify the IADC input using the IADC_PosInput_t typedef.  This
 * must be paired with a corresponding macro definition that allocates
 * the corresponding ABUS to the IADC.  These are...
 *
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AEVEN0_ADC0
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BEVEN0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BODD0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDEVEN0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDODD0_ADC0
 *
 * ...for port A, port B, and port C/D pins, even and odd, respectively.
 */
#define IADC_INPUT_0_PORT_PIN     iadcPosInputPortBPin0;
#define IADC_INPUT_1_PORT_PIN     iadcPosInputPortBPin1;

#define IADC_INPUT_0_BUS          BBUSALLOC
#define IADC_INPUT_0_BUSALLOC     GPIO_BBUSALLOC_BEVEN0_ADC0
#define IADC_INPUT_1_BUS          BBUSALLOC
#define IADC_INPUT_1_BUSALLOC     GPIO_BBUSALLOC_BODD0_ADC0

// GPIO output toggle to notify IADC conversion complete
#define GPIO_OUTPUT_0_PORT        gpioPortC
#define GPIO_OUTPUT_0_PIN         5

/*******************************************************************************
***************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/

static volatile double scanResult[NUM_INPUTS];

/**************************************************************************//**
 * @brief  GPIO initialization
 *****************************************************************************/
void initGPIO(void)
{
  /*
   * On EFR32xG21 devices, CMU_ClockEnable() calls have no effect as
   * clocks are enabled/disabled on-demand in response to peripheral
   * requests.  Deleting such lines is safe on xG21 devices and will
   * provide a small reduction in code size.
   */
  // CMU_ClockEnable(cmuClock_GPIO, true);

  // Output toggled upon completing each IADC scan sequence
  GPIO_PinModeSet(GPIO_OUTPUT_0_PORT, GPIO_OUTPUT_0_PIN, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief  IADC initialization
 *****************************************************************************/
void initIADC(void)
{
  // Declare initialization structures
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;

  // Scan table structure
  IADC_ScanTable_t scanTable = IADC_SCANTABLE_DEFAULT;

  CMU_ClockEnable(cmuClock_IADC0, true);

  // Use the EM01GRPACLK as the IADC clock
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_EM01GRPACLK);

  // Shutdown between conversions to reduce current
  init.warmup = iadcWarmupNormal;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  /*
   * The IADC local timer runs at CLK_SRC_ADC_FREQ, which is at least
   * 2x CLK_ADC_FREQ. CLK_SRC_ADC_FREQ in this example is equal to the
   * HFXO frequency. Dividing the frequency of the HFXO by 1000 will give
   * the tick count for 1 ms trigger rate.
   * For example - if HFXO freq = 38.4 MHz,
   *
   * ticks for 1 ms trigger = 38400000 / 1000
   * ticks =  38400
   */
  init.timerCycles = CMU_ClockFreqGet(cmuClock_IADCCLK)/1000;

  /*
   * Configuration 0 is used by both scan and single conversions by
   * default.  Use internal bandgap as the reference and specify the
   * reference voltage in mV.
   *
   * Resolution is not configurable directly but is based on the
   * selected oversampling ratio (osrHighSpeed), which defaults to
   * 2x and generates 12-bit results.
   */
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;
  initAllConfigs.configs[0].vRef = 1650;
  initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x;
  initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain1x;

  /*
   * CLK_SRC_ADC must be prescaled by some value greater than 1 to
   * derive the intended CLK_ADC frequency.
   *
   * Based on the default 2x oversampling rate (OSRHS)...
   *
   * conversion time = ((4 * OSRHS) + 2) / fCLK_ADC
   *
   * ...which results in a maximum sampling rate of 833 ksps with the
   * 2-clock input multiplexer switching time is included.
   */
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                     CLK_ADC_FREQ,
                                                                     0,
                                                                     iadcCfgModeNormal,
                                                                     init.srcClkPrescale);

  /*
   * The IADC local timer triggers conversions.
   *
   * Set the SCANFIFODVL flag when scan FIFO holds 2 entries.  In this
   * example, the interrupt associated with the SCANFIFODVL flag in
   * the IADC_IF register is not used.
   *
   * Tag each FIFO entry with scan table entry ID.
   */
  initScan.triggerSelect = iadcTriggerSelTimer;
  initScan.dataValidLevel = iadcFifoCfgDvl2;
  initScan.showId = true;

  /*
   * Configure entries in the scan table.  CH0 is single-ended from
   * input 0; CH1 is single-ended from input 1.
   */
  scanTable.entries[0].posInput = IADC_INPUT_0_PORT_PIN;
  scanTable.entries[0].negInput = iadcNegInputGnd;
  scanTable.entries[0].includeInScan = true;

  scanTable.entries[1].posInput = IADC_INPUT_1_PORT_PIN;
  scanTable.entries[1].negInput = iadcNegInputGnd;
  scanTable.entries[1].includeInScan = true;

  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize scan
  IADC_initScan(IADC0, &initScan, &scanTable);

  // Enable the IADC timer (must be after the IADC is initialized)
  IADC_command(IADC0, iadcCmdEnableTimer);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
  GPIO->IADC_INPUT_1_BUS |= IADC_INPUT_1_BUSALLOC;

  // Enable scan interrupts
  IADC_enableInt(IADC0, IADC_IEN_SCANFIFODVL);

  // Enable ADC interrupts
  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);
}

void ADC_init(void)
{
  initGPIO();
  initIADC();
  // Start scan
  IADC_command(IADC0, iadcCmdStartScan);
}

/**************************************************************************//**
 * @brief  IADC interrupt handler
 *****************************************************************************/
void IADC_IRQHandler(void)
{
  IADC_Result_t sample;

  // Toggle GPIO to signal scan completion
  GPIO_PinOutToggle(GPIO_OUTPUT_0_PORT, GPIO_OUTPUT_0_PIN);

  // While the FIFO count is non-zero...
  while (IADC_getScanFifoCnt(IADC0))
  {
    // Pull a scan result from the FIFO
    sample = IADC_pullScanFifoResult(IADC0);

    /*
     * Calculate the voltage converted as follows:
     *
     * For single-ended conversions, the result can range from 0 to
     * +Vref, i.e., for Vref = VBGR = 1.65V, and with analog gain = 1,
     * 0xFFF represents the full scale value of 3.3V.
     */
    scanResult[sample.id] = sample.data * 3.3 / 0xFFF;
  }

  /*
   * Clear the scan table complete interrupt.  Reading from the FIFO
   * does not do this automatically.
   */
  IADC_clearInt(IADC0, IADC_IF_SCANFIFODVL);
}

void ADC_read(float *measured_uv)
{
  float uv = (scanResult[0] - 0.99) * 15.0 / 1.81;
  uv = (uv >= 0) ? uv : 0;
  *measured_uv = uv;
  printf("UV: %f\n\r", *measured_uv);
}

void ADC_read_bat(float *measured_bat)
{
  float bat = scanResult[1] / 3.3 * 100;
  *measured_bat = bat;
  printf ("Volt: %f, Battery: %f\n\r",scanResult[1], *measured_bat);
}

