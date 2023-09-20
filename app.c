/***************************************************************************//**
 * @file app.c
 * @brief Callbacks implementation and application specific code.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
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

#include "app/framework/include/af.h"
#include "network-steering.h"
#include "find-and-bind-initiator.h"
#include <stdint.h>
#include "sl_i2cspm_instances.h"
#include "i2c_scan.h"
#include "pms7003.h"
#include "bh1750.h"
#include "ADC_lib.h"

#include "em_gpio.h"

#define MANUFACTURE_CODE             0x1002

#define REPORT_PERIOD_MS             (60000)

#define LED_BLINK_PERIOD_MS          1000

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
#define RHT_MEASUREMENT_ENDPOINT    1  // Endpoint with the RH and Temp ZCL
#define FIND_AND_BIND_DELAY_MS      3000 // Delay for find and bind handler

static void network_steering_event_handler(sl_zigbee_event_t *event);
static void finding_and_binding_event_handler(sl_zigbee_event_t *event);
static void leave_network_event_handler(sl_zigbee_event_t *event);
static void attribute_report_event_handler(sl_zigbee_event_t *event);
static void sensor_init_event_handler(sl_zigbee_event_t *event);
static uint8_t binding_table_unicast_binding_count(void);
static void binding_table_print(void);

static sl_zigbee_event_t network_steering_event_control; // Custom event control
static sl_zigbee_event_t finding_and_binding_event_control; // Custom event control
static sl_zigbee_event_t leave_network_event_control; // Custom event control
static sl_zigbee_event_t attribute_report_event_control; // Custom event control
static sl_zigbee_event_t sensor_init_event_control; // Custom event control

static bool commissioning = false; // Holds the commissioning status
static bool binding = false; // Holds the binding status


/** @brief Complete network steering.
 *
 * This callback is fired when the Network Steering plugin is complete.
 *
 * @param status On success this will be set to EMBER_SUCCESS to indicate a
 * network was joined successfully. On failure this will be the status code of
 * the last join or scan attempt. Ver.: always
 *
 * @param totalBeacons The total number of 802.15.4 beacons that were heard,
 * including beacons from different devices with the same PAN ID. Ver.: always
 * @param joinAttempts The number of join attempts that were made to get onto
 * an open Zigbee network. Ver.: always
 *
 * @param finalState The finishing state of the network steering process. From
 * this, one is able to tell on which channel mask and with which key the
 * process was complete. Ver.: always
 */

sl_bme280_dev bme;
pms_dev pm;
sl_bh1750_config cfg;
sl_bh1750_dev dev;
float measure_uv;
float measure_bat;

/** @brief Complete network steering.
 *
 * This callback is fired when the Network Steering plugin is complete.
 *
 * @param status On success this will be set to EMBER_SUCCESS to indicate a
 * network was joined successfully. On failure this will be the status code of
 * the last join or scan attempt. Ver.: always
 *
 * @param totalBeacons The total number of 802.15.4 beacons that were heard,
 * including beacons from different devices with the same PAN ID. Ver.: always
 * @param joinAttempts The number of join attempts that were made to get onto
 * an open Zigbee network. Ver.: always
 *
 * @param finalState The finishing state of the network steering process. From
 * this, one is able to tell on which channel mask and with which key the
 * process was complete. Ver.: always
 */
void emberAfPluginNetworkSteeringCompleteCallback(EmberStatus status,
                                                  uint8_t totalBeacons,
                                                  uint8_t joinAttempts,
                                                  uint8_t finalState)
{
  sl_zigbee_app_debug_print("%s network %s: 0x%02X\n", "Join", "complete", status);

  if (status != EMBER_SUCCESS) {
    commissioning = false;
  } else {
    // On successful join, do find and bind after a short delay
    sl_zigbee_event_set_delay_ms(&finding_and_binding_event_control,
                                 FIND_AND_BIND_DELAY_MS);
  }
}

/** @brief
 *
 * Application framework equivalent of ::emberRadioNeedsCalibratingHandler
 */
void emberAfRadioNeedsCalibratingCallback(void)
{
  sl_mac_calibrate_current_channel();
}// Sending-OnOff-Commands: Step 2

#include "sl_simple_button.h"
#include "sl_simple_button_instances.h"

#define BUTTON0 0
#define BUTTON1 1

void emberAfMainInitCallback(void)
{

  sl_zigbee_app_debug_print("FPT IoT Challenge SED Zigbee\n");

  // Enable BH1750
  cfg.address = BH1750_ADDR_LO;
  cfg.mode = BH1750_MODE_CONTINUOUS;
  cfg.resolution = BH1750_RES_HIGH;
  bh1750_init(sl_i2cspm_inst0);
  if (bh1750_configure(cfg) == SL_STATUS_OK) {
      printf("Init BH1750 done!\n");
  }
  else {
      printf("Init BH1750 failed!");
  }

  // Enable ADC to read ML8511
  ADC_init();

  sl_zigbee_event_init(&network_steering_event_control, network_steering_event_handler);
  sl_zigbee_event_init(&finding_and_binding_event_control, finding_and_binding_event_handler);
  sl_zigbee_event_init(&leave_network_event_control, leave_network_event_handler);
  sl_zigbee_event_init(&attribute_report_event_control, attribute_report_event_handler);
  sl_zigbee_event_init(&sensor_init_event_control, sensor_init_event_handler);

  // Print content of binding table
  binding_table_print();
}

/** @brief Stack Status
 *
 * This function is called by the application framework from the stack status
 * handler.  This callbacks provides applications an opportunity to be notified
 * of changes to the stack status and take appropriate action. The framework
 * will always process the stack status after the callback returns.
 */
void emberAfStackStatusCallback(EmberStatus status)
{
  if (status == EMBER_NETWORK_DOWN) {
    sl_zigbee_event_set_inactive(&attribute_report_event_control);
    sl_zigbee_event_set_inactive(&sensor_init_event_control);
  } else if (status == EMBER_NETWORK_UP) {
    if (binding_table_unicast_binding_count() > 0) {
      binding = true;
      // If already in a network and bindings are valid, report attributes
      sl_zigbee_event_set_active(&attribute_report_event_control);
      sl_zigbee_event_set_active(&sensor_init_event_control);
    }
  }
}

void sl_button_on_change(const sl_button_t *handle)
{
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_RELEASED) {
    if (&sl_button_btn0 == handle) {
      sl_zigbee_event_set_active(&network_steering_event_control);
    } else if (&sl_button_btn1 == handle) {
      sl_zigbee_event_set_active(&leave_network_event_control);
    }
  }
}

/** @brief Complete
 *
 * This callback is fired by the initiator when the Find and Bind process is
 * complete.
 *
 * @param status Status code describing the completion of the find and bind
 * process Ver.: always
 */
void emberAfPluginFindAndBindInitiatorCompleteCallback(EmberStatus status)
{
  sl_zigbee_app_debug_print("Find and bind initiator %s: 0x%X\n", "complete", status);

  if (status == EMBER_SUCCESS) {
    sl_zigbee_event_set_delay_ms(&sensor_init_event_control,
                                 REPORT_PERIOD_MS);
  } else {
    sl_zigbee_app_debug_print("Ensure a valid binding target!\n");
    sl_zigbee_event_set_inactive(&attribute_report_event_control);
    sl_zigbee_event_set_inactive(&sensor_init_event_control);
    binding = false;
  }
}

// -----------------------------------------------------------------------------
//                          Event Handler
// -----------------------------------------------------------------------------

/** @brief Leave Network Event Handler
 *
 * This event handler is called in response to it's respective control
 * activation. It handles the network leaving process.
 *
 */
static void leave_network_event_handler(sl_zigbee_event_t *event)
{
  EmberStatus status;

  // Clear binding table
  status = emberClearBindingTable();
  sl_zigbee_app_debug_print("%s 0x%x", "Clear binding table\n", status);

  // Leave network
  status = emberLeaveNetwork();
  sl_zigbee_app_debug_print("%s 0x%x", "leave\n", status);

  commissioning = false;
  binding = false;
}

/** @brief Network Steering Event Handler
 *
 * This event handler is called in response to it's respective control
 * activation. It handles the network steering process. If already in a network
 * it forces the device to report it's attributes: Temperature and relative
 * humidity
 *
 */
static void network_steering_event_handler(sl_zigbee_event_t *event)
{
  EmberStatus status;

  if (emberAfNetworkState() == EMBER_JOINED_NETWORK) {
    // Check if the device has successfully established bindings, if not do so
    if (!binding) {
      sl_zigbee_event_set_active(&finding_and_binding_event_control);
    }
  } else {
    // If not in a network, attempt to join one
    status = emberAfPluginNetworkSteeringStart();
    sl_zigbee_app_debug_print("%s network %s: 0x%X\n",
                              "Join",
                              "start",
                              status);
    commissioning = true;
  }
}

/** @brief Find and Bind Event Handler
 *
 * This event handler is called in response to it's respective control
 * activation. It handles the find and bind process as an initiator. It requires
 * a valid target. Upon a successful procedure, a series of binding will be
 * added to the binding table of the device for matching clusters found in the
 * target.
 *
 */
static void finding_and_binding_event_handler(sl_zigbee_event_t *event)
{
  EmberStatus status;
  status = emberAfPluginFindAndBindInitiatorStart(RHT_MEASUREMENT_ENDPOINT);

  sl_zigbee_app_debug_print("Find and bind initiator %s: 0x%X\n", "start", status);

  binding = true;
}

static void sensor_init_event_handler(sl_zigbee_event_t *event)
{
  // Enable power 5V and 3.3V
  GPIO_PinOutSet (gpioPortC, 0);
  GPIO_PinOutSet (gpioPortC, 1);

  // Enable the BME280
  i2c_scan_init (sl_i2cspm_mikroe);

  // Enable PMS7003
  pms7003_init (&pm, outdoor);

  sl_zigbee_event_set_delay_ms(&sensor_init_event_control,
                                 REPORT_PERIOD_MS);

  sl_zigbee_event_set_active(&attribute_report_event_control);
}

/** @brief Attributes report Event Handler
 *
 * This event handler is called in response to its respective control
 * activation. It will report the MeasuredValue of the RH and Temperature
 * measurement server clusters. Data will be sent though the matching binding
 *
 */
static void attribute_report_event_handler(sl_zigbee_event_t *event)
{

  EmberStatus status = EMBER_SUCCESS;
  uint32_t rh;
  int32_t t;
  uint32_t p;
  uint16_t pm1;
  uint16_t pm2_5;
  uint16_t pm10;
  uint16_t uv;
  uint16_t bat;
  union {
    int16_t t;
    uint16_t rh;
    uint16_t p;
    uint16_t pm1;
    uint16_t pm2_5;
    uint16_t pm10;
    uint16_t ilm;
    uint16_t uv;
    uint16_t bat;
  }attribute;


  if (emberAfNetworkState() != EMBER_JOINED_NETWORK) {
    return;
  }

    i2c_scan_process(&bme);


    rh = (int)(bme.humidity*1000);
    t = (int)(bme.temperature*1000);
    p = (int)(bme.pressure*100);

    // Attribute MeasuredValue = 100 x temperature in degrees Celsius.
    attribute.t = t / 10;
#if USE_SEND_REPORT_TO_BINDINGS
    // Send report directly
    status = reportAttribute( ZCL_TEMP_MEASUREMENT_CLUSTER_ID,
                              ZCL_TEMP_MEASURED_VALUE_ATTRIBUTE_ID,
                              ZCL_INT16S_ATTRIBUTE_TYPE,
                              (uint8_t *)&attribute);
#else
    // Just update attribute data to send report to bindings
    status = emberAfWriteServerAttribute(RHT_MEASUREMENT_ENDPOINT,
                                         ZCL_TEMP_MEASUREMENT_CLUSTER_ID,
                                         ZCL_TEMP_MEASURED_VALUE_ATTRIBUTE_ID,
                                         (uint8_t *)&attribute,
                                         ZCL_INT16S_ATTRIBUTE_TYPE);
    // emberAfWriteManufacturerSpecificServerAttribute()
#endif
    sl_zigbee_app_debug_print("%s reported: 0x%X\n\r", "Temp - MeasuredValue", status);

    // Attribute MeasuredValue = 100 x Relative humidity
    attribute.rh = rh / 10;
#if USE_SEND_REPORT_TO_BINDINGS
    // Send report directly
    status = reportAttribute( ZCL_TEMP_MEASUREMENT_CLUSTER_ID,
                              ZCL_TEMP_MEASURED_VALUE_ATTRIBUTE_ID,
                              ZCL_INT16U_ATTRIBUTE_TYPE,
                              (uint8_t *)&attribute);
#else
    // Just update attribute data to send report to bindings
    status = emberAfWriteServerAttribute(RHT_MEASUREMENT_ENDPOINT,
                                         ZCL_RELATIVE_HUMIDITY_MEASUREMENT_CLUSTER_ID,
                                         ZCL_RELATIVE_HUMIDITY_MEASURED_VALUE_ATTRIBUTE_ID,
                                         (uint8_t *)&attribute,
                                         ZCL_INT16U_ATTRIBUTE_TYPE);
#endif
    sl_zigbee_app_debug_print("%s reported: 0x%X\n\r", "Hum - MeasuredValue", status);

    // Attribute MeasuredValue = 10 x Pressure
    attribute.p = p / 10;
#if USE_SEND_REPORT_TO_BINDINGS
    // Send report directly
    status = reportAttribute( ZCL_TEMP_MEASUREMENT_CLUSTER_ID,
                              ZCL_TEMP_MEASURED_VALUE_ATTRIBUTE_ID,
                              ZCL_INT16U_ATTRIBUTE_TYPE,
                              (uint8_t *)&attribute);
#else
    // Just update attribute data to send report to bindings
    status = emberAfWriteServerAttribute(RHT_MEASUREMENT_ENDPOINT,
                                         ZCL_PRESSURE_MEASUREMENT_CLUSTER_ID,
                                         ZCL_PRESSURE_MEASURED_VALUE_ATTRIBUTE_ID,
                                         (uint8_t *)&attribute,
                                         ZCL_INT16U_ATTRIBUTE_TYPE);
#endif
    sl_zigbee_app_debug_print("%s reported: 0x%X\n\r", "Pres - MeasuredValue", status);

    pm1 = pm.pm1_0;
    pm2_5 = pm.pm2_5;
    pm10 = pm.pm10;

    attribute.pm1 = pm1;
    status = emberAfWriteManufacturerSpecificServerAttribute(RHT_MEASUREMENT_ENDPOINT,
                                                             ZCL_SAMPLE_MFG_SPECIFIC_PARTICLE_CLUSTER_ID,
                                                             ZCL_ATTRIBUTE_ONE_ATTRIBUTE_ID,
                                                             MANUFACTURE_CODE,
                                                             (uint8_t *)&attribute,
                                                             ZCL_INT16U_ATTRIBUTE_TYPE);
    sl_zigbee_app_debug_print("%s reported: 0x%X\n\r", "PM1 - MeasuredValue", status);

    attribute.pm2_5 = pm2_5;
    status = emberAfWriteManufacturerSpecificServerAttribute(RHT_MEASUREMENT_ENDPOINT,
                                                             ZCL_SAMPLE_MFG_SPECIFIC_PARTICLE25_CLUSTER_ID,
                                                             ZCL_MEASURED_PM25_ATTRIBUTE_ID,
                                                             MANUFACTURE_CODE,
                                                             (uint8_t *)&attribute,
                                                             ZCL_INT16U_ATTRIBUTE_TYPE);
    sl_zigbee_app_debug_print("%s reported: 0x%X\n\r", "PM2.5 - MeasuredValue", status);

    attribute.pm10 = pm10;
    status = emberAfWriteManufacturerSpecificServerAttribute(RHT_MEASUREMENT_ENDPOINT,
                                                             ZCL_SAMPLE_MFG_SPECIFIC_PARTICLE10_CLUSTER_ID,
                                                             ZCL_MEASURED_PM10_ATTRIBUTE_ID,
                                                             MANUFACTURE_CODE,
                                                             (uint8_t *)&attribute,
                                                             ZCL_INT16U_ATTRIBUTE_TYPE);
    sl_zigbee_app_debug_print("%s reported: 0x%X\n\r", "PM10 - MeasuredValue", status);

    ADC_read_bat(&measure_bat);

    bat = (int) (measure_bat * 100);
    attribute.bat = bat;
    status = emberAfWriteServerAttribute(RHT_MEASUREMENT_ENDPOINT,
                                         ZCL_ELECTRICAL_MEASUREMENT_CLUSTER_ID,
                                         ZCL_DC_VOLTAGE_ATTRIBUTE_ID,
                                         (uint8_t *)&attribute,
                                         ZCL_INT16U_ATTRIBUTE_TYPE);
    sl_zigbee_app_debug_print("%s reported: 0x%X\n\r", "Power - MeasuredValue", status);

    ADC_read(&measure_uv);

    uv = (int)(measure_uv*1000);
    attribute.uv = uv;
//    status = emberAfWriteManufacturerSpecificServerAttribute(RHT_MEASUREMENT_ENDPOINT,
//                                                             ZCL_SAMPLE_MFG_SPECIFIC_UV_CLUSTER_ID,
//                                                             ZCL_MEASURED_UV_ATTRIBUTE_ID,
//                                                             MANUFACTURE_CODE,
//                                                             (uint8_t *)&attribute,
//                                                             ZCL_INT16U_ATTRIBUTE_TYPE);
    status = emberAfWriteServerAttribute(RHT_MEASUREMENT_ENDPOINT,
                                         ZCL_ILLUM_MEASUREMENT_CLUSTER_ID,
                                         ZCL_ILLUM_MEASURED_VALUE_ATTRIBUTE_ID,
                                         (uint8_t *)&attribute,
                                         ZCL_INT16U_ATTRIBUTE_TYPE);
    sl_zigbee_app_debug_print("%s reported: 0x%X\n\r", "UV - MeasuredValue", status);

  // Disable power 5V and 3.3V
  GPIO_PinOutClear (gpioPortC, 0);
  GPIO_PinOutClear (gpioPortC, 1);
}

static uint8_t binding_table_unicast_binding_count(void)
{
  uint8_t i;
  EmberBindingTableEntry result;
  uint8_t bindings = 0;

  for (i = 0; i < emberAfGetBindingTableSize(); i++) {
    EmberStatus status = emberGetBinding(i, &result);
    if (status == EMBER_SUCCESS
        && result.type == EMBER_UNICAST_BINDING) {
      bindings++;
    }
  }
  return bindings;
}

static void binding_table_print(void)
{
  uint8_t i;
  EmberBindingTableEntry result;

  PGM_P typeStrings[] = {
    "EMPTY",
    "UNICA",
    "M2ONE",
    "MULTI",
    "?    ",
  };
  uint8_t bindings = 0;

  emberAfCorePrintln("#  type   nwk  loc   rem   clus   node   eui");
  for (i = 0; i < emberAfGetBindingTableSize(); i++) {
    EmberStatus status = emberGetBinding(i, &result);
    if (status == EMBER_SUCCESS) {
      if (result.type > EMBER_MULTICAST_BINDING) {
        result.type = 4;  // last entry in the string list above
      }
      if (result.type != EMBER_UNUSED_BINDING) {
        bindings++;
        emberAfCorePrint("%d: ", i);
        emberAfCorePrint("%p", typeStrings[result.type]);
        emberAfCorePrint("  %d    0x%x  0x%x  0x%2x 0x%2x ",
                        result.networkIndex,
                        result.local,
                        result.remote,
                        result.clusterId,
                        emberGetBindingRemoteNodeId(i));
        emberAfPrintBigEndianEui64(result.identifier);
        emberAfCorePrintln("");
      }
    } else {
      emberAfCorePrintln("0x%x: emberGetBinding Error: %x", status);
      emberAfAppFlush();
    }
    emberAfAppFlush();
  }
  emberAfCorePrintln("%d of %d bindings used",
      bindings,
      emberAfGetBindingTableSize());
}
