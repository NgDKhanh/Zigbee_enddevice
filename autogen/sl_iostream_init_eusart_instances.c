#include "em_device.h"
#include "sl_iostream.h"
#include "sl_iostream_uart.h"
#include "sl_iostream_eusart.h"
// Include instance config 
 #include "sl_iostream_eusart_mikroe_config.h"

// MACROs for generating name and IRQ handler function  
#define SL_IOSTREAM_EUSART_CONCAT_PASTER(first, second, third)        first ##  second ## third
#if defined(EUART_COUNT) && (EUART_COUNT > 0)
#define SL_IOSTREAM_EUSART_TX_IRQ_NUMBER(periph_nbr)     SL_IOSTREAM_EUSART_CONCAT_PASTER(EUART, periph_nbr, _TX_IRQn)   
#define SL_IOSTREAM_EUSART_RX_IRQ_NUMBER(periph_nbr)     SL_IOSTREAM_EUSART_CONCAT_PASTER(EUART, periph_nbr, _RX_IRQn)   
#define SL_IOSTREAM_EUSART_TX_IRQ_HANDLER(periph_nbr)    SL_IOSTREAM_EUSART_CONCAT_PASTER(EUART, periph_nbr, _TX_IRQHandler)   
#define SL_IOSTREAM_EUSART_RX_IRQ_HANDLER(periph_nbr)    SL_IOSTREAM_EUSART_CONCAT_PASTER(EUART, periph_nbr, _RX_IRQHandler)   
#else
#define SL_IOSTREAM_EUSART_TX_IRQ_NUMBER(periph_nbr)     SL_IOSTREAM_EUSART_CONCAT_PASTER(EUSART, periph_nbr, _TX_IRQn)   
#define SL_IOSTREAM_EUSART_RX_IRQ_NUMBER(periph_nbr)     SL_IOSTREAM_EUSART_CONCAT_PASTER(EUSART, periph_nbr, _RX_IRQn)   
#define SL_IOSTREAM_EUSART_TX_IRQ_HANDLER(periph_nbr)    SL_IOSTREAM_EUSART_CONCAT_PASTER(EUSART, periph_nbr, _TX_IRQHandler)   
#define SL_IOSTREAM_EUSART_RX_IRQ_HANDLER(periph_nbr)    SL_IOSTREAM_EUSART_CONCAT_PASTER(EUSART, periph_nbr, _RX_IRQHandler)   
#endif
#define SL_IOSTREAM_EUSART_RX_DMA_SIGNAL(periph_nbr)     SL_IOSTREAM_EUSART_CONCAT_PASTER(dmadrvPeripheralSignal_EUSART, periph_nbr, _RXDATAV)
#if defined(LDMAXBAR_CH_REQSEL_SIGSEL_EUART0RXFL)
#define SL_IOSTREAM_EUART_RX_DMA_SIGNAL                  SL_IOSTREAM_EUSART_CONCAT_PASTER(dmadrvPeripheralSignal_EUART, 0, _RXDATAV)
#endif

#if defined(EUART_COUNT) && (EUART_COUNT > 0)
#define SL_IOSTREAM_EUSART_CLOCK_REF(periph_nbr)         SL_IOSTREAM_EUSART_CONCAT_PASTER(cmuClock_, EUART, periph_nbr)  
#else
#define SL_IOSTREAM_EUSART_CLOCK_REF(periph_nbr)         SL_IOSTREAM_EUSART_CONCAT_PASTER(cmuClock_, EUSART, periph_nbr)  
#endif

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
// EM Events
#define SLEEP_EM_EVENT_MASK      (SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM0)
static void events_handler(sl_power_manager_em_t from,
                           sl_power_manager_em_t to);
static sl_power_manager_em_transition_event_info_t events_info =
{
  .event_mask = SLEEP_EM_EVENT_MASK,
  .on_event = events_handler,
};
static sl_power_manager_em_transition_event_handle_t events_handle;
#endif // SL_CATALOG_POWER_MANAGER_PRESENT


sl_status_t sl_iostream_eusart_init_mikroe(void);


// Instance(s) handle and context variable 
static sl_iostream_uart_t sl_iostream_mikroe;
sl_iostream_t *sl_iostream_mikroe_handle = &sl_iostream_mikroe.stream;
sl_iostream_uart_t *sl_iostream_uart_mikroe_handle = &sl_iostream_mikroe;
static sl_iostream_eusart_context_t  context_mikroe;
static uint8_t  rx_buffer_mikroe[SL_IOSTREAM_EUSART_MIKROE_RX_BUFFER_SIZE];
sl_iostream_instance_info_t sl_iostream_instance_mikroe_info = {
  .handle = &sl_iostream_mikroe.stream,
  .name = "mikroe",
  .type = SL_IOSTREAM_TYPE_UART,
  .periph_id = SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL_NO,
  .init = sl_iostream_eusart_init_mikroe,
};



sl_status_t sl_iostream_eusart_init_mikroe(void)
{
  sl_status_t status;
#if (SL_IOSTREAM_EUSART_MIKROE_ENABLE_HIGH_FREQUENCY == 0)
  EUSART_UartInit_TypeDef init_mikroe = EUSART_UART_INIT_DEFAULT_LF;
#else
  EUSART_UartInit_TypeDef init_mikroe = EUSART_UART_INIT_DEFAULT_HF;
#endif
  init_mikroe.baudrate = SL_IOSTREAM_EUSART_MIKROE_BAUDRATE;
  init_mikroe.parity = SL_IOSTREAM_EUSART_MIKROE_PARITY;
  init_mikroe.stopbits = SL_IOSTREAM_EUSART_MIKROE_STOP_BITS;

  sl_iostream_eusart_config_t config_mikroe = { 
    .eusart = SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL,
    .enable_high_frequency = SL_IOSTREAM_EUSART_MIKROE_ENABLE_HIGH_FREQUENCY,
    .clock = SL_IOSTREAM_EUSART_CLOCK_REF(SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL_NO),
#if defined(EUSART_COUNT) && (EUSART_COUNT > 1)
    .port_index = SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL_NO,
#endif
    .tx_port = SL_IOSTREAM_EUSART_MIKROE_TX_PORT,
    .tx_pin = SL_IOSTREAM_EUSART_MIKROE_TX_PIN,
    .rx_port = SL_IOSTREAM_EUSART_MIKROE_RX_PORT,
    .rx_pin = SL_IOSTREAM_EUSART_MIKROE_RX_PIN,
#if defined(SL_IOSTREAM_EUSART_MIKROE_CTS_PORT)
    .cts_port = SL_IOSTREAM_EUSART_MIKROE_CTS_PORT,
    .cts_pin = SL_IOSTREAM_EUSART_MIKROE_CTS_PIN,
#endif
#if defined(SL_IOSTREAM_EUSART_MIKROE_RTS_PORT)
    .rts_port = SL_IOSTREAM_EUSART_MIKROE_RTS_PORT,
    .rts_pin = SL_IOSTREAM_EUSART_MIKROE_RTS_PIN,
#endif
  };
  config_mikroe.flow_control = SL_IOSTREAM_EUSART_MIKROE_FLOW_CONTROL_TYPE != uartFlowControlSoftware ? SL_IOSTREAM_EUSART_MIKROE_FLOW_CONTROL_TYPE : eusartHwFlowControlNone;
#if defined(LDMAXBAR_CH_REQSEL_SIGSEL_EUART0RXFL) && (SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL_NO == 0)
  sl_iostream_dma_config_t dma_config_mikroe = {.src = (uint8_t *)&SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL->RXDATA,
                                                        .peripheral_signal = SL_IOSTREAM_EUART_RX_DMA_SIGNAL};
#else
  sl_iostream_dma_config_t dma_config_mikroe = {.src = (uint8_t *)&SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL->RXDATA,
                                                        .peripheral_signal = SL_IOSTREAM_EUSART_RX_DMA_SIGNAL(SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL_NO)};
#endif 
  sl_iostream_uart_config_t uart_config_mikroe = {
    .dma_cfg = dma_config_mikroe,
    .rx_buffer = rx_buffer_mikroe,
    .rx_buffer_length = SL_IOSTREAM_EUSART_MIKROE_RX_BUFFER_SIZE,
    .tx_irq_number = SL_IOSTREAM_EUSART_TX_IRQ_NUMBER(SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL_NO),
    .rx_irq_number = SL_IOSTREAM_EUSART_RX_IRQ_NUMBER(SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL_NO),
    .lf_to_crlf = SL_IOSTREAM_EUSART_MIKROE_CONVERT_BY_DEFAULT_LF_TO_CRLF,
    .rx_when_sleeping = SL_IOSTREAM_EUSART_MIKROE_RESTRICT_ENERGY_MODE_TO_ALLOW_RECEPTION,
  };
  uart_config_mikroe.sw_flow_control = SL_IOSTREAM_EUSART_MIKROE_FLOW_CONTROL_TYPE == uartFlowControlSoftware;
  // Instantiate eusart instance 
  status = sl_iostream_eusart_init(&sl_iostream_mikroe,
                                  &uart_config_mikroe,
                                  &init_mikroe,
                                  &config_mikroe,
                                  &context_mikroe);
  EFM_ASSERT(status == SL_STATUS_OK);

  return status;
}



void sl_iostream_eusart_init_instances(void)
{
  #if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
  // Enable power manager notifications
  sl_power_manager_subscribe_em_transition_event(&events_handle, &events_info);
  #endif
  // Instantiate eusart instance(s) 
  
  sl_iostream_eusart_init_mikroe();
  
}


void SL_IOSTREAM_EUSART_TX_IRQ_HANDLER(SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL_NO)(void)
{
  sl_iostream_eusart_irq_handler(&sl_iostream_mikroe);
}

void SL_IOSTREAM_EUSART_RX_IRQ_HANDLER(SL_IOSTREAM_EUSART_MIKROE_PERIPHERAL_NO)(void)
{
  sl_iostream_eusart_irq_handler(&sl_iostream_mikroe);
}

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#if !defined(SL_CATALOG_KERNEL_PRESENT)
 
sl_power_manager_on_isr_exit_t sl_iostream_eusart_mikroe_sleep_on_isr_exit(void)
{
  return sl_iostream_uart_sleep_on_isr_exit(&sl_iostream_mikroe);
}

#endif // SL_CATALOG_KERNEL_PRESENT
static void events_handler(sl_power_manager_em_t from,
                           sl_power_manager_em_t to)
{
  (void) from;
  if (to < SL_POWER_MANAGER_EM3){
    // Only prepare for wakeup from EM2 or less, since EUSART doesn't run in EM3
    
    if (sl_iostream_uart_mikroe_handle->stream.context != NULL) {
      sl_iostream_uart_prepare_for_sleep(sl_iostream_uart_mikroe_handle);
    }
    
  }
}
#endif // SL_CATALOG_POWER_MANAGER_PRESENT
