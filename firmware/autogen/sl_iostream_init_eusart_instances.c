#include "sl_iostream.h"
#include "sl_iostream_uart.h"
#include "sl_iostream_eusart.h"
#include "sl_common.h"
#include "sl_device_peripheral.h"
#include "sl_clock_manager.h"
#include "sl_hal_eusart.h"
#include "sl_hal_gpio.h"


#include "sl_clock_manager_tree_config.h"
 

 
// Include instance config 
 #include "sl_iostream_eusart_inst_config.h"

// MACROs for generating name and IRQ handler function  
#if defined(EUART_COUNT) && (EUART_COUNT > 0) 
#define SL_IOSTREAM_EUSART_TX_IRQ_NUMBER(periph_nbr)     SL_CONCAT_PASTER_3(EUART, periph_nbr, _TX_IRQn)   
#define SL_IOSTREAM_EUSART_RX_IRQ_NUMBER(periph_nbr)     SL_CONCAT_PASTER_3(EUART, periph_nbr, _RX_IRQn)   
#define SL_IOSTREAM_EUSART_TX_IRQ_HANDLER(periph_nbr)    SL_CONCAT_PASTER_3(EUART, periph_nbr, _TX_IRQHandler)   
#define SL_IOSTREAM_EUSART_RX_IRQ_HANDLER(periph_nbr)    SL_CONCAT_PASTER_3(EUART, periph_nbr, _RX_IRQHandler)   
#define SL_IOSTREAM_EUSART_RX_DMA_SIGNAL(periph_nbr)     SL_CONCAT_PASTER_3(SL_DMA_SIGNAL_EUART, periph_nbr, _RXFL)
#define SL_IOSTREAM_EUSART_TX_DMA_SIGNAL(periph_nbr)     SL_CONCAT_PASTER_3(SL_DMA_SIGNAL_EUART, periph_nbr, _TXFL)
#define SL_IOSTREAM_EUSART_CLOCK_REF(periph_nbr)         SL_CONCAT_PASTER_2(SL_BUS_CLOCK_EUART, periph_nbr)  
#define SL_IOSTREAM_EUSART_PERIPHERAL(periph_nbr)        SL_CONCAT_PASTER_2(SL_PERIPHERAL_EUART, periph_nbr) 
#else
#define SL_IOSTREAM_EUSART_TX_IRQ_NUMBER(periph_nbr)     SL_CONCAT_PASTER_3(EUSART, periph_nbr, _TX_IRQn)   
#define SL_IOSTREAM_EUSART_RX_IRQ_NUMBER(periph_nbr)     SL_CONCAT_PASTER_3(EUSART, periph_nbr, _RX_IRQn)   
#define SL_IOSTREAM_EUSART_TX_IRQ_HANDLER(periph_nbr)    SL_CONCAT_PASTER_3(EUSART, periph_nbr, _TX_IRQHandler)   
#define SL_IOSTREAM_EUSART_RX_IRQ_HANDLER(periph_nbr)    SL_CONCAT_PASTER_3(EUSART, periph_nbr, _RX_IRQHandler)   
#define SL_IOSTREAM_EUSART_RX_DMA_SIGNAL(periph_nbr)     SL_CONCAT_PASTER_3(SL_DMA_SIGNAL_EUSART, periph_nbr, _RXFL)
#define SL_IOSTREAM_EUSART_TX_DMA_SIGNAL(periph_nbr)     SL_CONCAT_PASTER_3(SL_DMA_SIGNAL_EUSART, periph_nbr, _TXFL)
#define SL_IOSTREAM_EUSART_CLOCK_REF(periph_nbr)         SL_CONCAT_PASTER_2(SL_BUS_CLOCK_EUSART, periph_nbr)  
#define SL_IOSTREAM_EUSART_PERIPHERAL(periph_nbr)        SL_CONCAT_PASTER_2(SL_PERIPHERAL_EUSART, periph_nbr) 
#endif


#if defined(EUART_COUNT) && (EUART_COUNT > 0)
#define SL_IOSTREAM_EUSART_CLOCK_SOURCE(periph_nbr)      SL_CLOCK_MANAGER_EUART0CLK_SOURCE
#define SL_IOSTREAM_EUSART_HF_CLOCK_SOURCE               CMU_EUART0CLKCTRL_CLKSEL_EM01GRPACLK
#define SL_IOSTREAM_EUSART_DISABLED_CLOCK_SOURCE         CMU_EUART0CLKCTRL_CLKSEL_DISABLED
#else
#define SL_IOSTREAM_EUSART_CLOCK_SOURCE(periph_nbr)      SL_CLOCK_MANAGER_EUSART0CLK_SOURCE
#if defined(CMU_EUSART0CLKCTRL_CLKSEL_EM01GRPCCLK)
#define SL_IOSTREAM_EUSART_HF_CLOCK_SOURCE               CMU_EUSART0CLKCTRL_CLKSEL_EM01GRPCCLK
#elif defined(CMU_EUSART0CLKCTRL_CLKSEL_EM01GRPACLK)
#define SL_IOSTREAM_EUSART_HF_CLOCK_SOURCE               CMU_EUSART0CLKCTRL_CLKSEL_EM01GRPACLK
#endif
#define SL_IOSTREAM_EUSART_DISABLED_CLOCK_SOURCE         CMU_EUSART0CLKCTRL_CLKSEL_DISABLED
#endif

// Check clock configuration
 
#if (SL_IOSTREAM_EUSART_CLOCK_SOURCE(SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO) == \
    SL_IOSTREAM_EUSART_DISABLED_CLOCK_SOURCE)
  #error Peripheral clock is disabled for INST. Modify sl_clock_manager_tree_config.h \
  to enable the peripheral clock
#else
#define _SL_IOSTREAM_EUSART_INST_ENABLE_HIGH_FREQUENCY \
          SL_IOSTREAM_EUSART_CLOCK_SOURCE(SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO) == \
          SL_IOSTREAM_EUSART_HF_CLOCK_SOURCE

#if defined(SL_IOSTREAM_EUSART_INST_ENABLE_HIGH_FREQUENCY) && \
  (_SL_IOSTREAM_EUSART_INST_ENABLE_HIGH_FREQUENCY != \
  SL_IOSTREAM_EUSART_INST_ENABLE_HIGH_FREQUENCY)
#if SL_IOSTREAM_EUSART_INST_ENABLE_HIGH_FREQUENCY
#warning Configuration mismatch for IOStream EUSART INST. \
 IOStream was configured in high-frequency, but peripheral uses a low-frequency \
 oscillator in sl_clock_manager_tree_config.h.
#else 
#warning Configuration mismatch for IOStream EUSART INST. \
 IOStream was configured in low-frequency, but peripheral uses a high-frequency \
 oscillator in sl_clock_manager_tree_config.h.
#endif // SL_IOSTREAM_EUSART_INST_ENABLE_HIGH_FREQUENCY
#endif // Config mismatch
#endif // SL_IOSTREAM_EUSART_DISABLED_CLOCK_SOURCE


 

 


sl_status_t sl_iostream_eusart_init_inst(void);


// Instance(s) handle and context variable 
static sl_iostream_uart_t sl_iostream_inst;
sl_iostream_t *sl_iostream_inst_handle = &sl_iostream_inst.stream;

sl_iostream_uart_t *sl_iostream_uart_inst_handle = &sl_iostream_inst;
static sl_iostream_eusart_context_t  context_inst;

static uint8_t  rx_buffer_inst[SL_IOSTREAM_EUSART_INST_RX_BUFFER_SIZE];

static sli_iostream_uart_periph_t uart_periph_inst = {
  .rx_irq_number = SL_IOSTREAM_EUSART_RX_IRQ_NUMBER(SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO),
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)  
  .tx_irq_number = SL_IOSTREAM_EUSART_TX_IRQ_NUMBER(SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO),
#endif
};

sl_iostream_instance_info_t sl_iostream_instance_inst_info = {
  .handle = &sl_iostream_inst.stream,
  .name = "inst",
  .type = SL_IOSTREAM_TYPE_UART,
  .periph_id = SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO,
  .init = sl_iostream_eusart_init_inst,
};



sl_status_t sl_iostream_eusart_init_inst(void)
{
  sl_status_t status;

  sl_iostream_eusart_config_t config_inst = { 
    .eusart = SL_IOSTREAM_EUSART_PERIPHERAL(SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO),
    .eusart_nbr = SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO,
    .bus_clock = SL_IOSTREAM_EUSART_CLOCK_REF(SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO),
    .baudrate = SL_IOSTREAM_EUSART_INST_BAUDRATE,
    .parity = SL_IOSTREAM_EUSART_INST_PARITY,
    .flow_control = SL_IOSTREAM_EUSART_INST_FLOW_CONTROL_TYPE,
    .stop_bits = SL_IOSTREAM_EUSART_INST_STOP_BITS,
#if defined(EUSART_COUNT) && (EUSART_COUNT > 1)
    .port_index = SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO,
#endif
    .tx_port = SL_IOSTREAM_EUSART_INST_TX_PORT,
    .tx_pin = SL_IOSTREAM_EUSART_INST_TX_PIN,
    .rx_port = SL_IOSTREAM_EUSART_INST_RX_PORT,
    .rx_pin = SL_IOSTREAM_EUSART_INST_RX_PIN,
#if defined(SL_IOSTREAM_EUSART_INST_CTS_PORT)
    .cts_port = SL_IOSTREAM_EUSART_INST_CTS_PORT,
    .cts_pin = SL_IOSTREAM_EUSART_INST_CTS_PIN,
#endif
#if defined(SL_IOSTREAM_EUSART_INST_RTS_PORT)
    .rts_port = SL_IOSTREAM_EUSART_INST_RTS_PORT,
    .rts_pin = SL_IOSTREAM_EUSART_INST_RTS_PIN,
#endif
  };

  sl_iostream_dma_config_t rx_dma_config_inst = {.src = (uint8_t *)&SL_IOSTREAM_EUSART_INST_PERIPHERAL->RXDATA,
                                                        .xfer_cfg = SL_HAL_LDMA_TRANSFER_CFG_PERIPHERAL(*(SL_IOSTREAM_EUSART_RX_DMA_SIGNAL(SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO)))};

  sl_iostream_dma_config_t tx_dma_config_inst = {.dst = (uint8_t *)&SL_IOSTREAM_EUSART_INST_PERIPHERAL->TXDATA,
                                                        .xfer_cfg = SL_HAL_LDMA_TRANSFER_CFG_PERIPHERAL(*(SL_IOSTREAM_EUSART_TX_DMA_SIGNAL(SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO)))};

  sl_iostream_uart_config_t uart_config_inst = {
    .rx_dma_cfg = rx_dma_config_inst,
    .tx_dma_cfg = tx_dma_config_inst,
    .rx_buffer = rx_buffer_inst,
    .rx_buffer_length = SL_IOSTREAM_EUSART_INST_RX_BUFFER_SIZE,
    .enable_high_frequency = _SL_IOSTREAM_EUSART_INST_ENABLE_HIGH_FREQUENCY, 
    .lf_to_crlf = SL_IOSTREAM_EUSART_INST_CONVERT_BY_DEFAULT_LF_TO_CRLF,
    .rx_when_sleeping = SL_IOSTREAM_EUSART_INST_RESTRICT_ENERGY_MODE_TO_ALLOW_RECEPTION,
    .uart_periph = &uart_periph_inst
  };
  uart_config_inst.sw_flow_control = SL_IOSTREAM_EUSART_INST_FLOW_CONTROL_TYPE == SL_IOSTREAM_EUSART_UART_FLOW_CTRL_SOFT;
#if defined(SL_IOSTREAM_EUSART_INST_ASYNC_TX)
  uart_config_inst.async_tx_enabled = SL_IOSTREAM_EUSART_INST_ASYNC_TX;
#else
  uart_config_inst.async_tx_enabled = false;
#endif
  // Instantiate eusart instance 
  status = sl_iostream_eusart_init(&sl_iostream_inst,
                                  &uart_config_inst,
                                  &config_inst,
                                  &context_inst);
  EFM_ASSERT(status == SL_STATUS_OK);

   

  return status;
}



void sl_iostream_eusart_init_instances(void)
{
   
  // Instantiate eusart instance(s) 
  
  sl_iostream_eusart_init_inst();
  
}


void SL_IOSTREAM_EUSART_TX_IRQ_HANDLER(SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO)(void)
{
  sl_iostream_eusart_irq_handler(&sl_iostream_inst);
}

void SL_IOSTREAM_EUSART_RX_IRQ_HANDLER(SL_IOSTREAM_EUSART_INST_PERIPHERAL_NO)(void)
{
  sl_iostream_eusart_irq_handler(&sl_iostream_inst);
}


