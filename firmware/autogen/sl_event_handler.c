#include "sl_event_handler.h"

#include "sl_clock_manager.h"
#include "sl_rail_util_compatible_pa.h"
#include "sl_rail_util_init.h"
#include "sl_dma_manager_instances.h"
#include "app_log.h"
#include "sl_gpio.h"
#include "sl_iostream_init_eusart_instances.h"
#include "sl_iostream_stdlib_config.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_led_instances.h"
#include "sl_cli_instances.h"
#include "sl_iostream_init_instances.h"
#include "sl_rail_sdk_packet_assistant.h"
#include "sl_rail_sdk_channel_selector.h"
#include "sl_iostream_handles.h"

void sli_driver_permanent_allocation(void)
{
}

void sli_service_permanent_allocation(void)
{
}

void sli_stack_permanent_allocation(void)
{
}

void sli_internal_permanent_allocation(void)
{
}

void sl_platform_init(void)
{
  sl_clock_manager_runtime_init();
  sl_dma_manager_instances_init();
}

void sli_internal_init_early(void)
{
}

void sl_driver_init(void)
{
  sl_gpio_init();
  sl_simple_button_init_instances();
  sl_simple_led_init_instances();
}

void sl_service_init(void)
{
  sl_iostream_stdlib_disable_buffering();
  sl_iostream_init_instances_stage_1();
  sl_iostream_init_instances_stage_2();
  sl_cli_instances_init();
}

void sl_stack_init(void)
{
  sl_rail_util_pa_init();
  sl_rail_util_init();
}

void sl_internal_app_init(void)
{
  app_log_init();
  validation_check();
  set_configured_channel();
}

void sli_platform_process_action(void)
{
}

void sli_service_process_action(void)
{
  sl_cli_instances_tick();
}

void sli_stack_process_action(void)
{
}

void sli_internal_app_process_action(void)
{
}

void sl_iostream_init_instances_stage_1(void)
{
  sl_iostream_eusart_init_instances();
}

void sl_iostream_init_instances_stage_2(void)
{
  sl_iostream_set_console_instance();
}

