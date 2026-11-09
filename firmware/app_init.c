/***************************************************************************//**
 * @file
 * @brief app_init.c
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include <stdint.h>
#include <inttypes.h>
#include "sl_component_catalog.h"
#include "sl_common.h"
#include "sl_rail_sdk_simple_assistance.h"
#include "sl_rail.h"
#include "sl_rail_util_init.h"
#include "app_process.h"
#include "sl_rail_sdk_channel_selector.h"

#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "app_task_init.h"
#endif

#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif
#include "em_device.h"
#include "sl_gpio.h"
#include "sl_iostream.h"
#include "sl_iostream_init_eusart_instances.h"

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
static void serial_mode_quiet_log_input(void);

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * Print sample app name
 *****************************************************************************/
SL_WEAK void print_sample_app_name(const char* app_name)
{
#ifdef SL_CATALOG_APP_LOG_PRESENT
  app_log_info("%s\n", app_name);
#endif
  // Avoid unused parameter warning
  (void)app_name;
}

/******************************************************************************
 * The function is used for some basic initialization related to the app.
 *****************************************************************************/
void rail_app_init(void)
{
  // Get RAIL handle, used later by the application
  sl_rail_handle_t rail_handle = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);

  // Turn OFF LEDs
  clear_receive_led();
  clear_send_led();
  // Start reception
  sl_rail_status_t status = sl_rail_start_rx(rail_handle, get_selected_channel(), NULL);
  if (status != SL_RAIL_STATUS_NO_ERROR) {
    app_log_warning("After initialization sl_rail_start_rx() result: 0x%08" PRIX32 "\n ", status);
  }

  // CLI info message
  print_sample_app_name("Simple TRX");
}

void app_init(void)
{
  serial_mode_quiet_log_input();   // lemon-serial: before the CLI's first pass
#if !defined(SL_CATALOG_KERNEL_PRESENT)
  rail_app_init();
#else
  app_task_init();
#endif
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * lemon-serial: a transmitter (F/S open) with the RANGE jumper fitted at power-up is controlled by the encoder's
 * MULTI frames on PA06, which is also the log UART's input. Stop that input here, before the CLI's first pass,
 * and drop what it has already received, so the CLI does not echo the frames into the log as garbage.
 * Any other jumper setting: nothing changes. (app_process.c bay_init sets up the serial input itself.)
 *****************************************************************************/
static void serial_mode_quiet_log_input(void)
{
  sl_gpio_t range = { .port = SL_GPIO_PORT_B, .pin = 2 };    // RANGE, active low
  sl_gpio_t fs    = { .port = SL_GPIO_PORT_B, .pin = 3 };    // F/S,   active low
  sl_gpio_set_pin_mode(&range, SL_GPIO_MODE_INPUT_PULL, 1);
  sl_gpio_set_pin_mode(&fs,    SL_GPIO_MODE_INPUT_PULL, 1);
  for (volatile uint32_t i = 0; i < 2000u; i++) { }           // let the pull-ups settle
  bool range_high = true, fs_high = true;
  sl_gpio_get_pin_input(&range, &range_high);
  sl_gpio_get_pin_input(&fs, &fs_high);
  if (range_high || !fs_high) return;                          // no RANGE jumper, or F/S fitted: as before
  EUSART0->CMD = EUSART_CMD_RXDIS;
  for (volatile uint32_t i = 0; i < 20000u; i++) { }          // what is still in the FIFO reaches the iostream buffer
  uint8_t junk[16];
  size_t n;
  do {
    n = 0;
    if (sl_iostream_read(sl_iostream_inst_handle, junk, sizeof(junk), &n) != SL_STATUS_OK) break;
  } while (n != 0u);
}
