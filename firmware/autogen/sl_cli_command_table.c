/***************************************************************************//**
 * @file sl_cli_command_table.c
 * @brief Declarations of relevant command structs for cli framework.
 * @version x.y.z
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

#include <stdlib.h>

#include "sl_cli_config.h"
#include "sl_cli_command.h"
#include "sl_cli_arguments.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *****************************   TEMPLATED FILE   ******************************
 ******************************************************************************/

/*******************************************************************************
 * Example syntax (.slcc or .slcp) for populating this file:
 *
 *   template_contribution:
 *     - name: cli_command          # Register a command
 *       value:
 *         name: status             # Name of command
 *         handler: status_command  # Function to be called. Must be defined
 *         help: "Prints status"    # Optional help description
 *         shortcuts:               # Optional shorcut list
 *           - name: st
 *         argument:                # Argument list, if apliccable
 *           - type: uint8          # Variable type
 *             help: "Channel"      # Optional description
 *           - type: string
 *             help: "Text"
 *     - name: cli_group            # Register a group
 *       value:
 *         name: shell              # Group name
 *         help: "Shell commands"   # Optional help description
 *         shortcuts:               # Optional shorcuts
 *           - name: sh
 *     - name: cli_command
 *       value:
 *         name: repeat
 *         handler: repeat_cmd
 *         help: "Repeat commands"
 *         shortcuts:
 *           - name: r
 *           - name: rep
 *         group: shell            # Associate command with group
 *         argument:
 *           - type: string
 *             help: "Text"
 *           - type: additional
 *             help: "More text"
 *
 * For subgroups, an optional unique id can be used to allow a particular name to
 * be used more than once. In the following case, from the command line the
 * following commands are available:
 *
 * >  root_1 shell status
 * >  root_2 shell status
 *
 *     - name: cli_group            # Register a group
 *       value:
 *         name: root_1             # Group name
 *
 *     - name: cli_group            # Register a group
 *       value:
 *         name: root_2             # Group name
 *
 *    - name: cli_group             # Register a group
 *       value:
 *         name: shell              # Group name
 *         id: shell_root_1         # Optional unique id for group
 *         group: root_1            # Add group to root_1 group
 *
 *    - name: cli_group             # Register a group
 *       value:
 *         name: shell              # Group name
 *         id: shell_root_2         # Optional unique id for group
 *         group: root_2            # Add group to root_1 group
 *
 *    - name: cli_command           # Register a command
 *       value:
 *         name: status
 *         handler: status_1
 *         group: shell_root_1      # id of subgroup
 *
 *    - name: cli_command           # Register a command
 *       value:
 *         name: status
 *         handler: status_2
 *         group: shell_root_2      # id of subgroup
 *
 ******************************************************************************/

// Provide function declarations
void cli_set_channel(sl_cli_command_arg_t *arguments);
void cli_get_channel(sl_cli_command_arg_t *arguments);
void cli_get_print_packet_info(sl_cli_command_arg_t *arguments);
void cli_set_print_packet_info(sl_cli_command_arg_t *arguments);
void cli_get_sidewalk_fcs_type(sl_cli_command_arg_t *arguments);
void cli_set_sidewalk_fcs_type(sl_cli_command_arg_t *arguments);
void cli_get_sidewalk_whitening(sl_cli_command_arg_t *arguments);
void cli_set_sidewalk_whitening(sl_cli_command_arg_t *arguments);
void cli_info(sl_cli_command_arg_t *arguments);
void cli_send_packet(sl_cli_command_arg_t *arguments);
void cli_receive_packet(sl_cli_command_arg_t *arguments);

// Command structs. Names are in the format : cli_cmd_{command group name}_{command name}
// In order to support hyphen in command and group name, every occurence of it while
// building struct names will be replaced by "_hyphen_"
static const sl_cli_command_info_t cli_cmd__set_channel = \
  SL_CLI_COMMAND(cli_set_channel,
                 "Set which channel the communication will happen",
                  "The channel number to apply" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT16, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__get_channel = \
  SL_CLI_COMMAND(cli_get_channel,
                 "Get which channel the communication is happening",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd_packet_assistant_get_print_packet_info = \
  SL_CLI_COMMAND(cli_get_print_packet_info,
                 "Get the print setting",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd_packet_assistant_set_print_packet_info = \
  SL_CLI_COMMAND(cli_set_print_packet_info,
                 "Set the print setting",
                  "ON/OFF" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT8, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd_packet_assistant_get_sidewalk_fcs_type = \
  SL_CLI_COMMAND(cli_get_sidewalk_fcs_type,
                 "Get the Sidewalk fsk fcs setting",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd_packet_assistant_set_sidewalk_fcs_type = \
  SL_CLI_COMMAND(cli_set_sidewalk_fcs_type,
                 "Set the Sidewalk fsk rate setting",
                  "ON/OFF" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT8, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd_packet_assistant_get_sidewalk_whitening = \
  SL_CLI_COMMAND(cli_get_sidewalk_whitening,
                 "Get the Sidewalk whitening setting",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd_packet_assistant_set_sidewalk_whitening = \
  SL_CLI_COMMAND(cli_set_sidewalk_whitening,
                 "Set the Sidewalk whitening setting",
                  "ON/OFF" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT8, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__info = \
  SL_CLI_COMMAND(cli_info,
                 "Unique ID of the MCU",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__send = \
  SL_CLI_COMMAND(cli_send_packet,
                 "Send a packet",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__receive = \
  SL_CLI_COMMAND(cli_receive_packet,
                 "Turn ON/OFF received message forwarding on CLI",
                  "ON/OFF" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT8, SL_CLI_ARG_END, });


// Create group command tables and structs if cli_groups given
// in template. Group name is suffixed with _group_table for tables
// and group commands are cli_cmd_grp_( group name )
static const sl_cli_command_entry_t packet_assistant_group_table[] = {
  { "get_print_packet_info", &cli_cmd_packet_assistant_get_print_packet_info, false },
  { "set_print_packet_info", &cli_cmd_packet_assistant_set_print_packet_info, false },
  { "get_sidewalk_fcs_type", &cli_cmd_packet_assistant_get_sidewalk_fcs_type, false },
  { "set_sidewalk_fcs_type", &cli_cmd_packet_assistant_set_sidewalk_fcs_type, false },
  { "get_sidewalk_whitening", &cli_cmd_packet_assistant_get_sidewalk_whitening, false },
  { "set_sidewalk_whitening", &cli_cmd_packet_assistant_set_sidewalk_whitening, false },
  { NULL, NULL, false },
};
static const sl_cli_command_info_t cli_cmd_grp_packet_assistant = \
  SL_CLI_COMMAND_GROUP(packet_assistant_group_table, "Packet assistant commands");

// Create root command table
const sl_cli_command_entry_t sl_cli_default_command_table[] = {
  { "set_channel", &cli_cmd__set_channel, false },
  { "get_channel", &cli_cmd__get_channel, false },
  { "info", &cli_cmd__info, false },
  { "send", &cli_cmd__send, false },
  { "receive", &cli_cmd__receive, false },
  { "packet_assistant", &cli_cmd_grp_packet_assistant, false },
  { "pa", &cli_cmd_grp_packet_assistant, true },
  { NULL, NULL, false },
};


#ifdef __cplusplus
}
#endif
