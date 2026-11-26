/*
 * board_config.h — which board the firmware is built for. Change the BOARD line (and, for the XIAO, the antenna
 * line), save, then build (.\build.ps1) and flash. The build prints the board it was made for.
 *
 *   BOARD_MODULE     the RF-BM-MG24B2I module on its carrier (Multi_Mode / plug-in / receiver boards): the normal build
 *   BOARD_XIAO_MG24  a Seeed Studio XIAO MG24 as a development board: 2.4 GHz only, log and command line over its
 *                    USB cable. Pins and what is left out: docs/XIAO_MG24.md
 *
 * Plain numbers only here: this file is also read by the SDK's configuration headers.
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define BOARD_MODULE      1
#define BOARD_XIAO_MG24   2

#define BOARD             BOARD_MODULE

/* XIAO MG24 only: 0 = the ceramic antenna on the board, 1 = the U.FL connector */
#define XIAO_ANTENNA_UFL  0

#if BOARD != BOARD_MODULE && BOARD != BOARD_XIAO_MG24
#error "board_config.h: BOARD must be BOARD_MODULE or BOARD_XIAO_MG24"
#endif
#endif
