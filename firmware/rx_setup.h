/*
 * rx_setup.h — MiTo receiver setup over the air (rx_setup.c, v0.1: bench proof, RLINK).
 *
 * RxSetup: the transmitter's MULTI frames carry byte 26 bit 3 (and the receiver's output layout in byte 26
 * bits 0..1). The TX module then sends SETUP packets instead of flying data:
 *   on RXS_CH (2460 MHz) with their own sync word RXS_SYNC, RLINK's PHY (2GFSK 250 kbps), 16 bytes, -10 dBm,
 *   one every RXS_TX_EVERY_US:
 *   [0]     RXS_TYPE 0x5A
 *   [1..6]  owner: the TX module's factory chip number (EUI-64, low 48 bits)
 *   [7]     Prot   } the receiver mode, as the frame's Prot / Sub
 *   [8]     Sub    }
 *   [9]     output layout (servo_out.h RXOUT_*)
 *   [10..13] bind data: RLINK = the transmitter's 4-byte ID (the one it flies with afterwards); else 0
 *   [14..15] CRC-16 (RLINK's, CCITT 0x1021 from 0xFFFF) over bytes 0..13
 * A receiver that is searching (not linked, or binding) leaves its own channel for RXS_SLOT_US every
 * RXS_SLOT_EVERY_US and listens there. It acts on RXS_NEED identical packets in a row, heard at RXS_RSSI_MIN or
 * stronger (on the bench, a metre or so):
 *   bind plug on servo connector 8 at power-up (pairing): the sender becomes one of its owners (up to 3). The plug
 *   also makes the board a receiver for that power-up (RLINK, binding) when it is new or set up as a transmitter,
 *   so a board needs no jumpers and no wire even for its first setup;
 *   otherwise: only packets from an owner count.
 * It then stores mode + layout (cfg.c), and for RLINK the bind (rx_store slot RX_SLOT_RLINK), blinks and restarts.
 * v0.2: any receiver mode. The frame's Prot / Sub in RxSetup are the mode the receiver shall have: the TX module
 * restarts into the RLINK TX role to send them (bay_rxs_role) and back into the model's mode when RxSetup ends.
 * A receiver in any mode, switched on with the bind plug, first listens RXS_WINDOW_US for setup packets
 * (rxs_window), then starts its stored mode binding - so DSMX / D8 / D16 / nRF24 receivers are given a mode over
 * the air and then bound the usual way to any transmitter of that mode (a MiTo module, a Spektrum, a FrSky ...).
 * Only RLINK gets its bind in the setup packet (the MiTo TX module is the RLINK transmitter).
 * The TX module, when it starts sending, takes its private RLINK ID (as an RLINK bind would) so the receiver set
 * up with it links as soon as the transmitter leaves RxSetup.
 *
 * #included into app_process.c after rlink.c and cfg.c.
 */
#ifndef RX_SETUP_H
#define RX_SETUP_H
#include <stdint.h>
#include <stdbool.h>

#define RXS_CH            60u              /* 2460 MHz: RLINK's bind channel, but another sync word */
#define RXS_SYNC          0x5E7A1C39u
#define RXS_TYPE          0x5Au
#define RXS_POWER_DDBM    (-100)           /* -10 dBm: a bench affair */
#define RXS_TX_EVERY_US   3000u
#define RXS_SLOT_EVERY_US 100000u          /* receiver: a listening slot this often while searching ... */
#define RXS_SLOT_US       7000u            /* ... this long: two packets at least */
#define RXS_RSSI_MIN      (-60)            /* dBm */
#define RXS_NEED          3u
#define RXS_WINDOW_US     3000000u         /* bind plug at power-up: listen this long for setup packets before the role starts */

/* TX (rlink.c TX role): every loop, true while the encoder's frames ask for RxSetup */
void rxs_tx_request(bool on, uint8_t proto, uint8_t sub, uint8_t layout);
/* receiver (rlink.c RX role): 0 = not now, 1 = the slot owns the radio, 2 = the slot just ended (listen again) */
static uint8_t rxs_rx_slot(uint32_t now, bool searching);
/* receiver: a 16-byte packet heard while the slot listened */
static void rxs_rx_packet(const uint8_t *b, uint16_t n, int8_t rssi);
static bool rxs_rx_listening(void);              /* the slot is on: packets go to rxs_rx_packet */
/* TX: true = the setup owns the radio this loop (no flying data) */
static bool rxs_tx_service(uint32_t now);
#endif
