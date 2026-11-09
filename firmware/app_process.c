/***************************************************************************//**
 * @file app_process.c  — nRF24-protocol port, V4-CONSOLIDATED + FrSky D8 v1 + D16 EU v0 + RLINK
 * @brief EFR32MG24 RF module: PPM in -> Tobe's nRF24 250 kbps link out (+ F/S low = nRF24 receiver, nrf24_rx.c).
 *
 * Protocol logic ported VERBATIM from Rf_module_3_packet_tx.ino (20260521):
 *  - 9-byte payloads (primary/aux/bind), + 2-byte software CRC16 => 11-byte PHY frame
 *  - 41 hop channels: (2*i)+2 + rand&1, Fisher-Yates, xorshift32
 *    seed = pipe[0]+pipe[1]+pipe[2]; state = seed?(seed<<16)|seed:0xDEADBEEF
 *  - 11 ms frames, cadence: PRIMARY, PRIMARY, AUX
 *  - startup bind: ALWAYS first 1500 ms on BIND_PIPE, hop[0], PA min;
 *    bind button held keeps binding; release after hold -> system reset
 *  - failsafeFlag = F_S input (PB03, active low). PPM loss -> channels 1400 us.
 *  - RANGE input (PB02, active low) -> PA min in flight (range check)
 * Inputs: PPM PB01 (falling edges), BIND PB00 (simple_button), RANGE PB02, F_S PB03
 * SERIAL CONTROL (bay_serial.c): only a transmitter (F/S open) with the RANGE jumper fitted at power-up.
 *   Then the transmitter controls the module through its MULTI frames on PA06 (J3 RX)
 *   and overrides every other jumper: the frames' Prot / Sub choose the protocol, the frames carry the channels, BIND
 *   and RANGE (the BIND pin still works beside them; the RANGE pin is the "encoder" jumper here, not range check).
 *     Prot 3 = FrSky D8, 15 = FrSky D16 (Sub 0 v1 EU-LBT, 1 v2 EU-LBT, 2 v1 FCC, 3 v2 FCC, 5 v1 FCC 8 ch), 6 Sub 2 / 3 = DSMX 22 / 11 ms (11 ms only with DTX_EU_LOCK 0),
 *     0 Sub 0 = nRF24, 0 Sub 1 = RLINK, 0 Sub 7 = TX test; anything else: RF off, both LEDs blink, the log says why.
 *   The module waits for the first frames (both LEDs blink together) and restarts by itself when Prot / Sub
 *   change and when the frames' BIND comes on (bind is at power-up in every role). While BIND stays
 *   on, nRF24 and RLINK keep binding and FrSky D8 / D16 take turns of 1.5 s bind and 1.5 s flying data;
 *   DSMX sends its single 1 s bind burst (the Lemon receiver gives up on a longer one).
 *   No frames later: input lost, as without PPM.
 *   Every other jumper setting -- no RANGE jumper, or F/S fitted -- works exactly as before.
 *   A stored TX setup ("cfg tx", or the controller box) does the same with no jumpers at all: F/S open, the
 *   module waits for MULTI frames on PA06 or PPM on PB01 and the first to bring 3 frames wins until the next
 *   power-up -- MULTI = encoder control as above, PPM = the stored protocol and Opt. F/S fitted = MULTI only.
 * DUAL build (branch dual-serial): an SX1278 (RA-01) on PC06-PC09 + PD03 carries Tobe's UK 459 MHz link
 *   (uk459_tx.c) beside every transmitter role -- the same channels, BIND and RANGE as the 2.4 GHz side, from PPM or
 *   from the encoder's frames; with RLINK only while RLINK's link alarm is on --; it sleeps in DSM capture, TX test
 *   and, for now, the receiver roles. With the encoder the UK ID includes the Rx No too (per-model binding; Rx No 0 =
 *   the ID as before). The receivers' servo outputs have moved off the SX pins (servo_pins.h). The 2.4 GHz side is
 *   lemon-serial's, unchanged, so single and dual units work together on 2.4 GHz; the UK link is simply not there
 *   on a single unit. The single-radio builds are lemon-profile, lemon-serial and No_EU.
 ******************************************************************************/

#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include "sl_component_catalog.h"
#include "sl_rail.h"
#include "app_process.h"
#include "sl_rail_util_init.h"
#include "sl_rail_sdk_simple_assistance.h"
#include "sl_rail_sdk_packet_assistant.h"
#include "sl_simple_button_instances.h"
#include "sl_gpio.h"
#include "sl_device_gpio.h"
#include "sl_code_classification.h"
#include "em_device.h"
#include "em_eusart.h"
#include "sl_cli_instances.h"   // the command line is ticked while a transmitter waits for its encoder (bay_detect)
#include "sl_iostream.h"
#include "sl_iostream_init_eusart_instances.h"
#include <stdio.h>
#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif

// Per-model binding: the MULTI frame's "Rx No" field (the model's receiver number).
// It is mixed into every protocol's transmitter ID, so a receiver answers only the model it was bound with.
// 0 = no encoder (jumpers) or Rx No 0: nRF24, DSMX and RLINK then send exactly the IDs they sent before per-model binding.
static uint8_t rc_model = 0;

// Protocol modules, selected by the PROTOCOL jumpers. Pulled in directly so the
// generated cmake source list stays untouched.
#include "cfg.h"       // stored setup: role / protocol / option instead of jumpers (cfg.c below)

// "Store failsafe" from the encoder: a Multiprotocol failsafe frame (header 0x57) makes the transmitter roles that
// can carry it (RLINK: a flag in the frame, nRF24: the failsafe flag of the original protocol) ask the receiver to
// store the positions it receives, for 0.3 s. The receiver stores them once per request.
static volatile uint32_t fs_cmd_until = 0;
static inline bool fs_cmd_active(uint32_t now) { return fs_cmd_until != 0u && (int32_t)(now - fs_cmd_until) < 0; }
static volatile bool fs_cmd_log = false;
#include "rx_store.c"
#include "servo_out.c"   // the one servo / PPM output of every receiver role (TIMER2/3/4 compare outputs); dual: 459 MHz override
#include "sbus_out.c"
#include "nrf24_rx.c"
#include "dsm_cap.c"
#include "jeti_cap.c"   // Jeti Duplex sniffer: 802.15.4 frames on channels 11..26 -> UART (PC00 + PC02, or encoder Prot 0 Sub 6)
#include "dsm_rx.c"
// EU build: DSMX 22 ms only (dsm_tx.c v1.14). Set 0 for use outside the EU, where 11 ms is allowed.
#define DTX_EU_LOCK 1
#include "dsm_tx.c"
#include "tx_test.c"
#include "bay_serial.c" // encoder MULTI frames on PA06 (EUSART1 RX): protocol, channels, BIND, RANGE
#include "etx_multi.c"  // ... or an EdgeTX radio's (inverted): the module answers as a Multiprotocol module
#include "uk459_tx.c"   // second radio: SX1276/78, UK 459 MHz link beside the 2.4 GHz transmitter roles
#include "uk459_rx.c"   // ... and the 459 MHz receiver role (PC01 + PC02 + F/S): V7 ported

// FrSky D8 / D16 power, same convention as nRF24 / RLINK / DSMX: flight at the module maximum (200 = +20 dBm
// requested; RAIL caps it at what the PA delivers), bind & RANGE -10 dBm.
// Literals on purpose: frsky_d8.c / frsky_d16.c use these at file scope, before TX_POWER_*_DDBM exist.
#define FRSKY_DATA_POWER_DDBM    200    // = TX_POWER_MAX_DDBM
#define FRSKY_BIND_POWER_DDBM   -100    // = TX_POWER_MIN_DDBM
#define FRSKY_RANGE_POWER_DDBM  -100    // = TX_POWER_MIN_DDBM
#define FRSKY16_DATA_POWER_DDBM  200    // = TX_POWER_MAX_DDBM
#define FRSKY16_BIND_POWER_DDBM -100    // = TX_POWER_MIN_DDBM
#define FRSKY16_RANGE_POWER_DDBM -100   // = TX_POWER_MIN_DDBM
#define FRSKY16_LBT_POWER_DDBM  -100    // = TX_POWER_MIN_DDBM
#include "frsky_d8.c"
#include "frsky_d16.c"
#include "rlink.c"

// ---------------------------------------------------------------- protocol --
#define PAYLOAD_MAX        9u
#define FRAME_LEN          (PAYLOAD_MAX + 2u)     // + software CRC16
#define num_channels       41u
#define FRAME_PERIOD_US    11000u
#define STARTUP_BIND_MS    1500u
#define STARTUP_BIND_FRAMES ((uint16_t)(STARTUP_BIND_MS / 11u))

#define PKT_PRIMARY  0
#define PKT_BIND     1
#define PKT_AUX      2

#define TX_POWER_MAX_DDBM   200    // flight: module maximum (+20 dBm requested, RAIL caps it); was 0 = nRF24 PA_MAX parity
#define TX_POWER_MIN_DDBM  -100    // -10.0 dBm ("PA_MIN": bind & range check)

/// nRF24 air order: address MSByte-first, MSbit-first. EFR32 sync register
/// transmits least-significant BIT first => load addr[0]..addr[3] MSB->LSB
/// bytes with each byte bit-reversed. Flip these only if bind fails:
#define ADDR_BYTE_REVERSE  0
#define ADDR_BIT_REVERSE   1

/// 1 = read BIND (PB00, active low) to extend/force bind; 0 = bind only via the
/// 1.5 s boot window (the proven-working bench state; set 1 once PB00 hardware
/// is verified: 3.3 V idle, 0 V when shorted to its GND pin).
#define BIND_PIN_ENABLED   0

static const uint8_t BIND_PIPE[4] = { '0', '0', '0', 0x00 };

// ------------------------------------------------------------- packet types -
typedef struct { uint8_t packetType, failsafeFlag, data[7]; }      PrimaryPacket;
typedef struct { uint8_t packetType, ch6, ch7, ch8, pad[5]; }      AuxPacket;
typedef struct { uint8_t packetType, failsafeFlag, pipe[4], pad[3];} BindPacket;

static PrimaryPacket primPkt = { PKT_PRIMARY, 0, {0} };
static AuxPacket     auxPkt  = { PKT_AUX, 0, 0, 0, {0} };
static BindPacket    bindPkt = { PKT_BIND, 0, {0}, {0} };

// ------------------------------------------------------------------- state --
static uint8_t  pipeOut[4]   = {0};
static uint8_t  hopChannels[num_channels];
static uint8_t  hopIndex     = 0;
static bool     buttonPressed = false;   // bind button state at boot / while held
static bool     startupBind  = true;
static uint8_t  frameCount   = 0;
static uint8_t  frameSeq     = 0;
static uint16_t startupFrames = 0;
static uint8_t  frame_buf[FRAME_LEN];

volatile bool tx_requested = false;      // kept for app_cli.c compatibility
volatile bool rx_requested = true;       // kept for app_cli.c compatibility

static volatile bool     frame_due   = false;
static volatile bool     packet_sent = false;
static volatile bool     tx_err      = false;
static volatile bool     rx_held     = false;   // a received packet is held in RAIL
static volatile bool     cal_err     = false;
static volatile bool     cal_pending = false;   // RAIL wants a recalibration; done from the main loop, not the ISR
volatile uint32_t        cal_count   = 0;       // calibrations actually run (diagnostic: dsm_tx.c/dsm_rx.c can print it)
static volatile uint64_t radio_events = 0;
static volatile sl_rail_status_t calibration_status = 0;

// PPM (updated in GPIO IRQ) — verbatim semantics from the AVR ICP1 ISR
static volatile uint16_t contrl_pos[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
static volatile uint16_t scaled10[5];
static volatile uint8_t  scaled8[3];
static volatile uint8_t  ppmCounter = 0;
static volatile uint32_t ppm_last_edge = 0;
static volatile uint32_t ppm_last_sync = 0;   // for loss detection
static volatile uint32_t ppm_syncs = 0;       // sync gaps seen, total (bay_detect: PPM or MULTI, whichever comes first)
// PPM diagnostic: same glitch seen in nRF24 mode too, so look at the encoder's own PPM signal,
// independent of which RF protocol is running. Printed once a second from app_process_action().
static uint16_t          ppm_prev_frame[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
static volatile uint32_t ppm_frame_period_us = 0;    // time between successive sync gaps (nominal ~22500)
static volatile uint16_t ppm_worst_jump      = 0;    // biggest single-channel change frame-to-frame, us
static volatile uint32_t ppm_frames          = 0;    // frames (sync gaps) seen, total
static volatile uint32_t ppm_worst_period    = 0;    // furthest a frame period has been from 22500 us
#ifndef PPM_DIAG
#define PPM_DIAG 1
#endif

static sl_gpio_t range_pin = { .port = SL_GPIO_PORT_B, .pin = 2 }; // RANGE, act.low
static sl_gpio_t fs_pin    = { .port = SL_GPIO_PORT_B, .pin = 3 }; // F_S,   act.low
static sl_gpio_t bind_pin  = { .port = SL_GPIO_PORT_B, .pin = 0 }; // BIND,  act.low
static sl_gpio_t led_a     = { .port = SL_GPIO_PORT_C, .pin = 4 }; // LED_A: TX activity
static sl_gpio_t led_b     = { .port = SL_GPIO_PORT_C, .pin = 5 }; // LED_B: PPM activity
static sl_gpio_t proto0_pin = { .port = SL_GPIO_PORT_C, .pin = 0 }; // PROTOCOL_00, act.low
static sl_gpio_t proto1_pin = { .port = SL_GPIO_PORT_C, .pin = 1 }; // PROTOCOL_01, act.low
static sl_gpio_t proto2_pin = { .port = SL_GPIO_PORT_C, .pin = 2 }; // PROTOCOL_02, act.low
// Protocol select at boot: all floating -> nRF24 (+ F/S low = nRF24 receiver); PC02 low -> RLINK (PC01 ignored);
// PC00 low -> FrSky D8; PC01 low -> FrSky D16 (v1 EU-LBT)
typedef enum { PROTO_NRF24 = 0, PROTO_D8, PROTO_D16, PROTO_RLINK, PROTO_DSMCAP, PROTO_DSMRX, PROTO_DSMTX, PROTO_TXTEST, PROTO_JETICAP, PROTO_UKONLY, PROTO_UKRX } proto_t;
static proto_t proto = PROTO_NRF24;
static bool    nrf_rx_role = false;      // nRF24 + F/S low at boot: receiver role (nrf24_rx.c)
static bool    uk_have_sx  = false;      // DUAL: an SX1278 answered at power-up
static bool    uk_dual_rx  = false;      // DUAL receiver: uk459_rx.c runs beside the 2.4 GHz receiver role
static uint32_t dual_rx_switches = 0;    // times the servos were handed to the 459 MHz data
#define DUAL_RX_SWITCH_US   100000u      // 2.4 GHz silent this long -> servos from 459 MHz
#define DUAL_RX_UK_HOLD_US 1200000u      // 459 MHz data counts this long after its last packet (V7 failsafe time)
static bool    uk_link_on  = false;      // SX fitted and a transmitter role: uk459_tx.c runs beside it
static bool    is_rx       = false;      // receiver role: F/S low, or the stored setup says RX (cfg.c)
static bool    cfg_used    = false;      // a stored setup chose role / protocol / input at this power-up
static cfg_t   cfg_boot;                 // ... and this is it
#define alt_mode (proto != PROTO_NRF24)

// Encoder serial control (bay_serial.c)
static bool          bay_listen      = false;   // transmitter role: EUSART1 listens on PA06
static volatile bool bay_mode        = false;   // MULTI frames at power-up: the encoder chose the role and feeds it
static bool          bay_unsupported = false;   // ... but asked for a Prot / Sub this module does not have: RF off
static bool          bay_bind_now    = false;   // BIND on in the frames
static bool          bay_range_now   = false;   // RANGE on in the frames
static bay_frame_t   bay_boot;                  // the frame the role was chosen from
#include "cfg.c"       // the "cfg" command and the stored setup (flash page 0x08042000)
#include "rx_setup.c"  // receiver setup over the air (RxSetup): setup packets on RLINK's PHY
static bool          bay_rxsetup_now = false;   // RxSetup on in the frames (byte 26 bit 3)
static uint8_t       bay_rxs_layout  = 0;       // ... with this receiver output layout (byte 26 bits 0..1)
static bool          bay_rxs_role    = false;   // started for RxSetup: the RLINK TX role sends setup packets for the encoder's Prot / Sub

// Direct-GPIO LEDs: the SDK's toggle_send_led() is a NO-OP unless the LED
// instance is named "led0" (SL_CATALOG_LED0_PRESENT) — ours is inst0.
static inline void led_a_toggle(void) { sl_gpio_toggle_pin(&led_a); }
static inline void led_b_toggle(void) { sl_gpio_toggle_pin(&led_b); }

static void start_frame_timer(void);

// ------------------------------------------------ VERBATIM: PRNG + hop table
static uint32_t xorshift32(uint32_t* state)
{
  uint32_t x = *state;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  return *state = x;
}

static uint16_t generateSeedFromPipe(const uint8_t* pipe)
{
  uint16_t seed = 0;
  for (int i = 0; i < 3; ++i) seed += pipe[i];
  return seed;
}

static void generateAndShuffleChannels(uint16_t seed)
{
  uint32_t state = seed ? ((uint32_t)seed << 16) | seed : 0xDEADBEEFUL;
  for (uint8_t i = 0; i < num_channels; ++i)
    hopChannels[i] = (uint8_t)((2u * i) + 2u + (uint8_t)(xorshift32(&state) & 1u));
  for (uint8_t i = num_channels - 1u; i > 0; --i) {
    uint8_t j = (uint8_t)(xorshift32(&state) % (i + 1u));
    if (j != i) { uint8_t t = hopChannels[j]; hopChannels[j] = hopChannels[i]; hopChannels[i] = t; }
  }
}

// --------------------------------------- VERBATIM-style: pipe from unique ID
// AVR hashed its 9 signature bytes (*31) + EEPROM hash byte. EFR32: hash the
// 8-byte EUI64 the same way; the hash byte is the model number (per-model binding, as the
// ATmega encoder's per-model hash byte; 0 without the encoder). Receivers learn the
// pipe at bind, so the value only needs to be stable & unique per module and model.
static void generatePipeFromUniqueIDAndHash(uint8_t* pipe)
{
  uint32_t uidw[2] = { DEVINFO->EUI64L, DEVINFO->EUI64H };
  const uint8_t *uid = (const uint8_t *)uidw;
  uint32_t combined = 0;
  for (int i = 0; i < 8; i++) combined = (combined * 31u) + uid[i];
  combined = (combined * 31u) + rc_model;         // hash byte = model number (0 = no encoder)
  for (int i = 0; i < 3; i++) pipe[i] = (uint8_t)((combined >> (i * 8)) & 0xFF);
  pipe[3] = 0x00;
}

// ------------------------------------------------- VERBATIM: scaling + pack
static inline uint16_t scaleTo10(uint16_t raw)
{ return (uint16_t)(((uint32_t)(raw - 900u) * 1023u + 600u) / 1200u); }
static inline uint8_t scaleTo8(uint16_t raw)
{ return (uint8_t)(((uint32_t)(raw - 900u) * 255u + 600u) / 1200u); }

static void packPrimary(volatile uint16_t* ch, uint8_t* out)
{
  out[0]  = (uint8_t)(ch[0] & 0xFF);
  out[1]  = (uint8_t)((ch[0] >> 8) & 0x03);
  out[1] |= (uint8_t)((ch[1] << 2) & 0xFF);
  out[2]  = (uint8_t)((ch[1] >> 6) & 0x0F);
  out[2] |= (uint8_t)((ch[2] << 4) & 0xFF);
  out[3]  = (uint8_t)((ch[2] >> 4) & 0x3F);
  out[3] |= (uint8_t)((ch[3] << 6) & 0xFF);
  out[4]  = (uint8_t)((ch[3] >> 2) & 0xFF);
  out[5]  = (uint8_t)(ch[4] & 0xFF);
  out[6]  = (uint8_t)((ch[4] >> 8) & 0x03);
}

// -------------------------------------------------------- CRC16 over frame --
static uint16_t crc16_ccitt(uint16_t crc, uint8_t b)
{
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
  return crc;
}

static void append_crc(uint8_t *frame, const uint8_t addr[4])
{
  uint16_t crc = 0xFFFFu;
  for (int8_t i = 3; i >= 0; i--)          // address MSByte-first on air
    crc = crc16_ccitt(crc, addr[i]);       // (register order is LSByte-first)
  for (uint8_t i = 0; i < PAYLOAD_MAX; i++)
    crc = crc16_ccitt(crc, frame[i]);
  frame[PAYLOAD_MAX]      = (uint8_t)(crc >> 8);
  frame[PAYLOAD_MAX + 1u] = (uint8_t)(crc);
}

// ----------------------------------------------------- sync word (address) --
static uint8_t bit_reverse8(uint8_t b)
{
  b = (uint8_t)((b >> 4) | (b << 4));
  b = (uint8_t)(((b & 0xCCu) >> 2) | ((b & 0x33u) << 2));
  b = (uint8_t)(((b & 0xAAu) >> 1) | ((b & 0x55u) << 1));
  return b;
}

static void set_air_address(const uint8_t addr[4])
{
  uint32_t w = 0;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t idx = ADDR_BYTE_REVERSE ? (uint8_t)(3u - i) : i;
    uint8_t b   = addr[idx];
    if (ADDR_BIT_REVERSE) { b = bit_reverse8(b); }
    w = (w << 8) | b;
  }
  sl_rail_handle_t rh = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
  sl_rail_idle(rh, SL_RAIL_IDLE, true);   // sync words only reconfigure when idle
  sl_rail_sync_word_config_t sc = { .sync_word_bits = 32, .sync_word_0 = w, .sync_word_1 = 0 };
  sl_rail_status_t st = sl_rail_config_sync_words(rh, &sc);
  (void)st;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log_info("addr %02X%02X%02X%02X sync=0x%08" PRIX32 " st=%lu\n",
               addr[0], addr[1], addr[2], addr[3], w, (unsigned long)st);
#endif
}

// -------------------------------------------------------------- PPM capture -
// The pulse edges are timestamped by TIMER0 CC0 hardware capture (PB01 -> EXTI -> PRS ch 7 ->
// TIMER0 CC0, 1 MHz ticks): the time latches in silicon at the edge, like the AVR's ICP1, so
// interrupt latency cannot touch the measurement. Software ISR timestamps showed 65-99 us of
// jitter with the sticks held still, in every RF mode; raising the GPIO interrupt priority
// changed nothing because RAIL and the SDK disable interrupts outright for stretches, and no
// NVIC priority helps against that -- only a hardware capture is immune.
static uint32_t ppm_cnt_mask = 0xFFFFFFFFu;     // TIMER0 counter span (TOP), for wrap-safe diffs
static bool     ppm_sync_seen = false;

static void ppm_capture(uint32_t now)           // now = the edge's captured time, 1 MHz ticks
{
  uint32_t dt = (now - ppm_last_edge) & ppm_cnt_mask;
  ppm_last_edge = now;

  if (dt > 3500u) {                       // sync gap (verbatim threshold)
    if (bay_mode) { ppmCounter = 0; return; }   // the channels come from the encoder's MULTI frames
    ppm_syncs++;
#if PPM_DIAG
    static uint32_t last_sync_tick = 0;
    if (ppm_sync_seen) {
      uint32_t period = (now - last_sync_tick) & ppm_cnt_mask;
      uint32_t prev = ppm_frame_period_us;
      ppm_frame_period_us = period;
      if (prev != 0u) {                   // period jitter: this frame period vs the previous one
        int32_t poff = (int32_t)period - (int32_t)prev;
        if (poff < 0) poff = -poff;
        if ((uint32_t)poff > ppm_worst_period) ppm_worst_period = (uint32_t)poff;
      }
      uint16_t worst = ppm_worst_jump;
      for (uint8_t i = 0; i < 8u; i++) {
        int32_t d = (int32_t)contrl_pos[i] - (int32_t)ppm_prev_frame[i];
        if (d < 0) d = -d;
        if ((uint16_t)d > worst) worst = (uint16_t)d;
        ppm_prev_frame[i] = contrl_pos[i];
      }
      ppm_worst_jump = worst;
      ppm_frames++;
    }
    last_sync_tick = now;
#endif
    ppm_sync_seen = true;
    ppmCounter = 0;
    sl_rail_handle_t rh = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
    ppm_last_sync = (uint32_t)sl_rail_get_time(rh);   // RAIL timebase: only the 100 ms loss check uses it
    led_b_toggle();                       // PPM frame heartbeat
  } else {
    if (bay_mode) return;
    uint16_t v = (uint16_t)dt;
    if (v > 2100u) v = 2100u;
    if (v < 900u)  v = 900u;
    contrl_pos[ppmCounter] = v;
    if (ppmCounter < 5u)      scaled10[ppmCounter]      = scaleTo10(v);
    else if (ppmCounter < 8u) scaled8[ppmCounter - 5u]  = scaleTo8(v);
    if (++ppmCounter > 7u) ppmCounter = 7u;
  }
}

void TIMER0_IRQHandler(void)              // one capture per PPM edge, ~45 x 9 per second
{
  while (TIMER_IntGet(TIMER0) & TIMER_IF_CC0) {
    TIMER_IntClear(TIMER0, TIMER_IF_CC0);
    ppm_capture(TIMER_CaptureGet(TIMER0, 0));
  }
}

static void ppm_edge_cb(uint8_t interrupt_no, void *ctx)
{ (void)interrupt_no; (void)ctx; }        // never fires: the EXTI routing feeds PRS, its interrupt is off

static void ppm_check_loss(uint32_t now)
{
  // verbatim behaviour: ~3 missed frames -> park everything at 1400 us
  // signed difference. 'now' is read before an input interrupt may set ppm_last_sync a little later; the
  // unsigned difference then wrapped to ~4295 s and parked every channel at 1400 us until the next frame - the
  // servo jitter of every mode (FrSky: 50..110 such steps of 552 us per 2 s, sticks still).
  if ((int32_t)(now - ppm_last_sync) > 100000) {  // 100 ms without a sync gap
    for (uint8_t i = 0; i < 8; ++i) contrl_pos[i] = 1400u;
    for (uint8_t i = 0; i < 5; ++i) scaled10[i] = scaleTo10(1400u);
    for (uint8_t i = 0; i < 3; ++i) scaled8[i]  = scaleTo8(1400u);
  }
}

static void inputs_init(void)
{
  sl_gpio_t ppm_pin = { .port = SL_GPIO_PORT_B, .pin = 1 };      // PB01
  sl_gpio_set_pin_mode(&ppm_pin, SL_GPIO_MODE_INPUT_PULL, 1);
  int32_t int_no = SL_GPIO_INTERRUPT_UNAVAILABLE;
  sl_gpio_configure_external_interrupt(&ppm_pin, &int_no,
                                       SL_GPIO_INTERRUPT_FALLING_EDGE,
                                       (sl_gpio_irq_callback_t)ppm_edge_cb, NULL);
  // Hardware timestamping (see ppm_capture above). The call above routed PB01 to EXTI line
  // int_no; that same routing feeds the pin level to PRS. The software edge interrupt is
  // turned off -- TIMER0 CC0 latches each falling edge in hardware and its ISR reads the
  // captured time, so no interrupt latency or critical section can jitter the pulse widths.
  if (int_no >= 0) {
    GPIO->IEN_CLR = (1u << (uint32_t)int_no);
    CMU_ClockEnable(cmuClock_PRS, true);
    CMU_ClockEnable(cmuClock_TIMER0, true);
    PRS->ASYNC_CH[7].CTRL = PRS_ASYNC_CH_CTRL_FNSEL_A | PRS_ASYNC_CH_CTRL_SOURCESEL_GPIO
                          | ((uint32_t)int_no << _PRS_ASYNC_CH_CTRL_SIGSEL_SHIFT);
    uint32_t tfreq = CMU_ClockFreqGet(cmuClock_TIMER0);
    TIMER_Init_TypeDef ti = TIMER_INIT_DEFAULT;
    ti.enable   = false;
    ti.prescale = (TIMER_Prescale_TypeDef)(tfreq / 1000000u - 1u);   // 1 MHz ticks (39 MHz -> /39)
    TIMER_Init(TIMER0, &ti);
    TIMER_TopSet(TIMER0, 0xFFFFFFFFu);        // free-running (a 16-bit timer clips TOP; the mask below copes)
    TIMER_InitCC_TypeDef cc = TIMER_INITCC_DEFAULT;
    cc.mode         = timerCCModeCapture;
    cc.edge         = timerEdgeFalling;
    cc.eventCtrl    = timerEventEveryEdge;
    cc.prsInput     = true;
    cc.prsInputType = timerPrsInputAsyncLevel;
    cc.prsSel       = 7;
    TIMER_InitCC(TIMER0, 0, &cc);
    ppm_cnt_mask = TIMER_TopGet(TIMER0);
    TIMER_IntClear(TIMER0, TIMER_IF_CC0);
    TIMER_IntEnable(TIMER0, TIMER_IEN_CC0);
    NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_SetPriority(TIMER0_IRQn, 2);
    NVIC_EnableIRQ(TIMER0_IRQn);
    TIMER_Enable(TIMER0, true);
  }
  sl_gpio_set_pin_mode(&range_pin, SL_GPIO_MODE_INPUT_PULL, 1);  // pull-ups
  sl_gpio_set_pin_mode(&fs_pin,    SL_GPIO_MODE_INPUT_PULL, 1);
  sl_gpio_set_pin_mode(&bind_pin,  SL_GPIO_MODE_INPUT_PULL, 1);
  sl_gpio_set_pin_mode(&led_a,     SL_GPIO_MODE_PUSH_PULL, 0);   // LEDs
  sl_gpio_set_pin_mode(&led_b,     SL_GPIO_MODE_PUSH_PULL, 0);
}

static bool pin_low(const sl_gpio_t *p)
{
  bool v = true;
  sl_gpio_get_pin_input(p, &v);
  return !v;
}

// BIND / RANGE of the transmitter roles: the pins, or BIND / RANGE in the serial frames
static uint32_t cfg_bind_until = 0;             // "cfg bind": BIND counts as held until this RAIL time (0 = no)
static bool bind_held(void)                     // the BIND jumper, or the virtual one after "cfg bind"
{
  if (pin_low(&bind_pin)) return true;
  if (cfg_bind_until != 0u) {
    uint32_t now = (uint32_t)sl_rail_get_time(sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0));
    if ((int32_t)(now - cfg_bind_until) < 0) return true;
    cfg_bind_until = 0;
  }
  return false;
}
static bool bind_in(void)  { return bind_held() || (bay_mode && bay_bind_now); }

// ------------------------------------------------- receiver outputs and the bind plug on servo connector 8 --
// Every receiver role: the stored output layout (cfg rx <proto> 5sbus ...: SBUS on servo connector 6, PPM on 7),
// and servo connector 8 read at power-up: a bind plug there (signal to GND) = BIND held for 8 s, as "cfg bind".
// The connector stays an input while the plug is in (a servo output into the plug's short would be a short);
// once the plug has been out for RXP_FREE_US it becomes servo 8 again.
#define RXP_FREE_US  300000u
static const sl_gpio_t rxp_pin = { .port = SL_GPIO_PORT_B, .pin = 5 };   // servo connector 8 (servo_pins.h)
static bool     rxp_held;                       // the plug was in at power-up and connector 8 is still an input
static uint32_t rxp_high_since;
// The plug, read once at power-up before the role is chosen: it also makes a board a receiver (proto_start)
static bool rxp_read(sl_rail_handle_t rh)
{
  sl_gpio_set_pin_mode(&rxp_pin, SL_GPIO_MODE_INPUT_PULL, 1);
  uint32_t t0 = (uint32_t)sl_rail_get_time(rh);
  while (((uint32_t)sl_rail_get_time(rh) - t0) < 300u) { }          // let the pull-up settle
  uint8_t low = 0;
  for (uint8_t i = 0; i < 8u; i++) { if (pin_low(&rxp_pin)) low++; t0 = (uint32_t)sl_rail_get_time(rh); while (((uint32_t)sl_rail_get_time(rh) - t0) < 100u) { } }
  return low == 8u;
}
static void rx_outputs_boot(sl_rail_handle_t rh, uint8_t layout, bool plug)
{
  servo_out_layout(layout);
  if (layout & RXOUT_SBUS) sbus_set_pin(SL_GPIO_PORT_A, 8);         // SBUS on connector 6 (PA08) instead of PA03
  rxp_held = plug;
  rxs_pairing = rxp_held;                                           // pairing: a TX module sending setup packets becomes an owner
  if (rxp_held) {
    servo_out_hold(7, true);                                        // connector 8 stays an input until the plug is out
    cfg_bind_until = ((uint32_t)sl_rail_get_time(rh) + 8000000u) | 1u;
    rxp_high_since = 0;
  }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("RXOUT outputs %s%s\r\n", servo_out_layout_name(layout),
          rxp_held ? "; bind plug on servo connector 8: BIND for 8 s, connector 8 an input until the plug is out" : "");
#endif
}
static void rx_outputs_service(uint32_t now)    // main loop, receiver roles: connector 8 back to servo 8 once the plug is out
{
  if (!rxp_held) return;
  if (pin_low(&rxp_pin)) { rxp_high_since = 0; return; }
  if (rxp_high_since == 0u) { rxp_high_since = now | 1u; return; }
  if ((now - rxp_high_since) < RXP_FREE_US) return;
  rxp_held = false;
  servo_out_hold(7, false);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("RXOUT bind plug out: servo connector 8 is servo 8 again\r\n");
#endif
}
static bool range_in(void) { return bay_mode ? bay_range_now : pin_low(&range_pin); }   // encoder mode: the pin is the mode jumper

// ------------------------------------------------------- encoder serial (bay) --
#define BAY_WAIT_LOG_MS  3000u    // encoder mode, no frames yet: a reminder in the log this often
#define BAY_CHANGE_MS     500u    // a new Prot / Sub must hold this long before the module restarts with it
#define BAY_STALE_US   500000u    // no frame for this long: the encoder's BIND / RANGE count as released
#define BAY_LOG_US    5000000u    // encoder mode: the BAY status line this often (a long log line delays the radio loop)
#define BAY_POL_TRY_MS     60u    // looking for frames: normal / inverted in turn, this long each (frames every 7 or 22 ms)
#define BAY_RESEARCH_MS  1000u    // running: frames gone this long -> look both ways up again (another radio plugged in)

// Prot / Sub -> role. Numbers as in Multiprotocol where one exists; 0 = Tobe's own links.
// Prot / Sub -> this module's mode, transmitter side (the serial frames, or a stored TX setup) or receiver side (a stored RX setup).
static bool cfg_map(uint8_t prot, uint8_t sub, bool rx, proto_t *p)
{
  switch (prot) {
    case 0u:
      if (sub == 0u) { *p = PROTO_NRF24;  return true; }
      if (sub == 1u) { *p = PROTO_RLINK;  return true; }
      if (sub == 2u) { *p = rx ? PROTO_UKRX : PROTO_UKONLY; return true; }   // UK 459 MHz receiver / UK only TX (2.4 GHz idle)
      if (sub == 4u && rx) { *p = PROTO_DSMCAP; return true; }               // DSM capture tool
      if (sub == 6u) { *p = PROTO_JETICAP; return true; }                    // Jeti Duplex sniffer (jeti_cap.c): listens only
      if (sub == 7u && !rx) { *p = PROTO_TXTEST; return true; }
      return false;
    case 3u:  *p = PROTO_D8;  return true;                                        // Multi "FrSkyD"
    case 6u:  if (sub == 2u || (sub == 3u && !DTX_EU_LOCK)) { *p = rx ? PROTO_DSMRX : PROTO_DSMTX; return true; }   // Multi "DSM": DSMX 22 / 11 ms
              return false;                                                     // 11 ms: not in the EU build
    case 15u: *p = PROTO_D16; return true;                                        // Multi "FrSkyX", any Sub
    default:  return false;
  }
}
static bool bay_map(const bay_frame_t *f, proto_t *p) { return cfg_map(f->proto, f->sub, false, p); }

static const char *bay_role_name(const bay_frame_t *f)
{
  proto_t p;
  if (!bay_map(f, &p)) return "not supported";
  switch (p) {
    case PROTO_NRF24:  return "nRF24";
    case PROTO_RLINK:  return "RLINK";
    case PROTO_TXTEST: return "TX test";
    case PROTO_JETICAP: return "Jeti capture";
    case PROTO_UKONLY: return "UK only (459 MHz, 2.4 GHz off)";
    case PROTO_D8:     return "FrSky D8";
    case PROTO_D16: {                               // the same Sub rule as frsky16_start
      static const char *const d16n[4] = { "FrSky D16 v1 EU-LBT", "FrSky D16 v2 EU-LBT", "FrSky D16 v1 FCC", "FrSky D16 v2 FCC" };
      return (f->sub == 5u) ? "FrSky D16 v1 FCC 8 ch" : d16n[f->sub & 3u]; }
    case PROTO_DSMTX:  return (f->sub == 3u) ? "DSMX 11 ms" : "DSMX 22 ms";
    default:           return "?";
  }
}

// Runs in the EUSART1 RX interrupt for every good frame.
static uint16_t bay_prev_us[8]; static bool bay_prev_ok;          // frame-to-frame channel jumps (> 100 us) in the encoder's frames
static volatile uint32_t bay_jumps; static volatile uint16_t bay_worst_jump;
static void bay_on_frame(const bay_frame_t *f)
{
  if (!bay_mode) return;                          // PPM / jumper mode: frames are only counted (bay_service)
  if (f->failsafe && f->edgetx) return;           // EdgeTX: every 7 s; the receivers here would store the sticks of the moment (etx_multi.h)
  if (f->failsafe) {                              // failsafe frame: the receiver shall store these positions
    fs_cmd_until = (f->t_us + 300000u) | 1u; fs_cmd_log = true;
    ppm_last_sync = f->t_us;
    return;                                       // not channel data; the next normal frame carries the same sticks
  }
  for (uint8_t i = 0; i < 8u; i++) {
    uint16_t v = f->us[i];
    if (v > 2100u) v = 2100u;                     // the PPM capture's limits
    if (v < 900u)  v = 900u;
    if (bay_prev_ok) {
      uint16_t d = v > bay_prev_us[i] ? v - bay_prev_us[i] : bay_prev_us[i] - v;
      if (d > bay_worst_jump) bay_worst_jump = d;
      if (d > 100u) bay_jumps++;
    }
    bay_prev_us[i] = v;
    contrl_pos[i] = v;
    if (i < 5u) scaled10[i] = scaleTo10(v); else scaled8[i - 5u] = scaleTo8(v);
  }
  bay_prev_ok = true;
  ppm_last_sync = f->t_us;                        // "input fresh" for every role, as a PPM sync gap
  led_b_toggle();                                 // frame heartbeat, as with PPM
}

static uint32_t bay_telem_sent;                 // telemetry frames sent to the encoder (bay_telemetry)

static void bay_log(const char *tag, const bay_frame_t *f)
{
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (f->edgetx) {
    app_log("%s EdgeTX protocol %u sub %u, receiver %u%s%s%s -> %s | ch %u %u %u %u %u %u %u %u | frames ok %ld, parity/framing %ld, overrun %ld | %lu MP frames to the radio\r\n",
            tag, f->wproto, f->wsub, f->rxnum, f->bind ? " BIND" : "", f->range ? " RANGE" : "", f->telem ? "" : " (telemetry off)",
            f->scan ? "protocol list" : bay_role_name(f),
            f->us[0], f->us[1], f->us[2], f->us[3], f->us[4], f->us[5], f->us[6], f->us[7],
            (long)bay_diag[BAY_OK], (long)bay_diag[BAY_ERR], (long)bay_diag[BAY_OVF], (unsigned long)etx_sent);
    bay_worst_jump = 0;
    return;
  }
  app_log("%s Model %u Prot %u Sub %u Opt %d%s%s -> %s | ch %u %u %u %u %u %u %u %u | frames ok %ld, header %ld (last %02lX), parity/framing %ld, overrun %ld, short %ld, jump %lu/%uus | telemetry %s, %lu sent on PB01\r\n",
          tag, f->rxnum, f->proto, f->sub, (int)f->option, f->bind ? " BIND" : "", f->range ? " RANGE" : "", bay_role_name(f),
          f->us[0], f->us[1], f->us[2], f->us[3], f->us[4], f->us[5], f->us[6], f->us[7],
          (long)bay_diag[BAY_OK], (long)bay_diag[BAY_HDR], (unsigned long)bay_diag[BAY_LASTHDR],
          (long)bay_diag[BAY_ERR], (long)bay_diag[BAY_OVF], (long)bay_diag[BAY_SHORT], (unsigned long)bay_jumps, (unsigned)bay_worst_jump,
          f->telem ? "asked (27-byte frame, bit 2)" : "not asked (26-byte frame, or bit 2 clear)", (unsigned long)bay_telem_sent);
  bay_worst_jump = 0;
#else
  (void)tag; (void)f;
#endif
}

static void bay_restart(const char *why)
{
  sl_rail_handle_t rh = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("BAY %s: restarting\r\n", why);
#else
  (void)why;
#endif
  uint32_t t0 = (uint32_t)sl_rail_get_time(rh);
  while (((uint32_t)sl_rail_get_time(rh) - t0) < 20000u) { }   // let the log line out
  NVIC_SystemReset();
}

// Encoder mode at power-up: wait for 3 good MULTI frames (*f = the latest), however long the encoder takes.
// Both LEDs blink together while it waits; the log reminds every BAY_WAIT_LOG_MS.
// Waits for the encoder's MULTI frames: true. ppm_too (a stored TX setup, F/S open): PPM frames on PB01 count as
// well, and whichever input brings 3 frames first wins - false = PPM, the stored protocol runs on it.
// The polarity is searched as well: normal (encoder) and inverted (EdgeTX) in turn, BAY_POL_TRY_MS each, until
// the frames check out (read the wrong way up they never do).
static bool bay_detect(sl_rail_handle_t rh, bay_frame_t *f, bool ppm_too)
{
  uint32_t t0 = (uint32_t)sl_rail_get_time(rh), tl = t0, tlog = t0, tpol = t0;
  uint32_t s0 = ppm_syncs;
  int32_t  ok0 = bay_diag[BAY_OK], good0 = bay_diag[BAY_GOOD];
  for (;;) {
    uint32_t now = (uint32_t)sl_rail_get_time(rh);
    if (bay_diag[BAY_OK] - ok0 >= 3 && bay_get(f)) return true;  // 3 good frames: about 66 ms (EdgeTX 21 ms)
    if (bay_diag[BAY_GOOD] != good0) { good0 = bay_diag[BAY_GOOD]; tpol = now; }   // frames this way up: stay
    if (bay_diag[BAY_OK] == ok0 && (uint32_t)(now - tpol) >= BAY_POL_TRY_MS * 1000u) {   // none yet: the other way up
      tpol = now;
      bay_set_inverted(!bay_inverted());
    }
    if (ppm_too && (uint32_t)(ppm_syncs - s0) >= 3u) return false;   // 3 PPM frames: about 66 ms
    sl_cli_instances_tick();                                     // the "cfg" command works while we wait (programmer on J3)
    if ((uint32_t)(now - tl) >= 150000u) { tl = now; led_a_toggle(); led_b_toggle(); }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    if ((uint32_t)(now - tlog) >= BAY_WAIT_LOG_MS * 1000u) {
      tlog = now;
      app_log("BAY still waiting for MULTI frames on PA06%s (%lu s), normal or inverted: serial output -> J3 RX? EdgeTX: External RF = MULTI, bay pin 1 -> J3 RX? bytes %ld, header %ld (last %02lX), parity/framing %ld\r\n",
              ppm_too ? " or PPM on PB01" : "", (unsigned long)((uint32_t)(now - t0) / 1000000u), (long)bay_diag[BAY_BYTES], (long)bay_diag[BAY_HDR],
              (unsigned long)bay_diag[BAY_LASTHDR], (long)bay_diag[BAY_ERR]);
    }
#endif
  }
}

static bool proto_started = false;

// The telemetry frame to the encoder (bay_serial.h lists the bytes): what the running role knows right now.
// RLINK has the full set; FrSky D8 / D16 the receiver's A1 (its battery), RSSI both ways and the link state;
// the rest only the mode and the power. The 459 MHz link (dual) adds its state in every transmitter role.
static void bay_telem_fill(sl_rail_handle_t rh, uint32_t now, bay_telem_t *tp)
{
  bay_telem_t t;
  memset(&t, 0, sizeof t);
  t.proto = bay_boot.proto;
  t.sub = bay_boot.sub;
  t.power_dbm = (int8_t)(sl_rail_get_tx_power_dbm(rh) / 10);
  t.rssi_rx = BAY_NO_RSSI;
  t.lq = BAY_NO_LQ;
  t.binding = bay_bind_now;
  t.range = bay_range_now;
  t.uk_link = uk_link_on;
  t.unsupported = bay_unsupported;
  if (bay_unsupported) { t.link_alarm = 2; *tp = t; return; }

  switch (proto) {
    case PROTO_RLINK: {
      bool fresh = rlink_tele_age_us(now) <= RL_ALM_TELE_US;
      t.link       = fresh;
      t.bound      = rlink_ever_linked();
      t.binding    = bay_bind_now || (rlink_diag[R_STATE] & 16) != 0;
      t.failsafe   = fresh && (rlink_diag[R_T_FLAGS] & 1) != 0;
      t.link_alarm = (uint8_t)rlink_diag[R_ALARM];
      t.vbat_alarm = (uint8_t)rlink_diag[R_VB_ALM];
      t.vbat_sensor = rlink_diag[R_VBAT] > 0;
      t.vbat_10mv  = (uint16_t)(rlink_diag[R_VBAT] / 10);
      t.power_dbm  = (int8_t)(rlink_diag[R_PWR] / 10);
      t.lost       = (uint8_t)rlink_diag[R_T_LOST];
      if (fresh) {
        t.rssi_rx = (int8_t)rlink_diag[R_T_RSSIA];  t.rssi_rx_dbm = true;
        t.rssi_tx = (int8_t)rlink_diag[R_T_RSSI_UP]; t.rssi_tx_valid = true;
        t.lq      = (uint8_t)(rlink_diag[R_T_LQ] > 100 ? 100 : rlink_diag[R_T_LQ] < 0 ? 0 : rlink_diag[R_T_LQ]);
      }
      break;
    }
    case PROTO_D8:
    case PROTO_D16: {
      static bool ever;
      bool d16  = proto == PROTO_D16;
      bool link = d16 ? frsky16_link_up() : frsky_link_up();
      volatile int32_t *dg = d16 ? frsky16_diag : frsky_diag;   /* same layout: [11] battery on the A1 scale, [13] RX RSSI figure */
      if (link) ever = true;
      t.link    = link;
      t.bound   = ever;
      t.binding = bay_bind_now || (d16 ? (int)frsky16_phase() : (int)frsky_phase()) == 0;
      if (link) {
        int32_t r = dg[13];
        t.rssi_rx = (int8_t)(r > 127 ? 127 : r < 0 ? 0 : r);     /* the receiver's own figure, as FrSky shows it */
        t.rssi_tx = d16 ? frsky16_tele_rssi_dbm() : frsky_tele_rssi_dbm();  t.rssi_tx_valid = true;
        int32_t a1 = dg[11];                                      /* D8 A1 / X-series RxBt: 13.2 V full scale */
        if (a1 > 0) { t.vbat_sensor = true; t.vbat_10mv = (uint16_t)((a1 * 1320L + 127L) / 255L); }
        t.link_alarm = (r < 45) ? 1u : 0u;                        /* FrSky's usual "RSSI low" figure */
      } else t.link_alarm = 2;
      break;
    }
    default:                                                    /* nRF24, DSMX, UK only, the tools: no telemetry back */
      break;
  }
  if (uk_link_on) {
    t.uk_active = uk_diag[UK_ACTIVE] != 0;
    t.uk_hop    = (uint8_t)uk_diag[UK_HOP];
    if (uk_diag[UK_PHASE] == 1) t.binding = true;
    if (proto == PROTO_UKONLY) { t.link = uk_diag[UK_PHASE] == 2; t.bound = t.link; }
  }
  *tp = t;
}

static void bay_telemetry(sl_rail_handle_t rh, uint32_t now)       // the encoder's 16-byte frame
{
  bay_telem_t t;
  bay_telem_fill(rh, now, &t);
  if (bay_send_telemetry(&t)) bay_telem_sent++;
}

static void bay_etx_telemetry(sl_rail_handle_t rh, uint32_t now, const bay_frame_t *f)   // EdgeTX: "MP" frames
{
  bay_telem_t t;
  bay_telem_fill(rh, now, &t);
  uint8_t kind = ETX_TEL_NONE;
  if (!bay_unsupported) {
    if (proto == PROTO_D8) kind = ETX_TEL_HUB;
    else if (proto == PROTO_D16 || proto == PROTO_RLINK) kind = ETX_TEL_SPORT;
  }
  etx_service(f, now, !bay_unsupported && !f->scan, kind, &t);
}

// Main loop, encoder mode: follow BIND / RANGE, restart on a new Prot / Sub and when BIND comes on.
// Every BAY_LOG_US a "BAY" status line (in place of the PPM line). Each new frame is answered with telemetry.
static void bay_service(sl_rail_handle_t rh)
{
  static bool     marks = false;
  static uint32_t seen_seq, chg_since, log_us, pol_us;
  static uint8_t  bind_frames;
  bay_frame_t f;
  bool have  = bay_get(&f);
  uint32_t now = (uint32_t)sl_rail_get_time(rh);                 // AFTER the copy: a newer frame must not look old
  bool fresh = have && (uint32_t)(now - f.t_us) < BAY_STALE_US;
  if (!marks) { marks = true; log_us = now; }

  if (fresh && f.edgetx != bay_boot.edgetx) {                   // another kind of radio after a silence (bay_detect again)
    bay_log(f.edgetx ? "BAY now EdgeTX" : "BAY now the encoder", &f);
    bay_restart(f.edgetx ? "an EdgeTX radio on the bay serial" : "the encoder on the bay serial");
  }
  if (fresh) {
    if (f.seq != seen_seq) {
      seen_seq = f.seq;
      bay_range_now = f.range;                                   // RANGE followed at any time
      bay_rxsetup_now = f.rxsetup; bay_rxs_layout = f.rxlayout;  // RxSetup followed at any time
      if (bay_rxs_role && !f.rxsetup) bay_restart("RxSetup over: the model's own mode now");
      if (!bay_rxs_role && f.rxsetup && proto != PROTO_RLINK) bay_restart("RxSetup: setup packets for this Prot / Sub");
      if (f.bind) { if (bind_frames < 255u) bind_frames++; } else bind_frames = 0;
      bool b = (bind_frames >= 2u) || (bay_bind_now && f.bind);  // set: 2 frames in a row; clear: at once
      if (b && !bay_bind_now) bay_restart("BIND in the frames");   // every role binds at power-up
      bay_bind_now = b;
      if (f.proto != bay_boot.proto || f.sub != bay_boot.sub || f.rxnum != bay_boot.rxnum || f.option != bay_boot.option) {   // Opt too: single / dual
        if (chg_since == 0u) chg_since = now | 1u;
        else if ((uint32_t)(now - chg_since) >= BAY_CHANGE_MS * 1000u) { bay_log("BAY new", &f); bay_restart("model or Prot / Sub changed"); }
      } else chg_since = 0;
      if (f.edgetx) {                                            // EdgeTX: status, protocol list, telemetry on PB01 (bay pin 5)
        if (f.telem) {
          bay_tx_polarity(f.tel_inv);
          if (!bay_tx_on()) bay_tx_begin();
          bay_etx_telemetry(rh, now, &f);
        } else bay_tx_end();                                     // the model has telemetry off: pin 5 left alone
      } else if (proto_started && f.telem) {                     // the reply: 16 bytes on PB01
        if (!bay_tx_on()) bay_tx_begin();                        // (taken again after a silence)
        bay_telemetry(rh, now);
      }
    }
  } else {                                                       // encoder silent (TX off, cable out, MULT left)
    bay_bind_now = false; bay_range_now = false; bay_rxsetup_now = false; bind_frames = 0; chg_since = 0;
    bay_tx_end();                                                // PB01 back to an input: the encoder may send PPM again
    if (have && (uint32_t)(now - f.t_us) >= BAY_RESEARCH_MS * 1000u && (uint32_t)(now - pol_us) >= BAY_POL_TRY_MS * 1000u) {
      pol_us = now;                                              // look both ways up: the encoder or an EdgeTX radio may come
      bay_set_inverted(!bay_inverted());
    }
  }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if ((uint32_t)(now - log_us) >= BAY_LOG_US) {
    log_us = now;
    if (!have) app_log("BAY no frame\r\n");
    else if (!fresh) app_log("BAY no frame for %lu ms (encoder TX off or cable out?): input lost, as without PPM\r\n",
                             (unsigned long)((uint32_t)(now - f.t_us) / 1000u));
    else bay_log("BAY", &f);
  }
#endif
}

// What the module is doing right now, for "cfg" (cfg.c).
static const char *proto_name(proto_t p, bool rx)
{
  switch (p) {
    case PROTO_NRF24:   return rx ? "nRF24 receiver" : "nRF24";
    case PROTO_D8:      return rx ? "FrSky D8 receiver" : "FrSky D8";
    case PROTO_D16:     return rx ? "FrSky D16 receiver" : "FrSky D16";
    case PROTO_RLINK:   return rx ? "RLINK receiver" : "RLINK";
    case PROTO_DSMCAP:  return "DSM capture";
    case PROTO_DSMRX:   return "DSMX receiver";
    case PROTO_DSMTX:   return "DSMX";
    case PROTO_TXTEST:  return "TX test";
    case PROTO_JETICAP: return "Jeti capture";
    case PROTO_UKONLY:  return "UK only (459 MHz)";
    case PROTO_UKRX:    return "UK 459 MHz receiver";
    default:            return "?";
  }
}
static void cfg_blink(bool ok)                    /* "cfg" result on the LEDs, for a box without the reply wire */
{
  sl_rail_handle_t rh = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
  for (uint8_t k = 0; k < (ok ? 6u : 20u); k++) {                 /* stored: both on/off 3x at 300 ms; refused: alternate 20x at 100 ms */
    if (ok) { if (k & 1u) { sl_gpio_clear_pin(&led_a); sl_gpio_clear_pin(&led_b); } else { sl_gpio_set_pin(&led_a); sl_gpio_set_pin(&led_b); } }
    else    { if (k & 1u) { sl_gpio_set_pin(&led_a); sl_gpio_clear_pin(&led_b); } else { sl_gpio_clear_pin(&led_a); sl_gpio_set_pin(&led_b); } }
    uint32_t t0 = (uint32_t)sl_rail_get_time(rh);
    while (((uint32_t)sl_rail_get_time(rh) - t0) < (ok ? 300000u : 100000u)) { }
  }
  sl_gpio_clear_pin(&led_a); sl_gpio_clear_pin(&led_b);
}
static bool cfg_in_use(void)                      /* a transmitter that is flying its input must not be re-programmed by a shared wire */
{
  if (!proto_started || is_rx) return false;
  if (bay_mode) return true;                      /* encoder frames (its command line is off then anyway) */
  uint32_t now = (uint32_t)sl_rail_get_time(sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0));
  return (int32_t)(now - ppm_last_sync) <= 100000 && ppm_last_sync != 0u;   /* PPM frames within 100 ms (signed: see ppm_check_loss) */
}
static void cfg_running_line(void)
{
  if (!proto_started) { printf("CFG running now: still starting\r\n"); return; }
  printf("CFG running now: %s%s%s, input %s, F/S %s%s\r\n",
         bay_unsupported ? "RF off (unsupported mode)" : proto_name(proto, is_rx),
         uk_link_on ? " + 459 MHz link" : "", uk_dual_rx ? " + 459 MHz receiver beside it" : "",
         bay_mode ? "encoder serial" : (bay_listen ? "encoder serial (no frames yet)" : (is_rx ? "radio" : "PPM")),
         pin_low(&fs_pin) ? "low" : "high", cfg_used ? " (from the stored setup)" : " (jumpers)");
}

// -------------------------------------------------------- 11 ms frame timer -
static void frame_timer_cb(sl_rail_handle_t rh)
{
  (void)rh;
  frame_due = true;
  if (startupFrames < 0xFFFFu) { ++startupFrames; }
  start_frame_timer();
}

static void start_frame_timer(void)
{
  sl_rail_handle_t rh = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
  sl_rail_set_timer(rh, FRAME_PERIOD_US, SL_RAIL_TIME_DELAY,
                    (sl_rail_timer_callback_t)frame_timer_cb);
}

// ------------------------------------------------- a radio on J3 RX at power-up --
// A board stored as a receiver (or set by jumpers, with no RANGE jumper) does not listen for serial frames. Put
// into a radio bay it must still become the transmitter module, so at power-up it first looks at J3 RX (PA06):
//   floating (nothing there: a receiver in a model) -> no delay, the board starts as set up;
//   driven (a radio, an encoder or the controller box on it) -> up to BAY_PROBE_MS for MULTI frames, either
//   polarity. Good frames -> transmitter under serial control for this power-up; nothing is stored, so the board
//   is a receiver again when it goes back into a model.
#define BAY_PROBE_MS  2000u
static bool j3_rx_driven(sl_rail_handle_t rh)
{
  sl_gpio_t rx = { .port = SL_GPIO_PORT_A, .pin = 6 };
  bool up, down;
  sl_gpio_set_pin_mode(&rx, SL_GPIO_MODE_INPUT_PULL, 1);
  uint32_t t0 = (uint32_t)sl_rail_get_time(rh); while (((uint32_t)sl_rail_get_time(rh) - t0) < 300u) { }
  up = !pin_low(&rx);
  sl_gpio_set_pin_mode(&rx, SL_GPIO_MODE_INPUT_PULL, 0);
  t0 = (uint32_t)sl_rail_get_time(rh); while (((uint32_t)sl_rail_get_time(rh) - t0) < 300u) { }
  down = !pin_low(&rx);
  sl_gpio_set_pin_mode(&rx, SL_GPIO_MODE_INPUT_PULL, 1);
  return up == down;                                 // the pulls cannot move it: something drives the line
}

static bool bay_probe(sl_rail_handle_t rh)
{
  bay_init(rh, bay_on_frame);
  uint32_t t0 = (uint32_t)sl_rail_get_time(rh), tpol = t0;
  int32_t good0 = bay_diag[BAY_GOOD];
  for (;;) {
    uint32_t now = (uint32_t)sl_rail_get_time(rh);
    if (bay_diag[BAY_OK] >= 3) return true;                       // EUSART1 stays on: bay_detect carries on
    if ((uint32_t)(now - t0) >= BAY_PROBE_MS * 1000u) break;
    if (bay_diag[BAY_GOOD] != good0) { good0 = bay_diag[BAY_GOOD]; tpol = now; }
    else if ((uint32_t)(now - tpol) >= BAY_POL_TRY_MS * 1000u) { tpol = now; bay_set_inverted(!bay_inverted()); }
  }
  NVIC_DisableIRQ(EUSART1_RX_IRQn);
  NVIC_DisableIRQ(EUSART1_TX_IRQn);
  EUSART_IntDisable(EUSART1, EUSART_IEN_RXFL);
  EUSART_Enable(EUSART1, eusartDisable);           // PA06 back to the log UART alone
  return false;
}

// ----------------------------------------------------------------- startup --
static void proto_start(void)
{
  sl_rail_handle_t rh = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  { uint32_t resetCause = EMU->RSTCAUSE;
    EMU->CMD = EMU_CMD_RSTCAUSECLR;
    app_log("RESET cause=0x%08lX\r\n", (unsigned long)resetCause); }
#endif

  inputs_init();
  cfg_cli_init();                                   // "cfg" on the log UART from here on
  sl_gpio_set_pin_mode(&proto0_pin, SL_GPIO_MODE_INPUT_PULL, 1);
  sl_gpio_set_pin_mode(&proto1_pin, SL_GPIO_MODE_INPUT_PULL, 1);
  sl_gpio_set_pin_mode(&proto2_pin, SL_GPIO_MODE_INPUT_PULL, 1);
  // Who decides the role and the protocol (cfg.h):
  //   nothing stored: the jumpers, exactly as before (F/S = receiver, RANGE at power-up = encoder serial, PC00..PC02 = protocol)
  //   TX stored:      F/S low = encoder serial control; F/S high = whichever input comes first: the encoder's MULTI
  //                   frames (encoder control) or PPM (the stored protocol and Opt) - no jumper needed for either
  //   RX stored:      F/S low = the protocol jumpers, as before; F/S high = the stored protocol
  bool fs = pin_low(&fs_pin);
  cfg_used = cfg_load(&cfg_boot);
  if (cfg_bind_pending()) {                         // "cfg bind": this power-up binds as if the BIND jumper were fitted
    cfg_bind_consume();
    cfg_bind_until = ((uint32_t)sl_rail_get_time(rh) + 8000000u) | 1u;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("SETUP bind asked for (cfg bind): BIND counts as held for 8 s\r\n");
#endif
  }
  is_rx = cfg_used ? (cfg_boot.role == CFG_ROLE_RX) : fs;
  bool plug = rxp_read(rh);                         // bind plug on servo connector 8 (PB05, unconnected in the module bay)
  if (plug && !is_rx) {                             // a new board, or one set up as a transmitter: the plug makes it a receiver
    cfg_used = true;                                // for this power-up only (nothing is stored): RLINK receiver, binding and
    memset(&cfg_boot, 0, sizeof cfg_boot);          // listening for setup packets, so a TX module can pair with it and set it up
    cfg_boot.role = CFG_ROLE_RX; cfg_boot.proto = 0u; cfg_boot.sub = 1u;
    is_rx = true;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("SETUP bind plug on servo connector 8: RLINK receiver for this power-up, waiting for setup packets (RxSetup)\r\n");
#endif
  }
  bool force_serial = false;                        // a radio on J3 RX: transmitter under serial control, this power-up
  bool would_listen = cfg_used ? !is_rx : (!fs && pin_low(&range_pin));
  if (!plug && !would_listen && j3_rx_driven(rh)) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("SETUP J3 RX is driven: looking %u ms for MULTI frames before starting as %s\r\n", (unsigned)BAY_PROBE_MS, is_rx ? "a receiver" : "PPM transmitter");
#endif
    if (bay_probe(rh)) {
      force_serial = true;
      cfg_used = true;                              // for this power-up only (nothing is stored): a transmitter whose
      memset(&cfg_boot, 0, sizeof cfg_boot);        // protocol comes from the frames, as a stored TX with F/S low
      cfg_boot.role = CFG_ROLE_TX;
      is_rx = false; fs = true;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("SETUP MULTI frames on J3 RX (%s): transmitter module for this power-up, the stored setup is kept\r\n",
              bay_inverted() ? "inverted, an EdgeTX radio" : "normal polarity");
#endif
    }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    else app_log("SETUP no MULTI frames on J3 RX: starting as set up\r\n");
#endif
  }
  if (is_rx) rx_outputs_boot(rh, cfg_used ? cfg_boot.outputs : 0u, plug);   // output layout, bind plug on servo connector 8
  if (is_rx && plug) {                              // any receiver mode: first a look for setup packets, then the role binds
    rxs_window(rh, RXS_WINDOW_US);
    cfg_bind_until = ((uint32_t)sl_rail_get_time(rh) + 8000000u) | 1u;
  }
  bool jumpers = !cfg_used || (is_rx && fs);
  if (jumpers) {
    bool f = is_rx;                                 // the receiver side of each protocol
    // PC00 + PC01 = DSMX: transmitter (dsm_tx.c); + F/S = receiver (dsm_rx.c), like the other protocols; + PC02 = raw capture / sniffer (dsm_cap.c);
    // PC00 + PC01 + PC02 + F/S = TX test: continuous carrier for power measurement (tx_test.c)
    proto = (pin_low(&proto0_pin) && pin_low(&proto1_pin))
          ? (pin_low(&proto2_pin) ? (f ? PROTO_TXTEST : PROTO_DSMCAP) : f ? PROTO_DSMRX : PROTO_DSMTX)
          : pin_low(&proto2_pin) ? (pin_low(&proto0_pin) ? PROTO_JETICAP                  // PC00 + PC02: Jeti sniffer (jeti_cap.c)
                                   : (pin_low(&proto1_pin) && f) ? PROTO_UKRX               // PC01 + PC02 + F/S: UK 459 MHz receiver (uk459_rx.c)
                                   : PROTO_RLINK)                                        // PC02 (PC01 ignored on a TX): RLINK
          : pin_low(&proto0_pin) ? PROTO_D8 : pin_low(&proto1_pin) ? PROTO_D16 : PROTO_NRF24;
    nrf_rx_role = (proto == PROTO_NRF24) && f;
  } else if (!(is_rx == false && fs)) {             // stored protocol: TX with F/S high (PPM input) or RX with F/S high
    proto_t pm;
    if (cfg_map(cfg_boot.proto, cfg_boot.sub, is_rx, &pm)) proto = pm; else { bay_unsupported = true; proto = PROTO_NRF24; }
    nrf_rx_role = (proto == PROTO_NRF24) && is_rx;
  }                                                 // else: TX stored, F/S low: the encoder chooses (below)
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  { sl_gpio_t proto3_pin = { .port = SL_GPIO_PORT_C, .pin = 3 };
    sl_gpio_set_pin_mode(&proto3_pin, SL_GPIO_MODE_INPUT_PULL, 1);
    app_log("JUMPERS PC00=%d PC01=%d PC02=%d PC03=%d F/S=%d RANGE=%d BIND=%d -> proto %d (0 nRF24 1 D8 2 D16 3 RLINK 4 DSMCAP 5 DSMRX 6 DSMTX 7 TXTEST 8 JETICAP 9 UKONLY 10 UKRX)%s\r\n",
            pin_low(&proto0_pin), pin_low(&proto1_pin), pin_low(&proto2_pin), pin_low(&proto3_pin), pin_low(&fs_pin), pin_low(&range_pin), pin_low(&bind_pin), (int)proto,
            jumpers ? "" : " (protocol pins not used: stored setup)");
    sl_gpio_set_pin_mode(&proto3_pin, SL_GPIO_MODE_DISABLED, 0); }
  if (cfg_used) {
    const char *nm = cfg_proto_name(cfg_boot.role, cfg_boot.proto, cfg_boot.sub);
    app_log("SETUP stored (cfg): %s Prot %u Sub %u = %s%s; F/S %s -> %s\r\n", is_rx ? "RX" : "TX", cfg_boot.proto, cfg_boot.sub,
            nm ? nm : "not in this module", (!is_rx && !(cfg_boot.proto == 0u && cfg_boot.sub >= 2u)) ? (cfg_boot.option ? ", Opt dual" : ", Opt single") : "",
            fs ? "low" : "high",
            is_rx ? (fs ? "protocol from the jumpers" : "that protocol") : (fs ? "encoder serial control" : "MULTI frames or PPM, whichever comes first"));
    if (bay_unsupported) app_log("SETUP Prot %u Sub %u is not a %s mode of this module: RF off, both LEDs blink. Store another with cfg\r\n",
                                 cfg_boot.proto, cfg_boot.sub, is_rx ? "receiver" : "transmitter");
  } else app_log("SETUP none stored: the jumpers decide ('cfg' on this line stores one)\r\n");
#endif
  // Encoder serial control: a transmitter (F/S open) with the RANGE jumper fitted at power-up - or a stored TX setup
  // with F/S low. It overrides every other jumper. A stored TX setup with F/S open (no jumpers) listens for both:
  // MULTI frames -> encoder control, PPM -> the stored protocol. Any other setting: exactly as before, no serial.
  bool bay_auto = cfg_used && !is_rx && !fs;
  bay_listen = force_serial || (cfg_used ? (!is_rx) : (!fs && pin_low(&range_pin)));
  if (bay_listen) {
    if (!force_serial) bay_init(rh, bay_on_frame);   // after the probe it is running already
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log(bay_auto ? "BAY v1.4 stored TX setup, no F/S: MULTI frames on PA06 = encoder control, PPM on PB01 = the stored protocol; waiting for either\r\n"
          : cfg_used ? "BAY v1.1 stored TX setup, F/S low: encoder control; waiting for MULTI frames on PA06\r\n"
                     : "BAY v1.1 RANGE jumper at power-up: encoder control, other jumpers ignored; waiting for MULTI frames on PA06\r\n");
#endif
  }

  // LED self-test: 3 fast double-blinks at every boot (proves LED hardware)
  for (uint8_t k = 0; k < 6; k++) {
    sl_gpio_toggle_pin(&led_a);
    sl_gpio_toggle_pin(&led_b);
    sl_rail_handle_t rh2 = rh;
    uint32_t t0 = (uint32_t)sl_rail_get_time(rh2);
    while (((uint32_t)sl_rail_get_time(rh2) - t0) < 60000u) { }
  }

  if (bay_listen) {
    bay_frame_t f;
    EUSART0->CMD = EUSART_CMD_RXEN;               // the command line listens while we wait (app_init.c had stopped it): "cfg" works here
    if (!bay_detect(rh, &f, bay_auto)) {          // PPM came first (stored TX, no F/S): the stored protocol on PPM, as before
      NVIC_DisableIRQ(EUSART1_RX_IRQn);
      EUSART_IntDisable(EUSART1, EUSART_IEN_RXFL);
      EUSART_Enable(EUSART1, eusartDisable);      // PA06 back to the log UART alone: its command line keeps listening
      bay_listen = false;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("BAY PPM on PB01 came first: the stored protocol on PPM input, encoder serial off until the next power-up\r\n");
#endif
    }
  }
  if (bay_listen) {
    bay_frame_t f;
    (void)bay_get(&f);                            // the frame bay_detect returned
    EUSART0->CMD = EUSART_CMD_RXDIS;              // the encoder owns PA06 now: the log UART (and its command line) stops listening
    { uint8_t junk[16]; size_t n;                 // ... and what the frames left in its buffer is dropped, not echoed
      do { n = 0; if (sl_iostream_read(sl_iostream_inst_handle, junk, sizeof junk, &n) != SL_STATUS_OK) break; } while (n != 0u); }
    {
      proto_t p;
      bay_boot = f;
      rc_model = f.rxnum;                          // per-model binding: the encoder's receiver number (Rx No)
      bay_bind_now  = f.bind;
      bay_range_now = f.range;
      nrf_rx_role = false;
      bay_unsupported = false;                    // the encoder chooses now, whatever was stored
      if (bay_map(&f, &p)) proto = p; else { bay_unsupported = true; proto = PROTO_NRF24; }
      if (f.rxsetup) {                            // RxSetup: the setup packets go out from the RLINK TX role (their PHY),
        bay_rxs_role = (proto != PROTO_RLINK);    // whatever the Prot / Sub (the receiver mode they carry)
        proto = PROTO_RLINK; bay_unsupported = false;
      }
      NVIC_DisableIRQ(EUSART1_RX_IRQn);
      bay_mode = true;
      bay_on_frame(&f);                           // channels in place before the role starts
      NVIC_EnableIRQ(EUSART1_RX_IRQn);
      TIMER_IntDisable(TIMER0, TIMER_IEN_CC0);    // PB01 is not a PPM input in encoder mode
      NVIC_DisableIRQ(TIMER0_IRQn);
      if (f.edgetx) bay_tx_polarity(f.tel_inv);   // EdgeTX: telemetry inverted (S.Port), unless the radio says otherwise
      if (f.telem) bay_tx_begin();                // ... and carries the telemetry when the encoder listens for it
      bay_log(f.edgetx ? "BAY v1.5 EdgeTX radio (inverted MULTI) on PA06, Multiprotocol telemetry back on PB01:"
                       : "BAY v1.3 encoder MULTI on PA06, telemetry back on PB01:", &f);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      if (f.edgetx && f.scan)
        app_log("BAY EdgeTX is reading the protocol list: RF off until it picks a protocol (the module restarts with it)\r\n");
      else if (f.edgetx && bay_unsupported)
        app_log("BAY EdgeTX protocol %u sub %u is not in this module: RF off, both LEDs blink. Use FrSkyD D8, DSM DSMX_1F%s, FrSkyX CH_16 / CH_8 / EU_16, FrSkyX2 CH_16 / EU_16, MiTo (120) nRF24 / RLINK / UK459\r\n",
                f.wproto, f.wsub, DTX_EU_LOCK ? "" : " / DSMX_2F");
      else if (bay_unsupported)
        app_log("BAY Prot %u Sub %u is not in this module%s: RF off, both LEDs blink. Use Prot 3 D8, 15 D16, 6 Sub 2 DSMX 22 ms%s, 0 Sub 0 nRF24, 0 Sub 1 RLINK, 0 Sub 2 UK only, 0 Sub 6 Jeti capture, 0 Sub 7 TX test\r\n",
                f.proto, f.sub, (f.proto == 6u && f.sub == 3u) ? " (DSMX 11 ms is locked out in this EU build)"
                              : (f.proto >= 64u) ? " (a 27-byte frame: protocol 64 or more, e.g. 64 FrSkyX2 = ACCST 2.x; this module does D16 ACCST 1.x only)" : "",
                DTX_EU_LOCK ? "" : ", 6 Sub 3 DSMX 11 ms");
#endif
    }
  }
  sl_gpio_clear_pin(&led_a);
  sl_gpio_clear_pin(&led_b);
  if (alt_mode) { sl_gpio_set_pin(&led_a); }   // opposite phases -> alternating flash

  generatePipeFromUniqueIDAndHash(pipeOut);     // after the encoder check: the pipe includes the model number
  memcpy(bindPkt.pipe, pipeOut, 4);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("MODEL %u%s\r\n", rc_model, rc_model ? ": transmitter IDs for this model (per-model binding)" : " (no encoder, or Rx No 0): IDs as before");
#endif

#if BIND_PIN_ENABLED
  buttonPressed = bind_held();
#else
  buttonPressed = false;
#endif
  if (bay_mode && bay_bind_now) buttonPressed = true;   // frames' BIND: the nRF24 bind goes on while it is on

  generateAndShuffleChannels(generateSeedFromPipe(BIND_PIPE));   // bind sequence
  set_air_address(BIND_PIPE);
  sl_rail_set_tx_power_dbm(rh, TX_POWER_MIN_DDBM);

  // DUAL build: the SX1278 (uk459_tx.c) is looked for in every role. With the encoder its BIND and RANGE come
  // from the frames (bind_in / range_in: the RANGE pin is the encoder jumper then, not the range check).
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("BUILD dual-serial: 2.4 GHz + SX1278 UK 459 MHz, encoder control, servo outputs PA07 PB04 PD04 PD05 PA00 PA08 PA09 PB05\r\n");
#endif
  uk_have_sx = uk_probe(rh);
  if (uk_have_sx) {
    bool rx_role = is_rx && (nrf_rx_role || proto == PROTO_D8 || proto == PROTO_D16 || proto == PROTO_DSMRX || proto == PROTO_RLINK);
    if (proto == PROTO_UKRX) { /* the receiver role takes the SX1278 itself (uk_rx_start in the main loop) */ }
    else if (rx_role && !bay_unsupported) {                  // DUAL receiver: the 459 MHz receiver runs beside the 2.4 GHz one
      uk_dual_rx = true;
      uk_rx_start(rh, bind_held(), false);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("UK dual receiver: 459 MHz receiver beside the 2.4 GHz one; servos follow 2.4 GHz, 459 MHz within 100 ms of losing it\r\n");
#endif
    }
    else if (proto == PROTO_DSMCAP || proto == PROTO_JETICAP || proto == PROTO_TXTEST || is_rx || bay_unsupported) {
      uk_sleep();                                             // tools (captures, TX test), RF off
    } else if (proto != PROTO_UKONLY && (bay_mode ? bay_boot.option == 0 : (cfg_used && cfg_boot.option == 0))) {   // Opt 0 = single: 2.4 GHz only
      uk_sleep();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log(bay_mode ? "UK single (Opt 0): 459 MHz link off. Opt 1 = dual, 459 MHz beside this mode\r\n"
                       : "UK single (stored setup, Opt single): 459 MHz link off. 'cfg tx <proto> dual' = 459 MHz beside this mode\r\n");
#endif
    } else {                                                  // dual: with the encoder Opt 1 (or any other value), without it always
      uk_start(contrl_pos, bind_in(), range_in());
      uk_link_on = true;
    }
  }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  else app_log("UK DUAL build but no SX1278 answered: 2.4 GHz only\r\n");
#endif

  if (proto == PROTO_NRF24 && !nrf_rx_role && !bay_unsupported) start_frame_timer();   // nRF24 TX; the RX roles, FrSky and RLINK run their own timing
  proto_started = true;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log_info("BOOT: bind window, ch=%u, pipe %02X%02X%02X%02X, btn=%d\n",
               hopChannels[0], pipeOut[0], pipeOut[1], pipeOut[2], pipeOut[3],
               (int)buttonPressed);
#endif
}

// ------------------------------------------------------------- main action --
void app_process_action(void)
{
  sl_rail_handle_t rh = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (fs_cmd_log) { fs_cmd_log = false; app_log("BAY failsafe frame from the encoder: the receiver stores these positions\r\n"); }
#endif
  if (cal_pending) {   // deferred from the ISR (SL_RAIL_EVENT_CAL_NEEDED): a predictable point in the
    cal_pending = false;   // loop beats doing it inline in interrupt context, which could land on a scheduled TX
    calibration_status = sl_rail_calibrate(rh, NULL, SL_RAIL_CAL_ALL_PENDING);
    cal_count++;
    if (calibration_status != SL_RAIL_STATUS_NO_ERROR) { cal_err = true; }
  }

#if PPM_DIAG && defined(SL_CATALOG_APP_LOG_PRESENT)
  {
    static uint32_t ppm_uart_us = 0;
    uint32_t nowd = (uint32_t)sl_rail_get_time(rh);
    if (!bay_mode && !is_rx && (nowd - ppm_uart_us) >= 1000000u) {   // transmitter with PPM only; encoder serial: bay_service prints a BAY line
      ppm_uart_us = nowd;
      app_log("PPM frames=%lu period=%lu period_jit=%lu worst_jump=%u\r\n",
              (unsigned long)ppm_frames, (unsigned long)ppm_frame_period_us,
              (unsigned long)ppm_worst_period, (unsigned)ppm_worst_jump);
      ppm_worst_jump = 0; ppm_worst_period = 0;   // per-second window
    }
  }
#endif

  if (!proto_started) { proto_start(); return; }
  if (bay_listen) bay_service(rh);   // encoder MULTI frames: BIND / RANGE, restarts, status line
  if (bay_unsupported) {             // the encoder asked for a Prot / Sub this module does not have: RF off
    static uint32_t tb = 0;
    static bool     ph = false;
    uint32_t t = (uint32_t)sl_rail_get_time(rh);
    if ((uint32_t)(t - tb) >= 250000u) {          // both LEDs, alternating, 2 Hz
      tb = t; ph = !ph;
      if (ph) { sl_gpio_set_pin(&led_a);   sl_gpio_clear_pin(&led_b); }
      else    { sl_gpio_clear_pin(&led_a); sl_gpio_set_pin(&led_b); }
    }
    return;
  }
  { static uint32_t so_report_at = 0;   // receiver roles: servo output health every 2 s (late interrupts, channel jumps)
    uint32_t now = (uint32_t)sl_rail_get_time(rh);
    if ((now - so_report_at) >= 2000000u) { so_report_at = now; servo_out_report(); }
    if (is_rx) rx_outputs_service(now); }
  if (uk_dual_rx) {   // DUAL receiver: 459 MHz receiver beside the 2.4 GHz role; servos take its data when 2.4 GHz is away
    uk_rx_service(false);
    uint32_t now = (uint32_t)sl_rail_get_time(rh);
    bool p = nrf_rx_role ? nrf_rx_fresh(now, DUAL_RX_SWITCH_US)
           : proto == PROTO_D8    ? frsky_rx_fresh(now, DUAL_RX_SWITCH_US)
           : proto == PROTO_D16   ? frsky16_rx_fresh(now, DUAL_RX_SWITCH_US)
           : proto == PROTO_DSMRX ? dsm_rx_fresh(now, DUAL_RX_SWITCH_US)
           : proto == PROTO_RLINK ? rlink_rx_fresh(now, DUAL_RX_SWITCH_US) : true;
    bool u = uk_rx_fresh(now, DUAL_RX_UK_HOLD_US);
    bool want = !p && u;
    if (want != servo_out_overridden()) {
      servo_out_override(want ? uk_rx_channels() : NULL);
      if (want) dual_rx_switches++;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log(want ? "DUAL servos from 459 MHz (2.4 GHz silent %lu ms)\r\n" : "DUAL servos back on 2.4 GHz\r\n", (unsigned long)(DUAL_RX_SWITCH_US / 1000u));
#endif
    }
  }
  if (proto == PROTO_UKRX) {   // UK 459 MHz receiver only (PC01 + PC02 + F/S): uk459_rx.c, the 2.4 GHz radio idle
    static bool ur_started = false; static uint32_t ur_tb = 0; static bool ur_ph = false;
    if (!ur_started) { ur_started = true;
      if (uk_have_sx) uk_rx_start(rh, bind_held(), true);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      else app_log("UK receiver asked for (PC01 + PC02 + F/S), but no SX1278 answered: nothing to receive, both LEDs blink\r\n");
#endif
    }
    if (!uk_have_sx) {
      uint32_t t = (uint32_t)sl_rail_get_time(rh);
      if ((uint32_t)(t - ur_tb) >= 250000u) { ur_tb = t; ur_ph = !ur_ph;
        if (ur_ph) { sl_gpio_set_pin(&led_a); sl_gpio_clear_pin(&led_b); } else { sl_gpio_clear_pin(&led_a); sl_gpio_set_pin(&led_b); } }
      return;
    }
    uk_rx_service(bind_held());
    return;
  }
  if (uk_link_on) {   // UK 459 MHz link beside every 2.4 GHz transmitter role (uk459_tx.c), its own 40 ms timing
    uint32_t last = ppm_last_sync;                            // read BEFORE the clock, as in the roles below
    uint32_t now  = (uint32_t)sl_rail_get_time(rh);           // (encoder frames update it as PPM does)
    // RLINK: UK data only while RLINK's link alarm is on (1 = 2 beeps, weak link; 2 = continuous, link lost);
    // every other transmitter role: all the time
    bool want = (proto != PROTO_RLINK) || (rlink_diag[R_ALARM] >= 1);
    uk_service(now, (int32_t)(now - last) <= 100000, bind_in(), range_in(), want);
  }
  if (proto == PROTO_UKONLY) {  // UK only (encoder Prot 0 Sub 2): the 459 MHz link alone, the 2.4 GHz radio stays idle.
    static bool    uo_started = false;                        // For aligning the UK transmitter with a receiver without
    static int32_t uo_last_tx = 0;                            // anything on 2.4 GHz. LED_A = encoder input present,
    static uint32_t uo_tb = 0; static bool uo_ph = false;     // LED_B toggles per UK packet sent.
    uint32_t t = (uint32_t)sl_rail_get_time(rh);
    if (!uo_started) { uo_started = true; sl_rail_idle(rh, SL_RAIL_IDLE, true);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      if (uk_link_on) app_log("UK only: 459 MHz link running, 2.4 GHz radio idle (encoder Prot 0 Sub 2)\r\n");
      else            app_log("UK only asked for, but no SX1278 answered: nothing to send, both LEDs blink\r\n");
#endif
    }
    if (!uk_link_on) {                                        // no RA-01 on this board: same signal as "not supported"
      if ((uint32_t)(t - uo_tb) >= 250000u) { uo_tb = t; uo_ph = !uo_ph;
        if (uo_ph) { sl_gpio_set_pin(&led_a); sl_gpio_clear_pin(&led_b); } else { sl_gpio_clear_pin(&led_a); sl_gpio_set_pin(&led_b); } }
      return;
    }
    { uint32_t last = ppm_last_sync;
      if ((int32_t)(t - last) <= 100000) sl_gpio_set_pin(&led_a); else sl_gpio_clear_pin(&led_a); }
    if (uk_diag[UK_TX] != uo_last_tx) { uo_last_tx = uk_diag[UK_TX]; sl_gpio_toggle_pin(&led_b); }
    return;
  }
  if (proto == PROTO_RLINK) { // resilient link (PC02): F/S=RX, RANGE=low-power range check
    static bool rl_started = false;
    if (!rl_started) { rl_started = true; rlink_start(rh, contrl_pos, is_rx && !bay_mode, range_in(), bind_in()); }
    if (rx_held) { rx_held = false; rlink_rx_process(); }
    { uint32_t last = ppm_last_sync;                          // read BEFORE the clock: the PPM ISR may update it
      uint32_t now  = (uint32_t)sl_rail_get_time(rh);        // in between, and a newer 'last' must not look old
      rlink_input((int32_t)(now - last) <= 100000);          // encoder PPM (or MULTI frames) present (100 ms)
      rlink_range(range_in());                               // RANGE followed at any time (TX)
      rlink_bind(bay_mode && bay_bind_now);                  // frames' BIND: the bind goes on while it is on
      rxs_tx_request(bay_mode && bay_rxsetup_now, bay_boot.proto, bay_boot.sub, bay_rxs_layout);   // RxSetup: setup packets
    }
    rlink_action();
    return;
  }
  if (proto == PROTO_DSMTX) {   // DSMX transmitter (PC00 + PC01): encoder PPM -> DSMX
    static bool dt_started = false;
    if (!dt_started) {
      dt_started = true;
      bool b = bind_in();
      // Serial frames: Sub 3 = 11 ms, Sub 2 = 22 ms -- taken at the bind; the EU build (DTX_EU_LOCK 1) refuses Sub 3.
      // Jumpers: RANGE at power-up selects encoder control, so a jumper bind is always 22 ms.
      bool r = (bay_mode && b) ? (bay_boot.sub == 3u) : range_in();
      dsm_tx_start(rh, contrl_pos, b, r);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      if (bay_mode && !b && t_mode22 != (bay_boot.sub == 2u))
        app_log("BAY DSMX: Sub %u asks for %s, but this module runs %s (%s). Bind again to change to %s\r\n",
                bay_boot.sub, (bay_boot.sub == 3u) ? "11 ms" : "22 ms", t_mode22 ? "22 ms" : "11 ms", t_mode_from,
                (bay_boot.sub == 3u) ? "11 ms" : "22 ms");
#endif
    }
    if (rx_held) { rx_held = false; dsm_tx_rx_process(); }
    { uint32_t last = ppm_last_sync;
      uint32_t now  = (uint32_t)sl_rail_get_time(rh);
      dsm_tx_input((int32_t)(now - last) <= 100000); }
    dsm_tx_range(range_in());   // RANGE followed at any time
    dsm_tx_action();
    return;
  }
  if (proto == PROTO_DSMRX) {   // DSMX receiver (PC00 + PC01 + F/S): PPM, servos out
    static bool dr_started = false;
    if (!dr_started) { dr_started = true; dsm_rx_start(rh, bind_held()); }
    if (rx_held) { rx_held = false; dsm_rx_process(); }
    dsm_rx_service(bind_held());
    return;
  }
  if (proto == PROTO_JETICAP) { // Jeti Duplex sniffer (PC00 + PC02, or encoder Prot 0 Sub 6): 802.15.4 frames -> UART hex
    static bool jc_started = false;
    if (!jc_started) { jc_started = true; jeti_cap_start(rh, bind_held(), pin_low(&fs_pin)); }   // BIND = lock on the busiest channel after 2 sweeps; F/S = lock on ch 11
    if (rx_held) { rx_held = false; jeti_cap_rx_process(); }
    jeti_cap_service();
    return;
  }
  if (proto == PROTO_DSMCAP) {  // DSM raw capture (PC00 + PC01 + PC02): demod bit stream -> UART hex
    static bool dc_started = false;
    if (!dc_started) { dc_started = true; dsm_cap_start(rh, pin_low(&range_pin), bind_held()); }   // RANGE = data test/census; BIND = bind sniffer on 2414 MHz
    if (rx_held) { rx_held = false; dsm_cap_rx_process(); }
    dsm_cap_service(bind_held());
    return;
  }
  if (proto == PROTO_TXTEST) {  // TX test (PC00 + PC01 + PC02 + F/S): continuous carrier; BIND at boot = hold at max
    static bool tt_started = false;
    if (!tt_started) { tt_started = true; tx_test_start(rh, bind_in()); }
    tx_test_action();
    return;
  }
  if (nrf_rx_role) {  // nRF24 receiver (no protocol jumper + F/S low): PPM, servos, SBUS out
    static bool nr_started = false;
    if (!nr_started) { nr_started = true; nrf_rx_start(rh, bind_held()); }
    if (rx_held) { rx_held = false; nrf_rx_process(); }
    nrf_rx_service(bind_held());
    return;
  }
  if (alt_mode) {   // FrSky D8 (PC00) or D16 EU (PC01): TX = bind then 9 ms data frames; + F/S low = receiver with PPM out
    static bool     started = false;
    static uint32_t t_b = 0;
    static int32_t  last_a = 0;
    bool d16 = (proto == PROTO_D16);
    if (!started) { started = true;
      bool rxr = is_rx && !bay_mode;
      frsky16_variant = (uint8_t)(bay_mode ? bay_boot.sub : (cfg_used ? cfg_boot.sub : 0u));   // D16: Sub 0 v1 EU, 1 v2 EU, 2 v1 FCC, 3 v2 FCC, 5 v1 FCC 8 ch
      if (d16) frsky16_start(rh, contrl_pos, bind_in(), rxr); else frsky_start(rh, contrl_pos, bind_in(), rxr); }
    if (rx_held) { rx_held = false; if (d16) frsky16_rx_process(); else frsky_rx_process(); }
    uint32_t t = (uint32_t)sl_rail_get_time(rh);
    ppm_check_loss(t);
    { bool bh = bay_mode && bay_bind_now;                     // frames' BIND: bind and flying turns while it is on
      if (d16) frsky16_bind(bh); else frsky_bind(bh);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      static bool bh_noted = false;
      if (bh && !bh_noted) { bh_noted = true;
        app_log("FrSky %s: while BIND is on, 1.5 s of bind and 1.5 s of flying take turns (LED_B fast blink = bind)\r\n", d16 ? "D16" : "D8"); }
#endif
    }
    if (d16) frsky16_service(bind_in(), range_in()); else frsky_service(bind_in(), range_in());
    volatile int32_t *dg = d16 ? frsky16_diag : frsky_diag;   /* same index layout 0..12 */
    int   phase   = d16 ? (int)frsky16_phase() : (int)frsky_phase();
    bool  link    = d16 ? frsky16_link_up() : frsky_link_up();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    { static uint32_t t_fs = 0;                               // FrSky status every 2 s
      if ((uint32_t)(t - t_fs) >= 2000000u) { t_fs = t;
        if (!dg[0])                                           // transmitter
        { uint32_t ln = 0, lw = 0, lr = 0, lf = 0; if (!d16) frsky_tx_stats(&ln, &lw, &lr, &lf); else frsky16_tx_stats(&ln, &lr, &lf);   /* D16: chg = LBT busy */
          app_log("FS %s tx %s bind%ld data%ld sent%ld err%ld tele%ld rxrssi%ld crcbad%lu cfg%ld chg%lu step%luus resync%lu refused%lu\r\n", d16 ? "D16" : "D8",
                  phase == 0 ? "bind" : "data", (long)dg[2], (long)dg[3], (long)dg[4], (long)dg[5], (long)dg[8], (long)dg[13],
                  d16 ? 0ul : (unsigned long)frsky_crc_bad(), (long)dg[6], (unsigned long)ln, (unsigned long)lw, (unsigned long)lr, (unsigned long)lf); }
        else {                                                // receiver
          uint16_t ln = 0, lr = 0, ls = 0; uint8_t lh = 0;
          if (!d16) frsky_crc_last(&ln, &lh, &lr, &ls);
          app_log("FS %s rx %s pk%ld bind%ld data%ld odd%ld miss%ld ch1 %ld ch2 %ld crcbad%lu hwcrc%lu cfg%ld | last n%u hw%u crc %04X cc2500 %04X\r\n", d16 ? "D16" : "D8",
                  dg[1] == 11 ? "link" : dg[1] == 10 ? "search" : "bind", (long)dg[7], (long)dg[8], (long)dg[9], (long)dg[10], (long)dg[15],
                  (long)dg[11], (long)dg[12], d16 ? 0ul : (unsigned long)frsky_crc_bad(), d16 ? 0ul : (unsigned long)frsky_hwcrc_agree(), (long)dg[6],
                  (unsigned)ln, (unsigned)lh, (unsigned)lr, (unsigned)ls);
          if (!d16 && ln) { const uint8_t *q = frsky_last_pkt();       // the packet itself
            app_log("FS D8 pkt %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | %02X %02X\r\n",
                    q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11], q[12], q[13], q[14], q[15], q[16], q[17], q[18], q[19]); }
          if (d16) { uint16_t pn = 0; const uint8_t *q = frsky16_last_pkt(&pn);   // the last packet heard, raw
            if (pn) {
              char hx[40 * 3 + 1]; uint16_t m = pn > 40u ? 40u : pn;
              for (uint16_t i = 0; i < m; i++) { static const char d[] = "0123456789ABCDEF"; hx[3*i] = d[q[i] >> 4]; hx[3*i+1] = d[q[i] & 15]; hx[3*i+2] = ' '; }
              hx[3 * m] = 0;
              app_log("FS D16 pkt n%u %s\r\n", (unsigned)pn, hx); } } }
      } }
#endif
    // LED_A: TX role steady while receiver telemetry is heard, else toggles per ~10 packets sent;
    //        RX role toggles per packet received.  LED_B: bind fast flash; data steady;
    //        RX role steady once a data packet decoded.
    if (!dg[0] && link)      { sl_gpio_set_pin(&led_a); }                       // TX: steady when telemetry seen
    else if (dg[0] && !link) { sl_gpio_clear_pin(&led_a); }                     // RX: off when no link
    else { int32_t a = dg[0] ? dg[7] : dg[4] / 10; if (a != last_a) { last_a = a; sl_gpio_toggle_pin(&led_a); } }
    if (dg[0])                             { if (link) sl_gpio_set_pin(&led_b); else sl_gpio_clear_pin(&led_b); }   // RX role: LED_B = live link
    else if (phase == 0)                   { if ((t - t_b) > 100000u) { t_b = t; sl_gpio_toggle_pin(&led_b); } }  // TX bind: fast blink
    else                                   { sl_gpio_set_pin(&led_b); }                                            // TX data: steady
    return;
  }

  if (cal_err) { cal_err = false;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log_error("cal err 0x%08" PRIX32 "\n", (uint32_t)calibration_status);
#endif
  }
  if (tx_err) { tx_err = false;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log_error("tx err 0x%" PRIX64 "\n", radio_events);
#endif
  }
  if (packet_sent) { packet_sent = false; led_a_toggle(); }

  if (!frame_due) { return; }
  frame_due = false;

  uint32_t now = (uint32_t)sl_rail_get_time(rh);
  ppm_check_loss(now);

#if BIND_PIN_ENABLED
  bool bindBtn     = bind_held();
#else
  bool bindBtn     = false;
#endif
  if (bay_mode && bay_bind_now) bindBtn = true;   // BIND on in the frames
  bool failsafePin = (pin_low(&fs_pin) && !bay_mode) || fs_cmd_active(now);   // the F/S button, or the encoder's failsafe frame
  bool rangePin    = range_in();

  // ------ startup-bind -> flight transition (verbatim) ------
  if (startupBind) {
    if (startupFrames >= STARTUP_BIND_FRAMES && !buttonPressed) {
      startupBind = false;
      generateAndShuffleChannels(generateSeedFromPipe(pipeOut));
      hopIndex   = 0;
      frameCount = 0;
      sl_rail_set_tx_power_dbm(rh, TX_POWER_MAX_DDBM);
      set_air_address(pipeOut);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log_info("-> FLIGHT\n");
#endif
    }
  }
  if (buttonPressed && !bindBtn) {        // held through boot, now released
    NVIC_SystemReset();                   // verbatim: resetFunc()
  }

  bool inBind = startupBind || buttonPressed;

  if (inBind) {
    bindPkt.failsafeFlag = failsafePin ? 1u : 0u;
    memcpy(frame_buf, &bindPkt, PAYLOAD_MAX);
    append_crc(frame_buf, BIND_PIPE);
    prepare_packet(rh, frame_buf, FRAME_LEN);
    (void)sl_rail_start_tx(rh, hopChannels[0], SL_RAIL_TX_OPTIONS_DEFAULT, NULL);
    return;
  }

  // ------------------------- flight (verbatim cadence) -------------------
  uint8_t channel = hopChannels[hopIndex];
  sl_rail_set_tx_power_dbm(rh, rangePin ? TX_POWER_MIN_DDBM : TX_POWER_MAX_DDBM);

  if (frameCount < 2u) {
    packPrimary(scaled10, primPkt.data);
    primPkt.data[6] = (uint8_t)((primPkt.data[6] & 0x03u) | ((frameSeq & 0x3Fu) << 2));
    frameSeq = (uint8_t)((frameSeq + 1u) & 0x3Fu);
    primPkt.failsafeFlag = failsafePin ? 1u : 0u;
    memcpy(frame_buf, &primPkt, PAYLOAD_MAX);
  } else {
    auxPkt.ch6 = scaled8[0];
    auxPkt.ch7 = scaled8[1];
    auxPkt.ch8 = scaled8[2];
    memcpy(frame_buf, &auxPkt, PAYLOAD_MAX);
  }
  append_crc(frame_buf, pipeOut);
  prepare_packet(rh, frame_buf, FRAME_LEN);
  (void)sl_rail_start_tx(rh, channel, SL_RAIL_TX_OPTIONS_DEFAULT, NULL);

  hopIndex = (uint8_t)((hopIndex + 1u) % num_channels);
  if (++frameCount >= 3u) { frameCount = 0; }
}

// ----------------------------------------------------------- RAIL callback --
SL_CODE_RAM void sl_rail_util_on_event(sl_rail_handle_t rh, sl_rail_events_t events)
{
  radio_events = events;
  if (events & SL_RAIL_EVENTS_TX_COMPLETION) {
    if (events & SL_RAIL_EVENT_TX_PACKET_SENT) { packet_sent = true; }
    else { tx_err = true; }
  }
  frsky_on_events(events);              // FrSky counters (ISR-safe, counts only)
  frsky16_on_events(events);
  if (proto == PROTO_RLINK) rlink_on_events(events);
  if (proto == PROTO_DSMCAP) dsm_cap_on_events(events);   // raw-capture FIFO overflow -> restart RX
  if (proto == PROTO_JETICAP) jeti_cap_on_events(events); // sniffer: overflow -> restart RX, counters
  if (proto == PROTO_DSMTX) dsm_tx_on_events(events);
  if (events & SL_RAIL_EVENT_RX_PACKET_RECEIVED) {
    (void)sl_rail_hold_rx_packet(rh);   // keep it in the FIFO for the main loop
    rx_held = true;
    if (proto == PROTO_D8) frsky_on_rx_held();   // D8 receiver: next hop / telemetry reply at once (31 kbit/s timing)
    if (proto == PROTO_D16) frsky16_on_rx_held(); // D16 receiver: the same
  }
  if (events & SL_RAIL_EVENT_CAL_NEEDED) {
    cal_pending = true;   // do the actual sl_rail_calibrate() from the main loop: it can take a few
  }                       // hundred us, and doing that inline here can land right on a scheduled TX
}

// -------------------------------------------------------------- button cb ---
SL_CODE_RAM void sl_button_on_change(const sl_button_t *handle)
{
  (void)handle;   // bind logic polls the button; nothing needed here
}
