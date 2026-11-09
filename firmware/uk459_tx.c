/* uk459_tx.c — see uk459_tx.h.  #included into app_process.c.   v1.6: the FIFO is written only once the SX has
 *   finished leaving TX (ModeReady after the standby command). Found on the bench, Oct 2026: this port runs the
 *   standby -> config -> FIFO -> TX sequence in about 60 us over the bit-banged SPI; the chip takes longer than that to
 *   leave TX and flushes the FIFO on the way, so every fourth packet went out empty (chip in TX, PLL locked, TxReady
 *   never set, FIFO empty, no "sent"). V9 on the ATmega never saw it: 500 kHz SPI and getRC() between the two.
 *   v1.5: a packet is never cut by the next slot (the slot waits until the packet is out: "sent" seen or UK_AIR_US
 *   passed), the retune after the beacon no longer depends on the main loop catching "sent" in time. The status
 *   line counts packets that never reported "sent" (nosent) and dumps the chip's registers for one of them every 2 s.
 *   v1.4 (branch dual-serial): with the encoder (Rx No)
 *   the UK ID includes the model too -- per-model binding, as every 2.4 GHz protocol; Rx No 0 / no encoder = the ID as
 *   before. BIND and RANGE come from the encoder's frames when it is in control (app_process.c passes them in).
 *   v1.3 (DUAL build, branch dual: the SX1278 has
 *   its pins to itself -- the receivers' servo outputs moved, servo_pins.h -- so receiver roles are no longer
 *   refused; the SX sleeps in them until the dual receiver is added.)
 *   v1.2: RLINK wake-up matched to the V7 receiver:
 *   V7 parks on hop position 0 after 1.2 s without packets, so after a longer silence the link restarts at hop 0,
 *   where the receiver is listening -- data within one packet, beacon sync within a dwell -- instead of making it
 *   wait up to one 4 s rotation. After a short silence (< 0.6 s) the schedule simply carries on; in between it
 *   waits until the receiver has parked, at most 0.65 s.)
 *   v1.1: with RLINK the data packets go out only while RLINK's link alarm is on -- 2 beeps or continuous -- and
 *   UK_GATE_HOLD_MS after; the hop schedule keeps running while they are held back. Other roles: all the time.
 *   v1.0: first version.
 *
 * Port of GFSK_Tx_UK459_HOP V9 (GFSK_Tx_UK459_HOP.ino, sx1278_fsk.ino, def_fsk.h, ppmin_fsk.ino) to the EFR32
 * module, as a second link that runs beside the 2.4 GHz transmitter roles.
 *
 * What is the same as V9 (the receiver sees no difference):
 *   - every SX register value (sx1278_fsk.ino SetFSKParams + init_sx1278), and the write order of doTX / RTXOFF
 *   - 10-byte packet every 40 ms; bytes 0-7 = PPM channels 1-8 encoded (clamp 900..2100 us - 900) x 17 + 40) / 80
 *   - data: byte 8 = ID0, byte 9 = ID1; every 5th packet of a dwell is the sync beacon: byte 8 = ID0 ^ next hop
 *     index, byte 9 = 0xFE, and after it the hop; channel bytes stay valid RC data in the beacon
 *   - bind packet on 458.5375 MHz: bytes 0-6 channels, byte 7 = lane, byte 8 = ID0, byte 9 = ID1
 *   - hop lanes (even 0,2..38 = 20 channels, odd 1,3..37 = 19), channel = 458.525 MHz + 25 kHz x n, order
 *     shuffled from the ID with V9's generator (the receiver rebuilds it from the ID it learned)
 *   - powers: flight 0xFF (+20 dBm, 100 mW), bind 0xF6, range 0xF6, idle 0x75 (RegPaConfig values)
 *   - no PPM: bind packets carry neutral values; data packets stop 500 ms after the last PPM frame
 * What differs:
 *   - the ID: two bytes 17..254 from the EFR32's EUI64 with V9's mix (byte 1 never 0xFE); rebind once
 *   - BIND: V9 resets the ATmega when BIND is released and the reset runs the auto-bind window again; here the
 *     2.4 GHz link must not be reset, so bind simply continues UK_AUTO_BIND_MS after release -- same packets
 *   - RANGE follows the input (no 30 s time-out), in step with the 2.4 GHz link
 *   - no failsafe-capture packet (D5 on V9): the failsafe is set on the 2.4 GHz side
 *   - no buzzer; the UART log tells the state
 *   - never blocks: V9 waits for "packet sent" after a beacon before retuning; here the main loop polls it
 *   - with RLINK, data packets only while RLINK's link alarm is on (see uk459_tx.h), woken up where V7 listens
 *
 * Modulation note: RegOpMode 0x21 / 0x23 / 0x25 (standby / TX / RX) as in sx1278_fsk.ino set the SX1276's
 * ModulationType field (bits 6-5) to 01, which the datasheet defines as OOK, not FSK; RegPaRamp 0x49 then means
 * OOK shaping with a cut-off at 2 x bit rate, and the 2.5 kHz deviation setting is not used. Both ends use the
 * same values, so the link works. UK_OPMODE_MOD keeps it as flown (0x20); 0x00 would make it real (G)FSK with
 * BT 0.5 and 2.5 kHz deviation, but only together with the same change in the receivers.
 *
 * SPI is bit-banged on the reserved pins (mode 0, MSB first, a few MHz): about 45 bytes per 40 ms slot, some
 * 150 us of main-loop time. The PPM input is timed by hardware and the 2.4 GHz packets are scheduled by RAIL, so
 * neither is disturbed by it.
 */
#include "uk459_tx.h"
#include "em_gpio.h"
#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif

/* ---------------------------------------------------------------------------------------------- settings -- */
#ifndef UK_AUTO_BIND_MS
#define UK_AUTO_BIND_MS     1500u    /* bind window at every power-up (V9 AUTO_BIND_MS); 0 = only with BIND held */
#endif
#ifndef UK_LANE_FROM_PC03
#define UK_LANE_FROM_PC03   1        /* 1: PC03 to GND at power-up = odd lane; 0: lane UK_LANE_FIXED              */
#endif
#define UK_LANE_FIXED       0u       /* 0 even (20 channels), 1 odd (19 channels)                                 */
#ifndef UK_UART_STATUS
#define UK_UART_STATUS      1        /* 1: a status line every 10 s                                                */
#endif
#ifndef UK_GATE_HOLD_MS
#define UK_GATE_HOLD_MS  10000u      /* RLINK: keep sending this long after its link alarm has cleared            */
#endif
#define UK_RX_STEP_MS      600u      /* silence shorter than this: V7 still hops roughly in step -> carry on      */
#define UK_RX_PARK_MS     1250u      /* V7 parks on hop position 0 after 1200 ms without packets (+50 ms margin)  */
#define UK_PA_FLIGHT     0xFFu       /* Pwr_UK458_100mW: +20 dBm on PA_BOOST with PaDac 0x87                      */
#define UK_PA_BIND       0xF6u       /* Pwr11dBm (V9 BIND_POWER)                                                  */
#define UK_PA_RANGE      0xF6u       /* Pwr11dBm (V9 RANGE_CHECK_POWER)                                           */
#define UK_PA_IDLE       0x75u       /* Pwr1dBm: what V9 leaves in RegPaConfig between packets                    */
#define UK_OPMODE_MOD    0x20u       /* modulation bits as flown by V9 (see the note above); 0x00 = FSK           */

#define UK_PERIOD_US    40000u       /* V9 TXTIME                                                                 */
#define UK_AIR_US       35000u       /* a 10-byte packet is on the air 32.9 ms (8+3+10+2 bytes at 5.6 kbps) + start */
#define UK_READY_US      2000u       /* v1.6: longest wait for ModeReady after the standby command (typ. < 0.3 ms)    */
#define UK_RC_HOLD_US  500000u       /* V9 getRC(): last PPM values held this long                                */
#define UK_DWELL            5u       /* packets per channel, the last one is the beacon                           */
#define UK_PKT_LEN         10u
#define UK_BIND_HZ   458537500u      /* off-grid bind frequency                                                   */
#define UK_BASE_HZ   458525000u      /* channel 0                                                                 */
#define UK_STEP_HZ       25000u
#define UK_HOPS_MAX        20u
#define UK_SX_VERSION    0x12u       /* RegVersion of SX1276/77/78/79                                             */

/* SX1276/78 registers (FSK/OOK page) */
#define UK_R_FIFO        0x00u
#define UK_R_OPMODE      0x01u
#define UK_R_BRMSB       0x02u
#define UK_R_BRLSB       0x03u
#define UK_R_FDEVMSB     0x04u
#define UK_R_FDEVLSB     0x05u
#define UK_R_FRMSB       0x06u
#define UK_R_FRMID       0x07u
#define UK_R_FRLSB       0x08u
#define UK_R_PACONFIG    0x09u
#define UK_R_PARAMP      0x0Au
#define UK_R_OCP         0x0Bu
#define UK_R_LNA         0x0Cu
#define UK_R_RXBW        0x12u
#define UK_R_AFCBW       0x13u
#define UK_R_PREMSB      0x25u
#define UK_R_PRELSB      0x26u
#define UK_R_SYNCCFG     0x27u
#define UK_R_SYNC1       0x28u
#define UK_R_PKTCFG1     0x30u
#define UK_R_PKTCFG2     0x31u
#define UK_R_PAYLEN      0x32u
#define UK_R_FIFOTHR     0x35u
#define UK_R_IRQ1        0x3Eu
#define UK_R_IRQ2        0x3Fu
#define UK_R_VERSION     0x42u
#define UK_R_PADAC       0x4Du
#define UK_IRQ2_SENT     0x08u
#define UK_OP_SLEEP      0x00u
#define UK_OP_STDBY      (UK_OPMODE_MOD | 0x01u)
#define UK_OP_TX         (UK_OPMODE_MOD | 0x03u)
#define UK_PKTCFG1_VAL   0x50u       /* fixed length | CRC on | DC-free whitening */

/* ------------------------------------------------------------------------------------------------- pins -- */
static const sl_gpio_t uk_sck  = { .port = SL_GPIO_PORT_C, .pin = 6 };
static const sl_gpio_t uk_mosi = { .port = SL_GPIO_PORT_C, .pin = 7 };
static const sl_gpio_t uk_miso = { .port = SL_GPIO_PORT_C, .pin = 8 };
static const sl_gpio_t uk_nss  = { .port = SL_GPIO_PORT_C, .pin = 9 };
static const sl_gpio_t uk_rst  = { .port = SL_GPIO_PORT_D, .pin = 3 };
static const sl_gpio_t uk_lane_pin = { .port = SL_GPIO_PORT_C, .pin = 3 };   /* J1 PROTOCOL_03 */

#define UK_HI(p)  GPIO_PinOutSet((GPIO_Port_TypeDef)(p).port, (p).pin)
#define UK_LO(p)  GPIO_PinOutClear((GPIO_Port_TypeDef)(p).port, (p).pin)
#define UK_IN(p)  GPIO_PinInGet((GPIO_Port_TypeDef)(p).port, (p).pin)

/* ------------------------------------------------------------------------------------------------ state -- */
volatile int32_t uk_diag[UK_DIAG_N];
static sl_rail_handle_t uk_rail;
static bool     uk_fitted, uk_on;
static const volatile uint16_t *uk_in;       /* encoder PPM, us, channels 1..8 (app_process.c contrl_pos)  */
static uint16_t uk_ch[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };
static bool     uk_have_rc;
static uint32_t uk_rc_at;
static uint8_t  uk_id[2];
static uint8_t  uk_lane, uk_n;
static uint8_t  uk_hops[UK_HOPS_MAX];
static uint8_t  uk_hop, uk_hop_next, uk_dwell;
static bool     uk_bind, uk_bind_btn, uk_range, uk_hop_wait;
static bool     uk_active;                      /* data packets go out (v1.1 gate: RLINK link alarm)          */
static bool     uk_wake_pending;                /* v1.2: wanted, waiting for the receiver to park on hop 0    */
static uint32_t uk_bind_until, uk_next, uk_hop_poll, uk_status_at, uk_active_until;
static uint32_t uk_quiet_since, uk_wake_at;
static uint8_t  uk_pkt[UK_PKT_LEN];
static uint32_t uk_now, uk_tx_at;               /* v1.5: time of this pass, time of the last TX command        */
static bool     uk_sent_seen;                   /* v1.5: "packet sent" seen for the last packet                 */
static uint32_t uk_n_nosent, uk_nosent_log_at;  /* v1.5: packets that never reported "sent" (status line, dump)  */
static uint32_t uk_n_notready;                  /* v1.6: standby not ModeReady within UK_READY_US (carried on)    */

/* -------------------------------------------------------------------------------------------------- SPI -- */
static inline void uk_ns(void) { __NOP(); __NOP(); __NOP(); __NOP(); }

static uint8_t uk_xfer(uint8_t out)             /* mode 0, MSB first */
{
  uint8_t in = 0;
  for (int b = 7; b >= 0; b--) {
    if ((out >> b) & 1u) UK_HI(uk_mosi); else UK_LO(uk_mosi);
    uk_ns();
    UK_HI(uk_sck);
    uk_ns();
    in = (uint8_t)((in << 1) | (UK_IN(uk_miso) ? 1u : 0u));
    UK_LO(uk_sck);
  }
  return in;
}

static void uk_write(uint8_t reg, uint8_t v)
{
  UK_LO(uk_nss); uk_ns();
  (void)uk_xfer((uint8_t)(reg | 0x80u)); (void)uk_xfer(v);
  uk_ns(); UK_HI(uk_nss); uk_ns();
}

static uint8_t uk_read(uint8_t reg)
{
  UK_LO(uk_nss); uk_ns();
  (void)uk_xfer((uint8_t)(reg & 0x7Fu));
  uint8_t v = uk_xfer(0);
  uk_ns(); UK_HI(uk_nss); uk_ns();
  return v;
}

static void uk_write_fifo(const uint8_t *p, uint8_t n)     /* V9 spiWritePacket */
{
  UK_LO(uk_nss); uk_ns();
  (void)uk_xfer((uint8_t)(UK_R_FIFO | 0x80u));
  for (uint8_t i = 0; i < n; i++) (void)uk_xfer(p[i]);
  uk_ns(); UK_HI(uk_nss); uk_ns();
}

static void uk_wait_us(uint32_t us)
{
  uint32_t t0 = (uint32_t)sl_rail_get_time(uk_rail);
  while (((uint32_t)sl_rail_get_time(uk_rail) - t0) < us) { }
}

static void uk_pins_release(void)               /* back to power-up state: nothing driven */
{
  sl_gpio_set_pin_mode(&uk_sck,  SL_GPIO_MODE_DISABLED, 0);
  sl_gpio_set_pin_mode(&uk_mosi, SL_GPIO_MODE_DISABLED, 0);
  sl_gpio_set_pin_mode(&uk_miso, SL_GPIO_MODE_DISABLED, 0);
  sl_gpio_set_pin_mode(&uk_nss,  SL_GPIO_MODE_DISABLED, 0);
  sl_gpio_set_pin_mode(&uk_rst,  SL_GPIO_MODE_DISABLED, 0);
}

/* ------------------------------------------------------------------------------------------------ radio -- */
static void uk_set_freq(uint32_t hz)            /* V9 SetFreq: Frf = f x 2^19 / 32 MHz, MSB, MID, LSB */
{
  uint32_t frf = (uint32_t)(((uint64_t)hz << 19) / 32000000u);
  uk_write(UK_R_FRMSB, (uint8_t)(frf >> 16));
  uk_write(UK_R_FRMID, (uint8_t)(frf >> 8));
  uk_write(UK_R_FRLSB, (uint8_t)frf);
}

static inline uint32_t uk_hop_hz(uint8_t idx) { return UK_BASE_HZ + UK_STEP_HZ * (uint32_t)uk_hops[idx]; }

static void uk_standby(void)                     /* V9 RTXOFF */
{
  uk_write(UK_R_OPMODE, UK_OP_STDBY);
  uk_write(UK_R_PACONFIG, UK_PA_IDLE);
}

static void uk_wait_ready(void)                  /* v1.6: ModeReady (RegIrqFlags1 bit 7) after a mode command */
{
  uint32_t t0 = (uint32_t)sl_rail_get_time(uk_rail);
  for (;;) {
    if (uk_read(UK_R_IRQ1) & 0x80u) return;
    if (((uint32_t)sl_rail_get_time(uk_rail) - t0) >= UK_READY_US) { uk_n_notready++; return; }
  }
}

static void uk_send(uint8_t pa)                  /* V9 doTX */
{
  uk_write(UK_R_IRQ1, 0xFF);
  uk_write(UK_R_IRQ2, 0xFF);
  uk_write(UK_R_OPMODE, UK_OP_STDBY);
  uk_wait_ready();                               /* v1.6: let the chip finish leaving TX before the FIFO is written */
  uk_write(UK_R_PACONFIG, pa);
  uk_write(UK_R_PKTCFG1, UK_PKTCFG1_VAL);
  uk_write(UK_R_PAYLEN, UK_PKT_LEN);
  uk_write(UK_R_FIFOTHR, (uint8_t)(0x80u | (UK_PKT_LEN - 1u)));
  uk_write_fifo(uk_pkt, UK_PKT_LEN);
  uk_write(UK_R_OPMODE, UK_OP_TX);
  uk_tx_at = uk_now;
  uk_sent_seen = false;
  uk_diag[UK_TX]++;
}

static void uk_config(void)                      /* V9 init_sx1278: GoSleep, SetFSKParams, then the rest */
{
  uk_write(UK_R_OPMODE, UK_OP_SLEEP);
  uk_write(UK_R_OPMODE, 0x01u);                  /* FSK standby first, as V9 */
  uk_write(UK_R_OPMODE, UK_OP_STDBY);
  uk_write(UK_R_PARAMP, 0x49);                   /* shaping bits 10 + 40 us ramp */
  uk_write(UK_R_BRMSB, 0x16); uk_write(UK_R_BRLSB, 0x52);        /* 5.6 kbps */
  uk_write(UK_R_FDEVMSB, 0x00); uk_write(UK_R_FDEVLSB, 0x29);    /* 2.5 kHz (used in FSK only) */
  uk_write(UK_R_RXBW, 0x04); uk_write(UK_R_AFCBW, 0x04);         /* 15.6 kHz (receive only) */
  uk_write(UK_R_PREMSB, 0x00); uk_write(UK_R_PRELSB, 0x08);      /* 8-byte preamble */
  uk_write(UK_R_SYNCCFG, 0x92);                                  /* sync on, 3 bytes: 2D D4 5A */
  uk_write(UK_R_SYNC1, 0x2D); uk_write(UK_R_SYNC1 + 1u, 0xD4); uk_write(UK_R_SYNC1 + 2u, 0x5A); uk_write(UK_R_SYNC1 + 3u, 0x96);
  uk_write(UK_R_PKTCFG1, UK_PKTCFG1_VAL);
  uk_write(UK_R_PKTCFG2, 0x40);                                  /* packet mode */
  uk_write(UK_R_PAYLEN, UK_PKT_LEN);
  uk_write(UK_R_FIFOTHR, (uint8_t)(0x80u | (UK_PKT_LEN - 1u)));
  uk_write(UK_R_LNA, 0x23);
  uk_write(UK_R_OCP, 0x2F);
  uk_write(UK_R_PADAC, 0x87);                                    /* +20 dBm option on PA_BOOST */
  uk_write(UK_R_PACONFIG, UK_PA_IDLE);
}

/* ------------------------------------------------------------------------------------- ID, hops, packet -- */
static void uk_gen_id(void)                      /* V9 generateHardwareID, fed with the EUI64 */
{
  uint64_t u = ((uint64_t)DEVINFO->EUI64H << 32) | DEVINFO->EUI64L;
  uint32_t seed = 0;
  for (int i = 0; i < 8; i++) {
    seed = seed * 31u + (uint8_t)(u >> (8 * i));
    seed = (seed << 13) | (seed >> 19);
  }
  seed += (uint32_t)rc_model * 0x9E3779B9u;      /* v1.4 per-model binding (app_process.c): 0 = the ID as before */
  seed ^= (seed >> 16); seed *= 0x85EBCA6Bu;
  seed ^= (seed >> 13); seed *= 0xC2B2AE35u;
  seed ^= (seed >> 16);
  uk_id[0] = (uint8_t)(17u + ((seed >> 8) & 0xFFu) % 238u);
  uk_id[1] = (uint8_t)(17u + (seed & 0xFFu) % 238u);
  if (uk_id[1] == 0xFEu) uk_id[1] = 0xFDu;       /* 0xFE marks the beacon */
}

static void uk_gen_hops(void)                    /* V9 generateHopSequence, verbatim arithmetic (16-bit seed) */
{
  uint16_t seed = (uint16_t)(uk_id[0] + ((uint16_t)uk_id[1] << 8));
  if (seed == 0u) seed = 0x2A5Bu;
  uk_n = (uk_lane == 1u) ? 19u : 20u;
  for (uint8_t i = 0; i < uk_n; i++) uk_hops[i] = (uk_lane == 1u) ? (uint8_t)(2u * i + 1u) : (uint8_t)(2u * i);
  for (uint8_t i = (uint8_t)(uk_n - 1u); i > 0u; i--) {
    seed = (uint16_t)(seed * 25173u + 13849u);
    uint8_t j = (uint8_t)(seed % (uint16_t)(i + 1u));
    uint8_t t = uk_hops[i]; uk_hops[i] = uk_hops[j]; uk_hops[j] = t;
  }
}

static void uk_fill_channels(bool has_rc)        /* ppmin_fsk.ino getRC() encoding; neutral without PPM */
{
  for (uint8_t c = 0; c < 8u; c++) {
    if (!has_rc) { uk_pkt[c] = (c == 0u) ? 0u : 128u; continue; }
    uint16_t us = uk_ch[c];
    if (us < 900u)  us = 900u;
    if (us > 2100u) us = 2100u;
    uk_pkt[c] = (uint8_t)(((uint32_t)(us - 900u) * 17u + 40u) / 80u);
  }
}

static void uk_retune(void)                      /* after the beacon: next channel */
{
  uk_hop_wait = false;
  uk_standby();
  uk_hop = uk_hop_next;
  uk_diag[UK_HOP] = uk_hop;
  uk_set_freq(uk_hop_hz(uk_hop));
}

/* --------------------------------------------------------------------------------------------- public -- */
bool uk_probe(sl_rail_handle_t rh)
{
  uk_rail = rh;
  /* Keep every line low (or open) while probing; NSS goes high only for the read itself. */
  sl_gpio_set_pin_mode(&uk_nss,  SL_GPIO_MODE_PUSH_PULL, 0);
  sl_gpio_set_pin_mode(&uk_sck,  SL_GPIO_MODE_PUSH_PULL, 0);
  sl_gpio_set_pin_mode(&uk_mosi, SL_GPIO_MODE_PUSH_PULL, 0);
  sl_gpio_set_pin_mode(&uk_miso, SL_GPIO_MODE_INPUT_PULL, 0);    /* pull-down: nothing fitted reads 0x00 */
  sl_gpio_set_pin_mode(&uk_rst,  SL_GPIO_MODE_PUSH_PULL, 0);     /* reset: low >= 100 us ... */
  uk_wait_us(1000u);
  sl_gpio_set_pin_mode(&uk_rst,  SL_GPIO_MODE_DISABLED, 0);      /* ... then released: the SX's pull-up takes it */
  uk_wait_us(10000u);                                            /* datasheet: 5 ms before the first access */
  UK_HI(uk_nss); uk_wait_us(2u);                                 /* NSS idle high, then the read's falling edge */
  uint8_t v = uk_read(UK_R_VERSION);
  uk_diag[UK_VERSION] = v;
  uk_fitted = (v == UK_SX_VERSION);
  if (!uk_fitted) uk_pins_release();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (uk_fitted) app_log("UK SX1276/78 found on the reserved pins (version 0x%02X)\r\n", (unsigned)v);
  else           app_log("UK no SX1276/78 on the reserved pins (version read 0x%02X): second link off\r\n", (unsigned)v);
#endif
  return uk_fitted;
}

void uk_sleep(void)
{
  if (!uk_fitted) return;
  uk_write(UK_R_OPMODE, UK_OP_SLEEP);
  uk_on = false; uk_diag[UK_PHASE] = 0;
}

void uk_start(const volatile uint16_t *ch_us, bool bind_held, bool range_low)
{
  if (!uk_fitted) return;
  uk_in = ch_us;
#if UK_LANE_FROM_PC03
  sl_gpio_set_pin_mode(&uk_lane_pin, SL_GPIO_MODE_INPUT_PULL, 1);
  uk_wait_us(100u);
  uk_lane = UK_IN(uk_lane_pin) ? 0u : 1u;         /* open = even, to GND = odd (V9's D0) */
  sl_gpio_set_pin_mode(&uk_lane_pin, SL_GPIO_MODE_DISABLED, 0);
#else
  uk_lane = UK_LANE_FIXED;
#endif
  uk_gen_id();
  uk_gen_hops();
  uk_config();

  uint32_t now = (uint32_t)sl_rail_get_time(uk_rail);
  uk_bind_btn = bind_held;
  uk_bind = bind_held || (UK_AUTO_BIND_MS > 0u);
  uk_bind_until = now + UK_AUTO_BIND_MS * 1000u;
  uk_range = range_low; uk_diag[UK_RANGE] = range_low ? 1 : 0;
  uk_active = true; uk_active_until = now + UK_GATE_HOLD_MS * 1000u; uk_diag[UK_ACTIVE] = 1;
  uk_hop = 0; uk_dwell = 0; uk_hop_wait = false;
  uk_set_freq(uk_bind ? UK_BIND_HZ : uk_hop_hz(0));
  uk_next = now + 1000u;
  uk_status_at = now + 10000000u;
  uk_on = true;
  uk_diag[UK_PHASE] = uk_bind ? 1 : 2;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("UK boot uk459_tx v1.6: ID %02X %02X (model %u), lane %s (%u channels), bind %s %u ms on 458.5375 MHz; PA flight 0x%02X bind 0x%02X range 0x%02X%s, modulation bits 0x%02X; hops:",
          uk_id[0], uk_id[1], (unsigned)rc_model, uk_lane ? "odd" : "even", (unsigned)uk_n,
          bind_held ? "while BIND is held, then" : "window", (unsigned)UK_AUTO_BIND_MS,
          (unsigned)UK_PA_FLIGHT, (unsigned)UK_PA_BIND, (unsigned)UK_PA_RANGE, range_low ? " (RANGE on)" : "", (unsigned)UK_OPMODE_MOD);
  for (uint8_t i = 0; i < uk_n; i++) app_log(" %u", (unsigned)uk_hops[i]);
  app_log(" (458.525 MHz + n x 25 kHz)\r\n");
#endif
}

void uk_service(uint32_t now, bool ppm_fresh, bool bind_low, bool range_low, bool want_data)
{
  if (!uk_on) return;
  uk_now = now;

  /* v1.1 gate (RLINK): on while wanted and UK_GATE_HOLD_MS after the last "want".
   * v1.2 wake-up, matched to the V7 receiver: it keeps hopping on its own clock (250 ms a hop) for 1.2 s after
   * the last packet, then parks on hop position 0 and listens there. */
  if (want_data) {
    uk_active_until = now + UK_GATE_HOLD_MS * 1000u;
    if (!uk_active && !uk_wake_pending) {
      uint32_t q = now - uk_quiet_since;         /* how long the link has been silent */
      if (uk_bind || q < UK_RX_STEP_MS * 1000u) {
        uk_active = true;                        /* receiver still roughly in step: carry on the schedule */
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("UK on: RLINK link alarm, silent %lu ms: hop schedule carries on\r\n", (unsigned long)(q / 1000u));
#endif
      } else {
        uk_wake_pending = true;                  /* receiver parked (or about to): restart at hop 0 */
        uk_wake_at = (q >= UK_RX_PARK_MS * 1000u) ? now : uk_quiet_since + UK_RX_PARK_MS * 1000u;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("UK on: RLINK link alarm, silent %lu ms: restart at hop 0 in %lu ms\r\n", (unsigned long)(q / 1000u),
                (unsigned long)((uk_wake_at - now) / 1000u));
#endif
      }
    }
  } else if (uk_active && (int32_t)(now - uk_active_until) >= 0) {
    uk_active = false; uk_quiet_since = now;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("UK standby: RLINK link good for %u s (hop schedule keeps running)\r\n", (unsigned)(UK_GATE_HOLD_MS / 1000u));
#endif
  }
  if (uk_wake_pending && (int32_t)(now - uk_wake_at) >= 0) {
    uk_wake_pending = false; uk_active = true;
    uk_hop_wait = false; uk_hop = 0; uk_dwell = 0; uk_diag[UK_HOP] = 0;   /* where V7 listens */
    uk_standby();
    uk_set_freq(uk_hop_hz(0));
    uk_next = now + 1000u;                       /* first packet now, then every 40 ms */
  }
  uk_diag[UK_ACTIVE] = uk_active ? 1 : 0;

  if (ppm_fresh && uk_in) {                      /* V9 getRC(): take a frame, hold it UK_RC_HOLD_US */
    for (uint8_t i = 0; i < 8u; i++) uk_ch[i] = uk_in[i];
    uk_rc_at = now; uk_have_rc = true;
  }
  bool has_rc = uk_have_rc && ((now - uk_rc_at) < UK_RC_HOLD_US);

  if (range_low != uk_range) {                   /* RANGE, followed at any time; used from the next packet */
    uk_range = range_low; uk_diag[UK_RANGE] = range_low ? 1 : 0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("UK RANGE %s: PA 0x%02X\r\n", range_low ? "on" : "off", (unsigned)(range_low ? UK_PA_RANGE : UK_PA_FLIGHT));
#endif
  }

  if (uk_hop_wait && (int32_t)(now - uk_hop_poll) >= 0) {        /* beacon on air: retune once it is out */
    uk_hop_poll = now + 1000u;
    if (uk_read(UK_R_IRQ2) & UK_IRQ2_SENT) { uk_sent_seen = true; uk_retune(); }
    else if ((now - uk_tx_at) >= UK_AIR_US) { uk_diag[UK_HOP_LATE]++; uk_retune(); }   /* out for sure: retune */
  }

#if UK_UART_STATUS && defined(SL_CATALOG_APP_LOG_PRESENT)
  if ((int32_t)(now - uk_status_at) >= 0) {
    uk_status_at = now + 10000000u;
    app_log("UK s%ld on%ld tx%ld bcn%ld quiet%ld late%ld noppm%ld hop%ld range%ld nosent%lu notready%lu\r\n",
            (long)uk_diag[UK_PHASE], (long)uk_diag[UK_ACTIVE], (long)uk_diag[UK_TX], (long)uk_diag[UK_BEACON], (long)uk_diag[UK_QUIET],
            (long)uk_diag[UK_HOP_LATE], (long)uk_diag[UK_NO_PPM], (long)uk_diag[UK_HOP], (long)uk_diag[UK_RANGE],
            (unsigned long)uk_n_nosent, (unsigned long)uk_n_notready);
  }
#endif

  if ((int32_t)(now - uk_next) < 0) return;      /* one slot every 40 ms */
  { uint8_t i2 = uk_read(UK_R_IRQ2);
    if (i2 & UK_IRQ2_SENT) uk_sent_seen = true;
    else if ((now - uk_tx_at) < UK_AIR_US) return;   /* v1.5: the previous packet is still on the air: the slot */
                                                     /* waits, it is not cut (V9 never cut one) */
    if (!uk_sent_seen && uk_diag[UK_TX] > 0) {   /* the previous packet never reported "sent": ask the chip why */
      uk_n_nosent++;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      if ((int32_t)(now - uk_nosent_log_at) >= 0) {
        uk_nosent_log_at = now + 2000000u;
        app_log("UK nosent: at +%lu ms op=%02X irq1=%02X irq2=%02X br=%02X pa=%02X fifothr=%02X paylen=%02X (TX 23, irq1 B0 = ready+lock)\r\n",
                (unsigned long)((now - uk_tx_at) / 1000u), (unsigned)uk_read(UK_R_OPMODE), (unsigned)uk_read(UK_R_IRQ1),
                (unsigned)i2, (unsigned)uk_read(UK_R_BRMSB), (unsigned)uk_read(UK_R_PACONFIG),
                (unsigned)uk_read(UK_R_FIFOTHR), (unsigned)uk_read(UK_R_PAYLEN));
      }
#endif
    }
  }
  uk_next += UK_PERIOD_US;
  if ((int32_t)(now - uk_next) >= 0) uk_next = now + UK_PERIOD_US;   /* far behind: restart the cadence */

  if (uk_hop_wait) { uk_diag[UK_HOP_LATE]++; uk_retune(); }   /* beacon out (checked above) but the loop missed it */
  uk_standby();

  if (uk_bind) {
    if (uk_bind_btn) {                           /* BIND held since power-up: keep binding, +1.5 s after release */
      if (bind_low) uk_bind_until = now + UK_AUTO_BIND_MS * 1000u;
      else uk_bind_btn = false;
    }
    if (!uk_bind_btn && (int32_t)(now - uk_bind_until) >= 0) {    /* bind over: data from the next slot */
      uk_bind = false; uk_diag[UK_PHASE] = 2;
      uk_hop = 0; uk_dwell = 0; uk_diag[UK_HOP] = 0;
      uk_set_freq(uk_hop_hz(0));
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("UK data: hopping from %lu Hz, %u channels, 5 packets a channel\r\n", (unsigned long)uk_hop_hz(0), (unsigned)uk_n);
#endif
      return;
    }
    uk_fill_channels(has_rc);
    uk_pkt[7] = uk_lane;                          /* teach the lane */
    uk_pkt[8] = uk_id[0];
    uk_pkt[9] = uk_id[1];
    uk_send(UK_PA_BIND);
    return;
  }

  if (!has_rc) { uk_diag[UK_NO_PPM]++; return; } /* V9: nothing sent without PPM -> receiver failsafe */
  if (!uk_active) {                              /* v1.1: held back -- keep the hop schedule running, silently */
    uk_diag[UK_QUIET]++;
    if (++uk_dwell >= UK_DWELL) { uk_dwell = 0; uk_hop_next = (uint8_t)((uk_hop + 1u) % uk_n); uk_retune(); }
    return;
  }
  uk_fill_channels(true);
  uint8_t pa = uk_range ? UK_PA_RANGE : UK_PA_FLIGHT;
  if (++uk_dwell >= UK_DWELL) {                  /* last packet on this channel: the sync beacon */
    uk_hop_next = (uint8_t)((uk_hop + 1u) % uk_n);
    uk_pkt[8] = (uint8_t)(uk_id[0] ^ uk_hop_next);
    uk_pkt[9] = 0xFEu;
    uk_send(pa);
    uk_diag[UK_BEACON]++;
    uk_dwell = 0;
    uk_hop_wait = true;
    uk_hop_poll = now + 25000u;                  /* the packet takes about 33 ms: look from 25 ms on */
  } else {
    uk_pkt[8] = uk_id[0];
    uk_pkt[9] = uk_id[1];
    uk_send(pa);
  }
}
