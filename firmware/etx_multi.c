/* etx_multi.c — see etx_multi.h.  #included into app_process.c (after bay_serial.c).   v1.0 */
#include "etx_multi.h"
#include <string.h>

/* The protocol list EdgeTX scans. Sub protocol names keep the Multiprotocol positions (EdgeTX sends the position):
 * fixed length, NUL-padded. opt = the option text EdgeTX shows (0 none, 1 "Option value", 7 "Enable max throw"). */
typedef struct {
  uint8_t     num;
  const char *name;            /* 7 characters at most; must not end in "RX" (EdgeTX takes those as receivers) */
  uint8_t     opt;
  uint8_t     nsub, sublen;
  const char *subs;            /* nsub x sublen bytes */
} etx_proto_t;

static const etx_proto_t etx_protos[] = {
  {   3u, "FrSkyD",  1u, 1u, 2u, "D8" },
  {   6u, "DSM",     7u, 4u, 7u, "DSM2_1F" "DSM2_2F" "DSMX_1F" "DSMX_2F" },
  {  15u, "FrSkyX",  1u, 3u, 5u, "CH_16" "CH_8\0" "EU_16" },
  {  64u, "FrSkyX2", 1u, 3u, 5u, "CH_16" "CH_8\0" "EU_16" },
  { ETX_PROTO_MITO, "MiTo", 1u, 3u, 5u, "nRF24" "RLINK" "UK459" },
};
#define ETX_NPROTO  ((uint8_t)(sizeof etx_protos / sizeof etx_protos[0]))

static const etx_proto_t *etx_find(uint8_t num)
{
  for (uint8_t i = 0; i < ETX_NPROTO; i++) if (etx_protos[i].num == num) return &etx_protos[i];
  return 0;
}

/* Single / dual (the 459 MHz link beside 2.4 GHz) from EdgeTX: the module's Opt is 0 = single, 1 = dual.
 *   MiTo:   the Option value as sent (0 single, anything else dual), as for a normal-polarity sender
 *   FrSky:  Option value 1 = dual; any other value single (a model from a 4-in-1 may carry an RF tune there)
 *   DSM:    EdgeTX packs its own settings into the option byte; its "Max throw" box (bit 7) = dual */
void etx_translate(bay_frame_t *f)
{
  uint8_t p = ETX_NONE, s = 0;
  bool keep_opt = false;
  int8_t dual = (f->option == 1) ? 1 : 0;
  switch (f->wproto) {
    case 3u:  if (f->wsub == 0u) { p = 3u; s = 0u; } break;                 /* D8 (not Cloned) */
    case 6u:  p = 6u; s = f->wsub;                                         /* the same Sub numbers: 2 / 3 = DSMX 22 / 11 ms */
              dual = ((uint8_t)f->option & 0x80u) ? 1 : 0; break;          /* "Max throw" */
    case 15u: if (f->wsub == 0u) { p = 15u; s = 2u; }                      /* CH_16 = ACCST v1 FCC        */
              else if (f->wsub == 1u) { p = 15u; s = 5u; }                 /* CH_8  = v1 FCC, 8 channels  */
              else if (f->wsub == 2u) { p = 15u; s = 0u; }                 /* EU_16 = v1 EU-LBT           */
              break;
    case 64u: if (f->wsub == 0u) { p = 15u; s = 3u; }                      /* FrSkyX2 CH_16 = ACCST v2 FCC */
              else if (f->wsub == 2u) { p = 15u; s = 1u; }                 /* FrSkyX2 EU_16 = v2 EU-LBT    */
              break;
    case ETX_PROTO_MITO:
              if (f->wsub <= 2u) { p = 0u; s = f->wsub; keep_opt = true; }  /* nRF24 / RLINK / UK only; Option = dual */
              break;
    default:  dual = 0; break;                                             /* 0 = the scan (its index is in the option), or not in this module */
  }
  f->proto = p;
  f->sub = s;
  if (!keep_opt) f->option = dual;                                         /* 0 single, 1 dual */
}

static uint8_t etx_put_name(uint8_t *d, const char *s, uint8_t n)          /* fixed field, NUL-padded */
{
  uint8_t i = 0;
  for (; i < n && s && s[i]; i++) d[i] = (uint8_t)s[i];
  for (; i < n; i++) d[i] = 0;
  return n;
}

uint8_t etx_build_status(uint8_t *b, const bay_frame_t *f, bool valid, bool binding)
{
  const etx_proto_t *e = etx_find(f->wproto);
  uint8_t *d = b + 4;
  b[0] = 'M'; b[1] = 'P'; b[2] = 0x01u; b[3] = 24u;
  d[0] = (uint8_t)(0x01u | 0x02u | (valid ? 0x04u : 0u) | (binding ? 0x08u : 0u));   /* input, serial; no failsafe */
  d[1] = 1u; d[2] = 3u; d[3] = 4u; d[4] = ETX_VERSION_PATCH;
  d[5] = 0xE4u;                                                            /* channel order AETR */
  uint8_t next = etx_protos[0].num, prev = etx_protos[ETX_NPROTO - 1u].num;
  for (uint8_t i = 0; i < ETX_NPROTO; i++) if (etx_protos[i].num > f->wproto) { next = etx_protos[i].num; break; }
  for (uint8_t i = ETX_NPROTO; i-- > 0u; )  if (etx_protos[i].num < f->wproto) { prev = etx_protos[i].num; break; }
  d[6] = next; d[7] = prev;
  etx_put_name(&d[8], e ? e->name : 0, 7u);
  d[15] = e ? (uint8_t)((e->nsub & 0x0Fu) | (e->opt << 4)) : 0u;
  if (e && f->wsub < e->nsub) {
    char sn[9]; memset(sn, 0, sizeof sn);
    memcpy(sn, e->subs + (uint16_t)f->wsub * e->sublen, e->sublen > 8u ? 8u : e->sublen);
    etx_put_name(&d[16], sn, 8u);
  } else etx_put_name(&d[16], 0, 8u);
  return 28u;
}

uint8_t etx_build_scan(uint8_t *b, uint8_t idx)
{
  uint8_t n = 4;
  b[0] = 'M'; b[1] = 'P'; b[2] = 0x11u;
  if (idx == 0xFFu) { b[n++] = ETX_NPROTO; }                               /* first request: how many follow */
  else if (idx >= ETX_NPROTO) { b[n++] = 0xFFu; }                          /* past the end: the list is over  */
  else {
    const etx_proto_t *e = &etx_protos[idx];
    b[n++] = e->num;
    for (const char *s = e->name; *s; s++) b[n++] = (uint8_t)*s;
    b[n++] = 0;
    b[n++] = (uint8_t)(e->opt << 4);                                       /* flags: option text; no failsafe */
    b[n++] = e->nsub;
    if (e->nsub) {
      b[n++] = e->sublen;
      memcpy(&b[n], e->subs, (uint16_t)e->nsub * e->sublen);
      n = (uint8_t)(n + e->nsub * e->sublen);
    }
  }
  b[3] = (uint8_t)(n - 4u);
  return n;
}

static uint8_t clamp_u8(int32_t v, int32_t lo, int32_t hi) { return (uint8_t)(v < lo ? lo : v > hi ? hi : v); }

uint8_t etx_rssi_figure(const bay_telem_t *t)                              /* EdgeTX's RSSI: 1..100, 0 = none */
{
  if (t->rssi_rx == BAY_NO_RSSI) return 0;
  return t->rssi_rx_dbm ? clamp_u8((int32_t)t->rssi_rx + 130, 1, 100) : clamp_u8(t->rssi_rx, 1, 100);
}

static uint8_t etx_vbat_raw(const bay_telem_t *t)                         /* A1 / RxBt: 13.2 V = 255 */
{
  return t->vbat_sensor ? clamp_u8(((int32_t)t->vbat_10mv * 255 + 660) / 1320, 0, 255) : 0u;
}

uint8_t etx_build_hub(uint8_t *b, const bay_telem_t *t)
{
  b[0] = 'M'; b[1] = 'P'; b[2] = 0x03u; b[3] = 4u;
  b[4] = 0xFEu;                                                            /* link frame */
  b[5] = etx_vbat_raw(t);                                                  /* A1 */
  b[6] = 0;                                                                /* A2 */
  b[7] = etx_rssi_figure(t);
  return 8u;
}

uint8_t etx_build_sport(uint8_t *b, uint16_t id, uint32_t value)
{
  uint8_t *p = b + 4;
  b[0] = 'M'; b[1] = 'P'; b[2] = 0x02u; b[3] = 9u;
  p[0] = 0x98u;                                                            /* physical ID 24 (a receiver's) */
  p[1] = 0x10u;                                                            /* data frame */
  p[2] = (uint8_t)id; p[3] = (uint8_t)(id >> 8);
  p[4] = (uint8_t)value; p[5] = (uint8_t)(value >> 8); p[6] = (uint8_t)(value >> 16); p[7] = (uint8_t)(value >> 24);
  uint16_t crc = 0;
  for (uint8_t i = 1; i < 8u; i++) { crc = (uint16_t)(crc + p[i]); crc = (uint16_t)((crc + (crc >> 8)) & 0xFFu); }
  p[8] = (uint8_t)(0xFFu - crc);
  return 13u;
}

/* ---- the turn-taking ---- */
static uint32_t etx_status_us, etx_tel_us;
static bool     etx_status_ever, etx_tel_alt;
static uint16_t etx_scan_last = 0x100u;                                    /* the scan index answered last (0x100: none) */
uint32_t        etx_sent;                                                  /* frames sent to the radio */

void etx_service(const bay_frame_t *f, uint32_t now_us, bool valid, uint8_t tel_kind, const bay_telem_t *t)
{
  uint8_t b[BAY_TXBUF];
  uint8_t n = 0;
  if (!f->scan) etx_scan_last = 0x100u;                                    /* a new scan starts at 0xFF again */
  if (bay_tx_busy()) return;
  if (f->scan && f->scan_idx != etx_scan_last) {                           /* one answer per index: EdgeTX counts them */
    n = etx_build_scan(b, f->scan_idx);
    etx_scan_last = f->scan_idx;
  } else if (!etx_status_ever || (uint32_t)(now_us - etx_status_us) >= ETX_STATUS_MS * 1000u) {
    n = etx_build_status(b, f, valid, t->binding);
    etx_status_us = now_us; etx_status_ever = true;
  } else if (tel_kind != ETX_TEL_NONE && t->link && (uint32_t)(now_us - etx_tel_us) >= ETX_TEL_MS * 1000u) {
    etx_tel_us = now_us;
    if (tel_kind == ETX_TEL_HUB) n = etx_build_hub(b, t);
    else {
      etx_tel_alt = !etx_tel_alt;
      if (etx_tel_alt || !t->vbat_sensor) {                                /* RSSI, with TX RSSI and TX LQI packed in */
        uint8_t trss = t->rssi_tx_valid ? clamp_u8((int32_t)t->rssi_tx + 130, 0, 127) : 0u;
        uint8_t tq   = (t->lq != BAY_NO_LQ) ? t->lq : 0u;
        n = etx_build_sport(b, 0xF101u, (uint32_t)etx_rssi_figure(t) | ((uint32_t)(trss << 1) << 8) | ((uint32_t)tq << 24));
      } else n = etx_build_sport(b, 0xF104u, etx_vbat_raw(t));           /* RxBt */
    }
  }
  if (n && bay_send_raw(b, n)) etx_sent++;
}
