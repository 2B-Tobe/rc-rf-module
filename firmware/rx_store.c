/* rx_store.c — see rx_store.h.  Uses emlib MSC (component "emlib_msc"). */
#include "rx_store.h"
#include "em_device.h"
#include "em_msc.h"
#include <string.h>
#include <stddef.h>

#define RX_STORE_MAGIC  0x52583031u          /* "RX01" */

volatile int32_t rx_store_diag[8];
#define RX_STORE_SLOTS  8u
/* A normal main-flash page well above the ~115 KB application and clear of the
 * top pages (reserved / SE-locked on this part). USERDATA and the top page both
 * report write-OK but silently discard, so we sit at a safe 0x08040000. */
#define RX_STORE_BASE   ((uint32_t *)(FLASH_BASE + 0x40000u))
#define RX_STORE_STRIDE ((sizeof(rx_record_t) + 3u) & ~3u)

static uint16_t rx_store_crc(const rx_record_t *r)
{
  const uint8_t *p = (const uint8_t *)r;
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < offsetof(rx_record_t, crc); i++) {   /* bytes before crc; NOT sizeof-2 (tail padding) */
    c ^= (uint16_t)p[i] << 8;
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

static const rx_record_t *rx_store_at(uint8_t slot)
{
  return (const rx_record_t *)((const uint8_t *)RX_STORE_BASE + slot * RX_STORE_STRIDE);
}

bool rx_store_load(uint8_t slot, rx_record_t *r)
{
  if (slot >= RX_STORE_SLOTS) return false;
  memcpy(r, rx_store_at(slot), sizeof(*r));
  rx_store_diag[4] = (int32_t)r->magic;                            /* raw word physically at slot base */
  rx_store_diag[5] = (int32_t)((uint32_t)(uintptr_t)rx_store_at(slot));
  if (r->magic != RX_STORE_MAGIC || r->crc != rx_store_crc(r)) { memset(r, 0, sizeof(*r)); return false; }
  return true;
}

static bool rx_store_write_all(const rx_record_t *recs)
{
  MSC_Init();
  MSC_Status_TypeDef er = MSC_ErasePage(RX_STORE_BASE);
  rx_store_diag[0] = (int32_t)er;
  bool ok = (er == mscReturnOk);
  for (uint8_t i = 0; ok && i < RX_STORE_SLOTS; i++) {
    if (recs[i].magic != RX_STORE_MAGIC) continue;                 /* empty slot stays erased */
    MSC_Status_TypeDef wr = MSC_WriteWord((uint32_t *)((uint8_t *)RX_STORE_BASE + i * RX_STORE_STRIDE),
                                          &recs[i], RX_STORE_STRIDE);
    rx_store_diag[1] = (int32_t)wr;
    ok = (wr == mscReturnOk);
  }
  MSC_Deinit();
  return ok;
}

bool rx_store_save(uint8_t slot, rx_record_t *r)
{
  if (slot >= RX_STORE_SLOTS) return false;
  rx_record_t all[RX_STORE_SLOTS];
  for (uint8_t i = 0; i < RX_STORE_SLOTS; i++) if (!rx_store_load(i, &all[i])) memset(&all[i], 0, sizeof(all[i]));
  r->magic = RX_STORE_MAGIC;
  r->crc = rx_store_crc(r);
  all[slot] = *r;
  bool ok = rx_store_write_all(all);
  rx_store_diag[2] = ok ? 1 : 0;   /* MSC status; a read-back here is unreliable (flash read-cache) */
  rx_store_diag[3]++;
  return ok;
}

void rx_store_clear(uint8_t slot)
{
  if (slot >= RX_STORE_SLOTS) return;
  rx_record_t all[RX_STORE_SLOTS];
  for (uint8_t i = 0; i < RX_STORE_SLOTS; i++) if (!rx_store_load(i, &all[i])) memset(&all[i], 0, sizeof(all[i]));
  memset(&all[slot], 0, sizeof(all[slot]));
  (void)rx_store_write_all(all);
}
