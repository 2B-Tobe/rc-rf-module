/*
 * rx_store.h — receiver settings kept in the EFR32MG24 USERDATA flash page
 * in the last page of main flash (untouched by a normal app reflash).
 * One record per protocol: bind (TX id + hop table; nRF24: the pipe in hop[0..3]) and the failsafe preset.
 * Pulled into app_process.c with #include "rx_store.c" ahead of the FrSky files.
 */
#ifndef RX_STORE_H
#define RX_STORE_H
#include <stdint.h>
#include <stdbool.h>

#define RX_SLOT_D8   0u
#define RX_SLOT_D16  1u
#define RX_SLOT_NRF  2u
/* slot 3: DSMX receiver (RX_SLOT_DSM, dsm_rx.h); slot 4: DSMX transmitter frame rate (RX_SLOT_DSMTX, dsm_tx.h) */
#define RX_SLOT_RLINK 5u
#define RX_SLOT_RLINKTX 6u
/* slot 7: UK 459 MHz receiver (UR_SLOT, uk459_rx.c): ID in hop[0..1], lane in hop[2] */

typedef struct {
  uint32_t magic;
  uint8_t  id2, id3;
  uint8_t  fs_valid;          /* 1 = fs_us holds a stored failsafe preset */
  uint8_t  bound;             /* 1 = id/hop valid                          */
  uint8_t  hop[52];
  uint16_t fs_us[8];
  uint16_t crc;
} rx_record_t;                /* 80 bytes with padding */

extern volatile int32_t rx_store_diag[8];   /* [0] last erase status, [1] last write status,
                                             * [2] write status ok, [3] save count, [4] raw magic word at slot base, [5] slot addr */
bool rx_store_load(uint8_t slot, rx_record_t *r);      /* false = nothing valid stored */
bool rx_store_save(uint8_t slot, rx_record_t *r);      /* fills magic/crc, rewrites the page */
void rx_store_clear(uint8_t slot);
#endif
