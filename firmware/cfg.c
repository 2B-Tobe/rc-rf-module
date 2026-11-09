/* cfg.c — see cfg.h.  #included into app_process.c (after the role statics; it calls cfg_running_line() there).   v1.0 */
#include "cfg.h"
#include "em_device.h"
#include "em_msc.h"
#include "sl_cli.h"
#include "sl_cli_command.h"
#include "sl_rail_util_init.h"
#include <stdio.h>
#include <string.h>

/* The page after the bind store (rx_store.c: 0x08040000, 8 KB pages). Main flash, far above the firmware. */
#define CFG_BASE   ((uint32_t *)(FLASH_BASE + 0x42000u))
#define CFG_MAGIC  0x31474643u                                      /* "CFG1" */
#define CFG_BIND_WORD ((uint32_t *)(FLASH_BASE + 0x42000u + 16u))   /* one-shot: "cfg bind" writes it, the next boot takes it */
#define CFG_BIND_MAGIC 0x444E4942u                                  /* "BIND" */

typedef struct {
  uint32_t magic;
  uint8_t  version, role, proto, sub, option, outputs, rsvd[2];   /* outputs: 0xFF in a record from before v1.1 = 0 */
  uint32_t crc;                                                     /* over the 12 bytes before it */
} cfg_rec_t;

/* v1.1: the receiver's owners (rx_setup.c: the TX modules allowed to set it up over the air), at +64 in the
   same page. cfg_save keeps them; cfg clear erases them with everything else. */
#define CFG_OWN_BASE  ((uint32_t *)(FLASH_BASE + 0x42000u + 64u))
#define CFG_OWN_MAGIC 0x314E574Fu                                   /* "OWN1" */
#define CFG_OWNERS    3u
typedef struct {
  uint32_t magic;
  uint8_t  n, rsvd[3];
  uint8_t  id[CFG_OWNERS][8];                                       /* 6 used: EUI-64 low 48 bits */
  uint32_t crc;                                                     /* over the 32 bytes before it */
} cfg_own_t;

static void cfg_running_line(void);                                 /* app_process.c: what the module is doing right now */
static void cfg_blink(bool ok);                                     /* app_process.c: LEDs say it too - stored: both blink slowly 3x; refused: alternate fast 2 s */
static bool cfg_in_use(void);                                       /* app_process.c: a transmitter with its input present (PPM or encoder frames) refuses changes */

static uint32_t cfg_crc(const uint8_t *p, uint32_t n)               /* CRC-32 (IEEE), bit by bit: 12 bytes, once */
{
  uint32_t c = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < n; i++) {
    c ^= p[i];
    for (uint8_t k = 0; k < 8u; k++) c = (c & 1u) ? (c >> 1) ^ 0xEDB88320u : c >> 1;
  }
  return ~c;
}

bool cfg_load(cfg_t *c)
{
  const cfg_rec_t *r = (const cfg_rec_t *)CFG_BASE;
  if (r->magic != CFG_MAGIC || r->version != 1u) return false;
  if (r->crc != cfg_crc((const uint8_t *)r, 12u)) return false;
  if (r->role != CFG_ROLE_TX && r->role != CFG_ROLE_RX) return false;
  c->role = r->role; c->proto = r->proto; c->sub = r->sub; c->option = r->option;
  c->outputs = (r->outputs <= RXOUT_MAX) ? r->outputs : 0u;
  return true;
}

static bool cfg_own_load(cfg_own_t *o)
{
  const cfg_own_t *s = (const cfg_own_t *)CFG_OWN_BASE;
  if (s->magic != CFG_OWN_MAGIC || s->n > CFG_OWNERS || s->crc != cfg_crc((const uint8_t *)s, 32u)) return false;
  *o = *s;
  return true;
}
/* the page holds the setup record (+0), the one-shot bind word (+16) and the owners (+64): erase, write back */
static bool cfg_write_page(const cfg_t *c, const cfg_own_t *o)
{
  cfg_rec_t r;
  if (c) {
    memset(&r, 0xFF, sizeof r);
    r.magic = CFG_MAGIC; r.version = 1u; r.role = c->role; r.proto = c->proto; r.sub = c->sub; r.option = c->option; r.outputs = c->outputs;
    r.crc = cfg_crc((const uint8_t *)&r, 12u);
  }
  cfg_own_t ow;
  if (o) { ow = *o; ow.magic = CFG_OWN_MAGIC; ow.crc = cfg_crc((const uint8_t *)&ow, 32u); }
  MSC_Init();
  bool ok = MSC_ErasePage(CFG_BASE) == mscReturnOk;
  if (ok && c) ok = MSC_WriteWord(CFG_BASE, &r, sizeof r) == mscReturnOk;
  if (ok && o) ok = MSC_WriteWord(CFG_OWN_BASE, &ow, sizeof ow) == mscReturnOk;
  MSC_Deinit();
  return ok;
}

bool cfg_save(const cfg_t *c)
{
  cfg_own_t o;
  bool keep = cfg_own_load(&o);                                     /* the owners stay */
  bool ok = cfg_write_page(c, keep ? &o : NULL);
  cfg_t back;
  return ok && cfg_load(&back) && memcmp(&back, c, sizeof back) == 0;
}

bool cfg_owner_known(const uint8_t id[6])
{
  cfg_own_t o;
  if (!cfg_own_load(&o)) return false;
  for (uint8_t i = 0; i < o.n; i++) if (memcmp(o.id[i], id, 6) == 0) return true;
  return false;
}
uint8_t cfg_owner_count(void) { cfg_own_t o; return cfg_own_load(&o) ? o.n : 0u; }

bool cfg_owner_add(const uint8_t id[6])                             /* a 4th one pushes the oldest out */
{
  if (cfg_owner_known(id)) return true;
  cfg_own_t o;
  if (!cfg_own_load(&o)) { memset(&o, 0, sizeof o); }
  if (o.n >= CFG_OWNERS) { memmove(o.id[0], o.id[1], sizeof o.id[0] * (CFG_OWNERS - 1u)); o.n = CFG_OWNERS - 1u; }
  memset(o.id[o.n], 0, 8); memcpy(o.id[o.n], id, 6); o.n++;
  cfg_t c;
  bool have = cfg_load(&c);
  return cfg_write_page(have ? &c : NULL, &o) && cfg_owner_known(id);
}

bool cfg_clear(void)
{
  MSC_Init();
  bool ok = MSC_ErasePage(CFG_BASE) == mscReturnOk;
  MSC_Deinit();
  return ok;
}

static bool cfg_bind_request(void)                                  /* the word is still erased after a save: just program it */
{
  uint32_t w = CFG_BIND_MAGIC;
  MSC_Init();
  bool ok = *CFG_BIND_WORD == 0xFFFFFFFFu && MSC_WriteWord(CFG_BIND_WORD, &w, sizeof w) == mscReturnOk;
  MSC_Deinit();
  return ok && *CFG_BIND_WORD == CFG_BIND_MAGIC;
}
bool cfg_bind_pending(void) { return *CFG_BIND_WORD == CFG_BIND_MAGIC; }
void cfg_bind_consume(void)                                         /* erase the word: rewrite the record, or wipe the page */
{
  cfg_t c; cfg_own_t ow;
  bool have = cfg_load(&c), own = cfg_own_load(&ow);
  if (have || own) (void)cfg_write_page(have ? &c : NULL, own ? &ow : NULL); else cfg_clear();
}

/* ---------------------------------------------------------------- names: Prot / Sub in both roles */
typedef struct { const char *key; uint8_t proto, sub; const char *tx, *rx; } cfg_mode_t;
static const cfg_mode_t cfg_modes[] = {
  { "nrf24",  0u, 0u, "nRF24",                   "nRF24 receiver" },
  { "rlink",  0u, 1u, "RLINK",                   "RLINK receiver" },
  { "uk",     0u, 2u, "UK only (459 MHz)",       "UK 459 MHz receiver" },
  { "dsmcap", 0u, 4u, NULL,                      "DSM capture" },
  { "jeti",   0u, 6u, "Jeti capture",            "Jeti capture" },
  { "txtest", 0u, 7u, "TX test",                 NULL },
  { "d8",     3u, 0u, "FrSky D8",                "FrSky D8 receiver" },
  { "d16",   15u, 0u, "FrSky D16",               "FrSky D16 receiver" },
  { "dsmx",   6u, 2u, "DSMX 22 ms",              "DSMX receiver" },
#if !DTX_EU_LOCK
  { "dsmx11", 6u, 3u, "DSMX 11 ms",              "DSMX receiver" },
#endif
};
#define CFG_MODES (sizeof cfg_modes / sizeof cfg_modes[0])

const char *cfg_proto_name(uint8_t role, uint8_t proto, uint8_t sub)
{
  for (uint32_t i = 0; i < CFG_MODES; i++) {
    const cfg_mode_t *m = &cfg_modes[i];
    bool hit = (m->proto == proto) && (m->proto == 0u || m->proto == 6u ? m->sub == sub : true);   /* D8 / D16: any Sub */
    if (hit) return role == CFG_ROLE_RX ? m->rx : m->tx;
  }
  return NULL;
}

static bool cfg_eq(const char *a, const char *b)                    /* case-insensitive */
{
  for (;; a++, b++) {
    char x = *a, y = *b;
    if (x >= 'A' && x <= 'Z') x = (char)(x + 32);
    if (y >= 'A' && y <= 'Z') y = (char)(y + 32);
    if (x != y) return false;
    if (x == 0) return true;
  }
}

static bool cfg_num(const char *s, uint8_t *v)
{
  if (*s == 0) return false;
  uint32_t n = 0;
  for (; *s; s++) { if (*s < '0' || *s > '9') return false; n = n * 10u + (uint32_t)(*s - '0'); if (n > 255u) return false; }
  *v = (uint8_t)n; return true;
}

static void cfg_print(const char *tag, const cfg_t *c)
{
  const char *n = cfg_proto_name(c->role, c->proto, c->sub);
  bool opt = c->role == CFG_ROLE_TX && !(c->proto == 0u && c->sub >= 2u);   /* single / dual: transmitter modes with a 2.4 GHz link */
  printf("%s %s Prot %u Sub %u = %s%s%s%s\r\n", tag, c->role == CFG_ROLE_RX ? "RX" : "TX", c->proto, c->sub,
         n ? n : "not in this module", opt ? (c->option ? ", Opt dual" : ", Opt single") : "",
         c->role == CFG_ROLE_RX ? ", outputs " : "", c->role == CFG_ROLE_RX ? servo_out_layout_name(c->outputs) : "");
}

static void cfg_restart(void)
{
  sl_rail_handle_t rh = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
  uint32_t t0 = (uint32_t)sl_rail_get_time(rh);
  while (((uint32_t)sl_rail_get_time(rh) - t0) < 30000u) { }     /* let the reply out */
  NVIC_SystemReset();
}

static void cfg_cli(sl_cli_command_arg_t *a)
{
  int n = sl_cli_get_argument_count(a);
  const char *w = n > 0 ? sl_cli_get_argument_string(a, 0) : "show";
  cfg_t c;
  if (cfg_eq(w, "show")) {
    if (cfg_owner_count()) printf("CFG owners (TX modules that may set this receiver up over the air): %u\r\n", cfg_owner_count());
    if (cfg_load(&c)) cfg_print("CFG stored:", &c);
    else printf("CFG nothing stored: the jumpers decide (F/S receiver, RANGE at power-up serial, PC00..PC02 protocol)\r\n");
    cfg_running_line();
    return;
  }
  if (cfg_in_use()) {                                                /* only a receiver, or a transmitter with no input, can be set up */
    printf("CFG refused: this transmitter is in use (input present). Unplug its input to set it up\r\n");
    cfg_blink(false);
    return;
  }
  if (cfg_eq(w, "bind")) {
    bool ok = cfg_bind_request();
    printf(ok ? "CFG bind: restarting with BIND held for 8 s\r\n" : "CFG bind failed (a bind is already pending?)\r\n");
    cfg_blink(ok);
    if (ok) cfg_restart();
    return;
  }
  if (cfg_eq(w, "clear")) {
    bool ok = cfg_clear();
    printf(ok ? "CFG cleared: the jumpers decide again. Restarting\r\n" : "CFG clear failed\r\n");
    cfg_blink(ok);
    cfg_restart();
    return;
  }
  bool tx = cfg_eq(w, "tx"), rx = cfg_eq(w, "rx");
  if ((!tx && !rx) || n < 2) {
    printf("CFG usage: cfg | cfg tx <proto> [dual] | cfg rx <proto> [8pwm|5sbus|5ppm|5both] | cfg bind | cfg clear\r\n"
           "    proto: nrf24 rlink uk d8 d16 dsmx jeti txtest(tx) dsmcap(rx), or the serial frame numbers: Prot Sub\r\n");
    cfg_blink(false);
    return;
  }
  memset(&c, 0, sizeof c);
  c.role = tx ? CFG_ROLE_TX : CFG_ROLE_RX;
  int i = 1;
  const char *p = sl_cli_get_argument_string(a, i);
  uint8_t num;
  if (cfg_num(p, &num)) {                                           /* numbers: Prot [Sub] */
    c.proto = num; c.sub = 0; i++;
    if (i < n && cfg_num(sl_cli_get_argument_string(a, i), &num)) { c.sub = num; i++; }
  } else {
    bool found = false;
    for (uint32_t k = 0; k < CFG_MODES; k++) if (cfg_eq(p, cfg_modes[k].key)) { c.proto = cfg_modes[k].proto; c.sub = cfg_modes[k].sub; found = true; break; }
    if (!found) { printf("CFG unknown protocol '%s': nrf24 rlink uk d8 d16 dsmx jeti txtest dsmcap\r\n", p); cfg_blink(false); return; }
    i++;
  }
  if (i < n && rx) {                                                /* receiver: the output layout */
    const char *o = sl_cli_get_argument_string(a, i);
    if (cfg_eq(o, "8pwm")) c.outputs = 0u; else if (cfg_eq(o, "5sbus")) c.outputs = RXOUT_SBUS;
    else if (cfg_eq(o, "5ppm")) c.outputs = RXOUT_PPM; else if (cfg_eq(o, "5both")) c.outputs = RXOUT_SBUS | RXOUT_PPM;
    else if (cfg_num(o, &num) && num <= RXOUT_MAX) c.outputs = num;
    else { printf("CFG unknown output layout '%s': 8pwm | 5sbus | 5ppm | 5both\r\n", o); cfg_blink(false); return; }
  } else if (i < n) {
    const char *o = sl_cli_get_argument_string(a, i);
    if (cfg_eq(o, "dual")) c.option = 1u; else if (cfg_eq(o, "single")) c.option = 0u;
    else if (cfg_num(o, &num)) c.option = num;
    else { printf("CFG unknown option '%s': single | dual\r\n", o); cfg_blink(false); return; }
  }
  if (cfg_proto_name(c.role, c.proto, c.sub) == NULL) { printf("CFG Prot %u Sub %u is not a %s mode of this module\r\n", c.proto, c.sub, tx ? "transmitter" : "receiver"); cfg_blink(false); return; }
  if (!cfg_save(&c)) { printf("CFG store failed\r\n"); cfg_blink(false); return; }
  cfg_print("CFG stored:", &c);
  printf("CFG restarting with it%s\r\n", tx ? " (no jumpers = encoder MULTI frames or PPM, whichever comes first; F/S low = encoder serial only)" : " (F/S high = this protocol, F/S low = jumpers)");
  cfg_blink(true);
  cfg_restart();
}

static const sl_cli_command_info_t cfg_cmd_info = SL_CLI_COMMAND(cfg_cli,
  "RF module setup: cfg | cfg tx <proto> [dual] | cfg rx <proto> [8pwm|5sbus|5ppm|5both] | cfg bind | cfg clear", "",
  { SL_CLI_ARG_STRINGOPT, SL_CLI_ARG_END, });
static const sl_cli_command_entry_t cfg_cmd_table[] = {
  { "cfg", &cfg_cmd_info, false },
  { NULL, NULL, false },
};
static sl_cli_command_group_t cfg_cmd_group = { { NULL }, false, cfg_cmd_table };

void cfg_cli_init(void)
{
  sl_cli_command_add_command_group(sl_cli_default_handle, &cfg_cmd_group);
}
