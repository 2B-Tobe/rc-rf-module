/* link.c - see link.h. */
#include "link.h"
#include "serial.h"
#include "board.h"
#include <string.h>

/* copy at most n characters, drop trailing blanks */
static void take(char *dst, const char *src, uint8_t n)
{
  uint8_t i = 0;
  while (i < n && src[i]) { dst[i] = src[i]; i++; }
  while (i > 0u && dst[i - 1u] == ' ') i--;
  dst[i] = 0;
}

/* copy up to the next comma (or end), at most n characters */
static void take_field(char *dst, const char *src, uint8_t n)
{
  uint8_t i = 0;
  while (i < n && src[i] && src[i] != ',') { dst[i] = src[i]; i++; }
  dst[i] = 0;
}

static const char *past(const char *line, const char *tag)
{
  const char *p = strstr(line, tag);
  return p ? p + strlen(tag) : 0;
}

static uint8_t number_at(const char *p)
{
  uint16_t n = 0;
  while (p && *p >= '0' && *p <= '9' && n < 256u) { n = (uint16_t)(n * 10u + (uint16_t)(*p - '0')); p++; }
  return (uint8_t)n;
}

static void parse(link_reply_t *r, const char *line)
{
  if (strncmp(line, "CFG ", 4) != 0) return;
  r->answered = true;
  const char *p;
  if ((p = past(line, "CFG stored: ")) != 0) {
    /* "<RX|TX> Prot <n> Sub <n> = <name>[, Opt <single|dual>]" */
    char text[48];
    text[0] = p[0]; text[1] = p[1]; text[2] = ' '; text[3] = 0;
    const char *name = past(p, "= ");
    if (name) { char field[40]; take_field(field, name, 39); strcat(text, field); }
    take(r->stored, text, LINK_TEXT - 1);
    r->setup = true;
    r->setup_tx = (p[0] == 'T');
    r->prot = number_at(past(p, "Prot "));
    r->sub  = number_at(past(p, "Sub "));
    const char *opt = past(p, ", Opt ");
    r->opt = opt ? (strncmp(opt, "dual", 4) == 0 ? 1 : 0) : -1;
    if (opt) { char text2[24]; strcpy(text2, "Opt "); take_field(text2 + 4, opt, 12); take(r->option, text2, LINK_TEXT - 1); }
    const char *out = past(p, ", outputs ");                       /* receiver: "8 PWM", "5 PWM + SBUS", "5 PWM + PPM", "5 PWM + SBUS + PPM" */
    if (out) {
      static const char *const shown[4] = { "Out 8 PWM", "Out 5 PWM + SBUS", "Out 5 PWM + PPM", "Out 5+SBUS+PPM" };
      bool sbus = strncmp(out, "5 PWM + SBUS", 12) == 0, ppm = strstr(out, "PPM") != 0;
      r->outputs = (int8_t)((sbus ? 1 : 0) | (ppm ? 2 : 0));
      strcpy(r->option, shown[r->outputs]);
    }
  } else if (past(line, "CFG nothing stored")) {
    strcpy(r->stored, "Nothing stored");
    strcpy(r->option, "jumpers decide");
  } else if ((p = past(line, "CFG running now: ")) != 0) {
    /* "<mode>, input <source>, F/S <level> (...)" */
    char text[48] = "Now: ";
    char field[40];
    take_field(field, p, 39);
    strcat(text, field);
    take(r->now, text, LINK_TEXT - 1);
    const char *src = past(p, ", input ");
    if (src) {
      take_field(text, src, 39);
      const char *fs = past(p, ", F/S ");
      if (fs) { strcat(text, " F/S "); char lvl[6]; take(lvl, fs, 4); strcat(text, lvl); }
      take(r->input, text, LINK_TEXT - 1);
    }
  } else if (past(line, "CFG bind:")) {
    strcpy(r->note, "Bind: 8 s window");
  } else if (past(line, "CFG restarting")) {
    strcpy(r->note, "Stored, restarting");
  } else if (past(line, "CFG cleared")) {
    strcpy(r->note, "Cleared: jumpers");
  } else if (!r->note[0]) {
    take(r->note, line + 4, LINK_TEXT - 1);                        /* a refusal or an error, as the module words it */
  }
}

void link_command(const char *cmd, uint32_t wait_ms, link_reply_t *r)
{
  char line[120];
  uint8_t n = 0, raw_n = 0;
  memset(r, 0, sizeof *r);
  r->outputs = -1; r->opt = -1;
  serial_listen();
  serial_send(cmd);
  serial_send("\r");
  uint32_t start = board_ms();
  while ((board_ms() - start) < wait_ms) {
    int c = serial_read();
    if (c < 0) continue;
    if (raw_n < 36u) { r->raw[raw_n++] = (c >= 32 && c < 127) ? (char)c : '.'; r->raw[raw_n] = 0; }
    if (c == '\r' || c == '\n') {
      if (n) { line[n] = 0; parse(r, line); n = 0; }
    } else if (n < sizeof line - 1u) {
      line[n++] = (char)c;
    }
  }
  if (n) { line[n] = 0; parse(r, line); }
}
