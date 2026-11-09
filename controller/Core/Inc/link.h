/*
 * link.h - talking to the RF module's "cfg" command.
 *
 * The module takes one text line at a time on J3 (115200 8N1) and answers with lines starting "CFG ":
 *   cfg                        -> "CFG stored: RX Prot 0 Sub 1 = RLINK receiver[, Opt dual]" or "CFG nothing stored ..."
 *                                 and "CFG running now: <mode>, input <source>, F/S <high|low> (...)"
 *   cfg rx <Prot> <Sub> <out>  -> "CFG stored: ..., outputs <layout>" then "CFG restarting ..."   (the module restarts;
 *                                 out 0 8 PWM, 1 5 PWM + SBUS, 2 5 PWM + PPM, 3 5 PWM + SBUS + PPM)
 *   cfg tx <Prot> <Sub> <dual|single>      (Prot / Sub: the numbers of the transmitter's serial frames)
 *   cfg bind                   -> "CFG bind: ..."                                  (restarts into a bind window)
 *   cfg clear                  -> "CFG cleared ..."
 *   anything refused           -> "CFG <reason>"
 * link_command() sends a line, listens for wait_ms and distils the answer into short display strings.
 */
#ifndef LINK_H
#define LINK_H
#include <stdint.h>
#include <stdbool.h>

#define LINK_TEXT 19                                               /* 18 characters + terminator: one display row */

typedef struct {
  bool answered;                   /* at least one "CFG" line came back */
  char stored[LINK_TEXT];          /* "RX RLINK receiver" / "Nothing stored" */
  char option[LINK_TEXT];          /* "Opt dual" / "" */
  char now[LINK_TEXT];             /* "Now: RLINK receiver" */
  char input[LINK_TEXT];           /* "radio F/S high" */
  char note[LINK_TEXT];            /* "Stored, restarting", "Bind: 8 s window", "Cleared", or a refusal */
  char raw[37];                    /* the first bytes received, printable, for the no-answer page */
  bool setup;                      /* "CFG stored:" came back: the numbers below are valid */
  bool setup_tx;                   /* stored role: true = TX */
  uint8_t prot, sub;               /* stored Prot / Sub */
  int8_t opt;                      /* stored Opt: 1 dual, 0 single, -1 not shown */
  int8_t outputs;                  /* stored receiver output layout 0..3, -1 not shown (older module firmware) */
} link_reply_t;

void link_command(const char *cmd, uint32_t wait_ms, link_reply_t *reply);
#endif
