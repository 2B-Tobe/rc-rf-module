/*
 * RF Module Controller - main.c
 *
 * A hand-held box that sets up the EFR32 RF module (rc_module_trx firmware) without jumpers: role, protocol,
 * single/dual link, and a jumper-less bind. It sends the module's "cfg" text commands over J3 and shows the
 * answer. See README.md and docs/MANUAL.md.
 *
 * The setup is sent as numbers, "cfg tx <Prot> <Sub> <dual|single>" / "cfg rx <Prot> <Sub>": the same Prot / Sub /
 * Opt a transmitter puts in its serial frames, so a module set up here runs exactly as one fed serial frames,
 * and a receiver set up here matches a transmitter running that Prot / Sub (FrSky D16: the Sub is the version).
 *
 * Screen (18 columns x 5 rows):
 *   row 0  what the module has stored (read at power-up), or the title
 *   row 1  Role    RX / TX
 *   row 2  Proto   the protocol
 *   row 3  Opt     Single / Dual        (transmitter modes with a 2.4 GHz link)
 *          Out     8 PWM / 5+SBUS / 5+PPM / 5+S+P   (receiver: its output layout - SBUS on servo connector 6,
 *                                                     PPM on connector 7; module firmware with rx-setup)
 *   row 4  Send    Bind                 (two cursor positions)
 * Keys: INC / DEC move the cursor; ENTER on a value starts editing (shown reversed), INC / DEC step it,
 *       ENTER finishes; ENTER on Send stores, on Bind binds; BACK reads the module; BACK held 2 s clears
 *       its setup; AUX = Bind.
 */
#include "board.h"
#include "display.h"
#include "serial.h"
#include "link.h"
#include <string.h>

#define VERSION "1.3"

/* ------------------------------------------------------------------ the module's modes, in Prot / Sub numbering */
typedef struct {
  uint8_t prot, sub;       /* the module's Prot / Sub: the numbers of the serial frames */
  const char *label;       /* on the screen: 9 characters at most */
  bool tx, rx;             /* which roles have it */
  bool has_option;         /* single / dual applies (a transmitter mode with a 2.4 GHz link) */
} mode_t;

static const mode_t modes[] = {
  {  0, 0, "nRF24",     true,  true,  true  },
  {  0, 1, "RLINK",     true,  true,  true  },
  {  0, 2, "UK-459",    true,  true,  false },
  {  3, 0, "FrSky D8",  true,  true,  true  },
  { 15, 0, "D16v1 EU",  true,  true,  true  },   /* ACCST D16 v1 EU-LBT */
  { 15, 1, "D16v2 EU",  true,  true,  true  },   /* ACCST D16 v2 EU-LBT */
  { 15, 2, "D16v1 FCC", true,  true,  true  },   /* ACCST D16 v1 FCC */
  { 15, 3, "D16v2 FCC", true,  true,  true  },   /* ACCST D16 v2 FCC */
  { 15, 5, "D16 FCC8",  true,  true,  true  },   /* ACCST D16 v1 FCC, channels 1-8 only */
  {  6, 2, "DSMX",      true,  true,  true  },   /* DSMX 22 ms */
  {  0, 6, "Jeti cap",  true,  true,  false },
  {  0, 7, "TX test",   true,  false, false },
  {  0, 4, "DSM cap",   false, true,  false },
};
#define MODE_COUNT ((int)(sizeof modes / sizeof modes[0]))

/* ------------------------------------------------------------------ state */
enum { ROW_ROLE = 0, ROW_PROTO, ROW_OPTION, ROW_SEND, ROW_BIND };
enum { PAGE_MENU, PAGE_ANSWER, PAGE_BUSY };

static bool     role_tx;
static int      mode_idx = 1;
static bool     option_dual = true;
static uint8_t  rx_out;                    /* receiver output layout 0..3, sent as the number after Sub */
static const char *const out_names[4] = { "8 PWM", "5+SBUS", "5+PPM", "5+S+P" };
static uint8_t  cursor = ROW_ROLE;
static bool     editing;
static uint8_t  page = PAGE_MENU;
static const char *busy_text = "";
static link_reply_t reply;

static bool mode_fits(int i)      { return role_tx ? modes[i].tx : modes[i].rx; }
static bool option_shown(void)    { return role_tx ? modes[mode_idx].has_option : true; }   /* RX: Out, always */

static void step_mode(int dir)
{
  int i = mode_idx;
  do {
    i += dir;
    if (i < 0) i = MODE_COUNT - 1;
    if (i >= MODE_COUNT) i = 0;
  } while (!mode_fits(i));
  mode_idx = i;
}

/* INC / DEC while editing the value under the cursor */
static void edit_step(int dir)
{
  if (cursor == ROW_ROLE) {
    role_tx = !role_tx;
    if (!mode_fits(mode_idx)) step_mode(1);
  } else if (cursor == ROW_PROTO) {
    step_mode(dir);
  } else if (role_tx) {
    option_dual = !option_dual;
  } else {
    rx_out = (uint8_t)((rx_out + (dir > 0 ? 1u : 3u)) & 3u);
  }
}

static void number(char *dst, uint32_t v)
{
  char tmp[11]; int n = 0;
  do { tmp[n++] = (char)('0' + v % 10u); v /= 10u; } while (v);
  while (n) *dst++ = tmp[--n];
  *dst = 0;
}

/* the table entry for a stored Prot / Sub (FrSky D8 takes any Sub), or -1 */
static int mode_of(uint8_t prot, uint8_t sub)
{
  for (int i = 0; i < MODE_COUNT; i++)
    if (modes[i].prot == prot && (modes[i].sub == sub || prot == 3u)) return i;
  return -1;
}

/* row 1: "TX D16v1 FCC" from the numbers when the box knows them, else the module's own words */
static void stored_text(char *dst)
{
  int i = reply.setup ? mode_of(reply.prot, reply.sub) : -1;
  if (i < 0) { strcpy(dst, reply.stored); return; }
  strcpy(dst, reply.setup_tx ? "TX " : "RX ");
  strcat(dst, modes[i].label);
  if (reply.setup_tx && modes[i].has_option && reply.opt >= 0) strcat(dst, reply.opt ? " D" : " S");
  if (!reply.setup_tx && reply.outputs > 0) { static const char *const suf[4] = { "", " +S", " +P", " +SP" }; strcat(dst, suf[reply.outputs & 3]); }
}

/* ------------------------------------------------------------------ pages */
static void page_menu(void)
{
  char top[LINK_TEXT + 4];
  if (reply.answered && reply.stored[0]) stored_text(top); else strcpy(top, "RF MODULE SETUP");
  draw_text(0, 0, top);
  draw_text(2, 1, "Role");
  draw_text(2, 2, "Proto");
  draw_text(2, 3, role_tx ? "Opt" : "Out");
  draw_text(2, 4, "Send");
  draw_text(11, 4, "Bind");

  const char *opt = !option_shown() ? "-" : role_tx ? (option_dual ? "Dual" : "Single") : out_names[rx_out & 3u];
  const char *values[3] = { role_tx ? "TX" : "RX", modes[mode_idx].label, opt };
  for (uint8_t r = 0; r < 3u; r++) {
    if (editing && cursor == r) draw_text_reversed(9, (uint8_t)(r + 1u), values[r]);
    else draw_text(9, (uint8_t)(r + 1u), values[r]);
  }
  if (cursor == ROW_BIND) draw_cursor(9, 4); else draw_cursor(0, (uint8_t)(cursor + 1u));
}

static void page_answer(void)
{
  if (!reply.answered) {
    char text[20], num[12];
    uint32_t frames = serial_frames();
    draw_text(0, 0, "No answer on J3");
    if (frames) {                                                  /* bytes came but made no sense: show them */
      char part[19];
      strncpy(part, reply.raw, 18); part[18] = 0; draw_text(0, 1, part);
      strncpy(part, reply.raw + (strlen(reply.raw) > 18u ? 18u : strlen(reply.raw)), 18); part[18] = 0; draw_text(0, 2, part);
    } else {
      draw_text(0, 1, "nothing received:");
      draw_text(0, 2, "wires? module on?");
    }
    strcpy(text, "bytes "); number(num, frames); strcat(text, num);
    strcat(text, " bad "); number(num, serial_errors()); strcat(text, num);
    strcat(text, serial_inverted() ? " L" : " H");
    draw_text(0, 3, text);
    draw_text(0, 4, "any key: menu");
    return;
  }
  char top[LINK_TEXT + 4];
  stored_text(top);
  draw_text(0, 0, reply.note[0] ? reply.note : "Module answers:");
  draw_text(0, 1, top);
  draw_text(0, 2, reply.option);
  draw_text(0, 3, reply.now);
  draw_text(0, 4, reply.input[0] ? reply.input : "any key: menu");
}

static void redraw(void)
{
  display_begin();
  do {
    if (page == PAGE_MENU) page_menu();
    else if (page == PAGE_ANSWER) page_answer();
    else { draw_text(0, 1, busy_text); draw_text(0, 3, "..."); }
  } while (display_end());
}

static void show_busy(const char *t) { busy_text = t; page = PAGE_BUSY; redraw(); }
static void show_answer(void)        { page = PAGE_ANSWER; redraw(); }

/* ------------------------------------------------------------------ actions */
static void action_read(void)
{
  show_busy("Reading module");
  link_command("cfg", 1500, &reply);
  show_answer();
}

static void action_clear(void)
{
  show_busy("Clearing setup");
  link_command("cfg clear", 2500, &reply);
  show_answer();
}

static void action_bind(void)
{
  show_busy("Bind request");
  link_command("cfg bind", 2500, &reply);
  show_answer();
}

static void action_send(void)
{
  char cmd[32], num[12];
  strcpy(cmd, role_tx ? "cfg tx " : "cfg rx ");                    /* numbers, as the serial frames carry them */
  number(num, modes[mode_idx].prot); strcat(cmd, num); strcat(cmd, " ");
  number(num, modes[mode_idx].sub);  strcat(cmd, num);
  if (role_tx && option_shown()) strcat(cmd, option_dual ? " dual" : " single");
  if (!role_tx) { strcat(cmd, " "); number(num, rx_out); strcat(cmd, num); }   /* the output layout (older module firmware ignores it) */
  show_busy("Sending setup");
  link_command(cmd, 3000, &reply);                                 /* the module stores, answers, blinks, restarts */
  bool stored = reply.answered && strstr(reply.note, "Stored") != 0;
  show_answer();
  if (stored) {                                                    /* let it come up again, then ask what it runs */
    board_delay_ms(2500);
    show_busy("Module restarted,");
    link_reply_t again;
    link_command("cfg", 1500, &again);
    if (again.answered) { reply = again; strcpy(reply.note, "Stored and running"); }
    show_answer();
  }
}

/* after a read: preset the menu to what the module holds */
static void preset_from_reply(void)
{
  if (!reply.answered || reply.stored[0] == 0 || reply.stored[0] == 'N') return;
  if (!reply.setup) return;
  role_tx = reply.setup_tx;
  int i = mode_of(reply.prot, reply.sub);
  if (i >= 0) mode_idx = i;
  if (!mode_fits(mode_idx)) step_mode(1);
  if (reply.opt >= 0) option_dual = (reply.opt == 1);
  if (!reply.setup_tx && reply.outputs >= 0) rx_out = (uint8_t)reply.outputs;
}

/* ------------------------------------------------------------------ main */
int main(void)
{
  board_init();
  board_backlight(true);
  serial_init();
  display_init();

  display_begin();
  do {
    draw_text(0, 1, "RF MODULE");
    draw_text(0, 2, "CONTROLLER v" VERSION);
    draw_text(0, 4, "reading module...");
  } while (display_end());
  board_delay_ms(500);
  link_command("cfg", 1200, &reply);
  preset_from_reply();
  redraw();

  uint8_t scan1 = 0, scan2 = 0, held = 0;
  uint32_t back_since = 0, page_since = 0;
  bool back_long = false;

  for (;;) {
    board_delay_ms(15);
    uint8_t raw = board_keys();
    uint8_t keys = (uint8_t)(raw & scan1 & scan2);                 /* a key counts when three scans agree */
    scan2 = scan1; scan1 = raw;
    uint8_t pressed  = (uint8_t)(keys & ~held);
    uint8_t released = (uint8_t)(held & ~keys);
    held = keys;

    if (keys & KEY_BACK) {                                         /* BACK held two seconds: clear */
      if (back_since == 0u) back_since = board_ms() | 1u;
      else if (!back_long && (board_ms() - back_since) >= 2000u) { back_long = true; action_clear(); }
    } else back_since = 0;

    if (page != PAGE_MENU) {                                       /* answer page: any key, or 8 s, back to the menu */
      if (page_since == 0u) page_since = board_ms() | 1u;
      bool leave = (pressed & (KEY_INC | KEY_DEC | KEY_ENTER | KEY_AUX)) || (board_ms() - page_since) >= 8000u;
      if (released & KEY_BACK) { if (!back_long) leave = true; back_long = false; }
      if (leave) { page = PAGE_MENU; page_since = 0; redraw(); }
      continue;
    }
    page_since = 0;

    if (editing) {
      if (pressed & KEY_INC)   { edit_step(1);  redraw(); }
      if (pressed & KEY_DEC)   { edit_step(-1); redraw(); }
      if (pressed & KEY_ENTER) { editing = false; redraw(); }
    } else {
      if (pressed & KEY_INC) {
        cursor = (uint8_t)(cursor == ROW_ROLE ? ROW_BIND : cursor - 1u);
        if (cursor == ROW_OPTION && !option_shown()) cursor = ROW_PROTO;
        redraw();
      }
      if (pressed & KEY_DEC) {
        cursor = (uint8_t)(cursor == ROW_BIND ? ROW_ROLE : cursor + 1u);
        if (cursor == ROW_OPTION && !option_shown()) cursor = ROW_SEND;
        redraw();
      }
      if (pressed & KEY_ENTER) {
        if (cursor == ROW_SEND) action_send();
        else if (cursor == ROW_BIND) action_bind();
        else { editing = true; redraw(); }
      }
    }
    if (released & KEY_BACK) { if (!back_long) action_read(); back_long = false; }
    if (pressed & KEY_AUX) action_bind();
  }
}
