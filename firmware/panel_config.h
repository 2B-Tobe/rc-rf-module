/*
 * panel_config.h — TX panel build options. Change the two lines below, save, then build (.\build.ps1) and flash.
 * Only used on a board with the panel display; a board without it runs as before whatever is set here.
 */
#ifndef PANEL_CONFIG_H
#define PANEL_CONFIG_H

/* ---- Input: what the module listens to ------------------------------------------------------------------- */
#define PANEL_IN_MULTI      1   /* MULTI only:  MULTI frames on J3 RX (EdgeTX radios, MiTo encoders); no PPM          */
#define PANEL_IN_PPM        2   /* PPM only:    PPM on the PPM input (J6 pin 3, PB01); J3 RX never used for MULTI     */
#define PANEL_IN_MULTI_PPM  3   /* MULTI + PPM: whichever comes first; menu item Input (Auto / PPM only)              */

#define PANEL_INPUT   PANEL_IN_MULTI_PPM

/* ---- Buttons ----------------------------------------------------------------------------------------------- */
/* 3: Up (PC00), Down (PC01), Enter (BIND).  4: + Back (PC02, J1 pin 4) - Back leaves a page or the menu, and the
   menu has no Back line.                                                                                         */
#define PANEL_BUTTONS 3

#endif
