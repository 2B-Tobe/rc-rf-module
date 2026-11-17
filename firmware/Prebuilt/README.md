# Prebuilt firmware

`rc_rf_module_1.7.hex`: built from this folder's sources on 17 November 2026 (1.7) and bench-tested on the author's
boards: the TX panel, and Futaba S-FHSS and FASST between two boards of this kind (experimental, not yet with Futaba
equipment: `docs/FUTABA.md`). Flash it with pyOCD over SWD, no toolchain needed:

```
python -m pyocd flash -t efr32mg24a420f1536im48 C:\dev\rc-rf-module\firmware\Prebuilt\rc_rf_module_1.7.hex
```

## Changes

- **1.8 (17 Nov 2026):** full flight power in every mode: S-FHSS and FASST (were +10 dBm) and DSMX (were 18 dBm at
  22 ms, 15 dBm at 11 ms) now at the module maximum, about +19.5 dBm, as the other modes. The power shown on the TX
  panel and in EdgeTX is rounded (19.5 reads 20).
- **1.7 (17 Nov 2026), experimental Futaba modes:** Futaba S-FHSS (Prot 21) and FASST 10ch (Prot 110), transmitter
  and receiver roles, at +10 dBm, from EdgeTX, the Controller (1.4), `cfg` and the TX panel. Bench-tested between
  two boards of this kind only, **not yet with Futaba equipment** (`docs/FUTABA.md`). The binding store has 12
  slots (S-FHSS and FASST receivers in 8 and 9); slots written by 1.6 and earlier are kept.
- **1.6 (17 Nov 2026):** the TX panel (`docs/TX_PANEL.md`): a display, three or four buttons and a red / green LED
  wired to pins the board already has. A board without the display runs as 1.5. Bench-tested as part of 1.7.
- **1.5 (15 Nov 2026):** a stored transmitter setup also takes PPM on J3 RX (PA06), the pin a radio's bay pin 1
  is wired to, so a radio with External RF set to PPM works on the same wiring as MULTI; the module uses
  whichever brings three frames first. On the Taranis Q X7 the PPM frames were read from the bay and MULTI
  worked as before (protocol changes, nRF24 to flight). The binding store's work area is off the stack.
- **1.4 update (10 Nov 2026):** a board stored as a receiver, plugged into an EdgeTX radio's JR bay, now finds
  the radio's frames at start-up, stores the transmitter setup by itself and restarts once as the transmitter
  module. Bench-tested on the Taranis Q X7, including bind.
- **1.4 (8 Nov 2026):** the module works in the JR bay of an EdgeTX radio as a MULTI (4-in-1) module, using
  Multiprotocol numbers, with MiTo (120) for nRF24 / RLINK / UK459. It shows the status, the protocol list and
  telemetry (`docs/SERIAL_INPUT.md` 2.3); the same firmware still serves the encoder. Single or dual band per
  model from EdgeTX: the Option value line (MiTo, FrSky) or Max throw (DSM).
  - **Receiver setup over the air:** an encoder can now set up a receiver (RxSetup). A receiver started with a bind
    plug on servo connector 8 listens for 3 s for its owner's setup.
  - **Receiver output layouts:** 8 PWM, or 5 PWM + SBUS / PPM / both (`cfg rx <proto> [8pwm|5sbus|5ppm|5both]`).
  - **Stored transmitter:** with no jumpers, a stored transmitter takes either serial frames or PPM, whichever
    comes first.

- **1.3 (26 Oct 2026):** FrSky D16 works with real receivers: the doubled sync word now comes from the radio settings;
  the Sub picks the version (0 v1 EU-LBT, 1 v2 EU-LBT, 2 v1 FCC, 3 v2 FCC, 5 v1 FCC 8 channels); radio-timed
  transmitter with listen-before-talk on EU; receiver fixes as for D8. Tested: FrSky RX8R and RadioMaster R88 on
  Sub 2 (telemetry from every packet), and the module's receiver bound to a Taranis Q X7 in D16.
- **1.2 (25 Oct 2026):** the module's own FrSky D8 receiver works at the FrSky data rate: preamble detection 16 bits,
  hop and telemetry reply driven by the radio, every received packet processed (no more lock-up after a minute);
  reconnects by itself when the transmitter comes back.
- **1.1 (25 Oct 2026):** FrSky D8 works with real FrSky-compatible receivers (tested: RadioMaster R88, FrSky D8R-II
  Plus): the D8/D16 radio settings now run at the FrSky data rates (31,044 / 100,000 bit/s), FrSky's channel spacing,
  a fixed -20 kHz frequency tune on the D8 transmitter, and packets timed by the radio itself. Fixes servo jitter
  with serial input: a false "input lost" moment parked all channels at 1400 µs. FrSky D16 with real receivers
  not yet tested.
- **1.0 (24 Oct 2026):** first published version.

To refresh it after a build and a bench test:

```
Copy-Item C:\dev\rc-rf-module\firmware\cmake_gcc\build\base\rc_module_trx.hex C:\dev\rc-rf-module\firmware\Prebuilt\rc_rf_module_1.7.hex
```

## Setting a board up with only an SWD programmer

The files in `setup/` write only the setup page (0x08042000), as the `cfg` commands would: `setup_tx.hex`,
`setup_tx_dual.hex`, receivers for RLINK, DSMX, D8, D16 and nRF24 with their output layouts, and `setup_clear.hex`.
The full step-by-step guide, with the list of files: `docs/FLASHING_STLINK.md`.

## Licence of the hex files

The firmware is under the GNU GPL v3 (see `LICENSE`). `rc_rf_module_1.7.hex` also contains Silicon Labs software:
the RAIL radio library (Zlib) and parts used under the Silicon Labs Master Software License Agreement
(https://www.silabs.com/about-us/legal/master-software-license-agreement). Those Silicon Labs parts may be used
only as part of this firmware on Silicon Labs devices; they may not be extracted, redistributed on their own or
reverse-engineered. The `setup/` files hold settings only, no Silicon Labs code.
