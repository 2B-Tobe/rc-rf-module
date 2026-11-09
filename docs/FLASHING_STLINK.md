# Programming the module with an ST-Link

Everything a board needs can go in through its SWD pins: the firmware, and the setup that makes it a
transmitter module or a receiver. No USB-serial adapter, terminal program or controller box is needed.

## What you need

- An ST-Link V2 (an original or a clone) and three wires.
- The module, powered from its normal supply.
- A Windows PC with Python 3.
- The files in `firmware/Prebuilt/`: the firmware `rc_rf_module_1.4.hex`, and the setup files in `setup/`.

## 1. Prepare the PC (once)

1. Install Python 3 from python.org. In the installer, tick **Add python.exe to PATH**.
2. Open PowerShell and run, one line at a time:
   ```
   pip install pyocd
   python -m pyocd pack install efr32mg24a420f1536im48
   ```
3. Plug the ST-Link into the PC and run:
   ```
   python -m pyocd list
   ```
   It should list one probe (STLink). If the list is empty, install ST's driver **STSW-LINK009** from st.com,
   unplug and replug the ST-Link, and try again.

## 2. Connect the module

| ST-Link | Module, header J5 (SWD) |
|---|---|
| GND | GND |
| SWDIO | SWDIO |
| SWCLK | SWCLK |

J5 carries no supply: power the module as usual, from its own supply.

## 3. Flash the firmware

In PowerShell, go to the folder with the files, then flash:
```
cd C:\dev\rc-rf-module\firmware\Prebuilt
python -m pyocd flash -t efr32mg24a420f1536im48 rc_rf_module_1.4.hex
```
pyOCD ends with a line like `Erased … programmed … bytes`. The stored setup and the binds are kept: a firmware
update never touches them.

## 4. Set the role

Flash one setup file from `setup/`. It writes only the setup page; the firmware is not touched.
```
python -m pyocd flash -t efr32mg24a420f1536im48 setup\setup_tx.hex
```

| File | The board becomes |
|---|---|
| `setup_tx.hex` | transmitter module, 2.4 GHz only (in an EdgeTX radio or with an encoder; the radio's frames choose the protocol) |
| `setup_tx_dual.hex` | transmitter module, with the 459 MHz link beside 2.4 GHz (RA-01 fitted) |
| `setup_rx_rlink.hex` | RLINK receiver, 8 servos |
| `setup_rx_rlink_sbus.hex` | RLINK receiver, 5 servos + SBUS on connector 6 |
| `setup_rx_rlink_ppm.hex` | RLINK receiver, 5 servos + PPM on connector 7 |
| `setup_rx_rlink_sbus_ppm.hex` | RLINK receiver, 5 servos + SBUS + PPM |
| `setup_rx_dsmx.hex` / `_sbus` | DSMX receiver, 8 servos / 5 + SBUS |
| `setup_rx_d8.hex` / `_sbus` | FrSky D8 receiver, 8 servos / 5 + SBUS |
| `setup_rx_d16.hex` / `_sbus` | FrSky D16 receiver, EU-LBT, 8 servos / 5 + SBUS |
| `setup_rx_d16_fcc.hex` / `_sbus` | FrSky D16 receiver, FCC, 8 servos / 5 + SBUS |
| `setup_rx_nrf24.hex` | nRF24 receiver, 8 servos |
| `setup_clear.hex` | nothing stored: the jumpers decide again |

Then bind as usual: for a receiver, the bind plug on servo connector 8 at power-up.

## A board for an EdgeTX radio

A board stored as a receiver stays a receiver, even in a radio bay. For the bay, flash `setup_tx.hex` (or
`setup_tx_dual.hex`) first.

## If it does not work

| pyOCD says | Check |
|---|---|
| `No connected debug probes` | the ST-Link's USB cable; the driver (step 1.3) |
| `Target type efr32mg24… not recognized` | the pack install (step 1.2) |
| cannot connect to the target | the module's power; GND, SWDIO and SWCLK; swap SWDIO and SWCLK if unsure; try a slower clock: add `-f 1000000` |
