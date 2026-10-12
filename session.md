# Session notes: ESP32-S3 2.4 GHz SDR

Last updated: 2026-10-13 · Branch: `s3-radio-lib` (pushed, last commit `c6f0056`) · `main` untouched

## Goal

A cheap ESP32 board as a universal 2.4 GHz receive + transmit platform for
hobby and low-budget developers. The receive side is going into a research paper.

## Hardware rig

* `/dev/ttyACM0`: ESP32-S3 running `firmware/` (the new serial firmware).
* `/dev/ttyACM1`: ESP32-S3 running the throwaway RX+TX spike firmware
  (text commands `TXON <mhz> <atten>`, `TXF <field>`, `TXOFF`, `TXBLE <count> <gap_us> <dev>`).
* Toolchain used locally: ESP-IDF 5.3.5. CI builds 5.0–5.5 (results not checked yet).

## Done (phase 1: receiver)

* Full rewrite into a radio-only library `components/esp_sdr` plus a serial app `firmware/`.
  The protocol is defined once in `host/esp_sdr/protocol.py` and generated into headers and `docs/protocol.md`.
* Parity with the legacy firmware proven on hardware (`tests/hw/parity.py`, ALL PASS).
* RX spectrum was mirrored (raw Q inverted): fixed in the library, verified by `tests/hw/iq_orientation.py`.
* Tuning accurate to ±2 kHz over 1840–2790 MHz (`tests/hw/tuning_accuracy.py`, uses crystal harmonics).
* Full-band scan 1842–2790 MHz: 0.9 / 1.6 / 4.3 s at 80 / 40 / 16 Msps, with flatness correction,
  clock-spur suppression and a peak detector with dwell.
* IQ stream: 16 Msps input, decimation 32–1024, gap-free up to the ~800 KB/s USB link.

## Known limits

* Static memory headroom is only ~3.9 KB below the radio dump banks: keep large buffers on the heap.
* Absolute power (dBm) is not calibrated; needs a reference source.
* The `phy_param` calibration offset is hardware-verified on IDF 5.3 only.

## Next (phase 2)

1. Re-check the transmit tone-field sign inside the combined firmware, using a stock BLE scanner as the judge.
2. Add TX to the library (tone, GFSK packets), 2.4 GHz only, with region selection at init.
3. BLE advertising packets first, then the nRF24-style packet family for the user's own devices.
4. Merge `s3-radio-lib` into `main` once CI is confirmed.

## How to run

```bash
. $IDF_PATH/export.sh
idf.py -C firmware -p /dev/ttyACM0 flash
python3 -m unittest discover -s tests/host
python3 tests/hw/parity.py /dev/ttyACM0
python3 tools/scan.py /dev/ttyACM0 --rate 2 --out scan
```