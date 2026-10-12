# ESP-SDR S3 radio library — design

Status: draft for review · Branch: `s3-radio-lib` · Date: 2026-10-12

## 1. Goal

Turn the ESP32-S3's built-in 2.4 GHz radio into a single, documented C/C++ library
that can **receive** (raw I/Q, spectrum, full-band scan) and **transmit** (GFSK
packets for common low-cost 2.4 GHz protocols), so developers can talk to their own
RC/robotics gear with a ~$2 board instead of buying dedicated radios.

Success for v1:

- One firmware image does RX and TX on the same board.
- A stock BLE receiver accepts BLE advertisements produced by the library.
- nRF24 / XN297 / LT8900 frames produced by the library decode correctly in
  datasheet-based decoders, verified on air with a second board.
- A full scan of 1840–2790 MHz runs and locates a known transmitter.
- The existing S3 RX hardware smoke test still passes.

## 2. Scope

**In:** ESP32-S3 only. RX (burst capture, continuous ring, SPEC FFT, IQ stream —
the existing engine), full-band scan, TX tone + GFSK, protocol framers for BLE
advertising, nRF24 Enhanced ShockBurst (also covers BK2423), XN297, LT8900;
region-based TX permission; serial firmware; examples; host + hardware tests.

**Out (later versions):** other ESP32 chips (preserved on `main`), Spektrum DSM,
LoRa/ELRS, CC1101 sub-GHz backend, frequency-hopping protocols with binding,
Arduino packaging, raw-I/Q TX.

## 3. Established facts this design relies on

Verified on hardware with two S3 boards on 2026-10-12 (spike code, not kept):

| Fact | Evidence |
|---|---|
| Tone generator freq field `0x60006040[9:0]` is live-writable; 156.25 kHz/LSB; bit 1 = centre − dev | Tone moved as commanded; stock BLE scanner accepts +dev, rejects −dev |
| CPU-timed FSK works at 250 kbps, 1 Mbps, 2 Mbps (±312 kHz) | Transition counts exact on SDR |
| Hand-built BLE ADV_NONCONN_IND received by stock NimBLE scanner | ~26 % of sent on ch37 (ceiling ≈ 33 %: scanner rotates 37/38/39) |
| TX works inside RX firmware without RF-test mode: `txcal_debuge_mode()` + `start_tx_tone_step()`; off: `start_tx_tone_step(0…)` + `txcal_work_mode()` | Both boards TX→RX→TX, 40–54 dB |
| TX→RX switch ≈ 4 ms (3 ms is the fixed settle in `prepare_rx`) | Measured |
| PLL divider `N = 4f/(3·f_xtal) − 32` (8-bit int): nothing below 960 MHz; VCO locks ≈ 2.2–2.65 GHz direct, 1842–2209 via 6/5 plan | `libphy` disassembly; no TX/RX at 315/433/868/915/5800 MHz; no FM at 100 MHz |
| RX proven 1840–2790 MHz | Author's measurements |
| SDR RX I/Q is spectrally mirrored | Inverted polarity decodes on SDR while scanner needs +dev |
| Builds on ESP-IDF 5.3.5 with `tests/hw/idf53_compat.h` | `s3_smoke.py` ALL PASS |

Open, to be resolved during implementation (each has a task in the plan):

- Fine frequency bits `0x60006050[1:0]` (39 kHz) behave as expected → needed for
  Gaussian shaping and ±160 kHz nRF24 deviation.
- TX output usable across 1840–2790 MHz (only 2402–2480 measured so far).
- Correct the RX I/Q mirroring at the library boundary (one place) without
  breaking the existing RX tests.

## 4. Repository layout (branch `s3-radio-lib`)

```
components/esp_sdr/
  include/esp_sdr.h          public C API
  include/esp_sdr.hpp        C++ wrapper (header-only, no exceptions, no heap)
  src/core/                  init, region table + checks, caps, errors
  src/hal/esp32s3/           ONLY place with undocumented registers / ROM / libphy calls
      tune.c                 PLL + LO plan (from receiver.c / rx_lo.h)
      rx_capture.c           burst capture (from receiver.c capture())
      ring_capture.c, *.S    continuous ring / SPEC / IQS (moved as-is)
      calib.c                rx_recalibration, calibration state
      tx_tone.c              tone on/off, field writes, cycle-timed player
  src/dsp/                   spectrum, stats, fft wrapper (moved from main/common)
  src/proto/                 gfsk_shape.c, ble_adv.c, nrf24_esb.c, xn297.c, lt8900.c (pure C)
components/esp-dsp/          submodule, pinned v1.8.2
firmware/esp_sdr_serial/     the single RX+TX firmware (serial command protocol)
examples/ble_beacon/  examples/nrf24_tx/  examples/spectrum_scan/
tests/host/                  pure-C unit tests (host gcc) + Python runners
tests/hw/                    s3_smoke.py, tx_ble_scanner.py, tx_proto_decode.py, scan.py
tools/                       build/export/version scripts, S3 only
docs/                        README, API reference, legal note, protocol notes
```

Removed on this branch: `main/targets/*` except S3 code moved into the HAL,
`main/families/`, S31 streaming + web UI, `platform/esp32s2/`, per-chip sdkconfigs,
chip-specific tests. `firmware-targets.json` (missing today) is recreated with the
single S3 profile so `tools/build_firmware.py --matrix` and CI work again.

## 5. Public API

C is the source of truth; C++ wraps it 1:1. All calls return `esp_err_t`.

```c
typedef enum { ESP_SDR_REGION_UNSET, ESP_SDR_REGION_EU, ESP_SDR_REGION_US,
               ESP_SDR_REGION_JP, ESP_SDR_REGION_IN, ESP_SDR_REGION_USER } esp_sdr_region_t;
typedef struct { uint16_t lo_mhz, hi_mhz; int8_t max_dbm; } esp_sdr_band_t;

esp_err_t esp_sdr_init(const esp_sdr_config_t *cfg);          /* region may be UNSET */
esp_err_t esp_sdr_set_region(esp_sdr_region_t r);
esp_err_t esp_sdr_set_user_bands(const esp_sdr_band_t *b, size_t n); /* REGION_USER */

/* RX */
esp_err_t esp_sdr_rx_tune(uint32_t mhz);                      /* 1840..2790 */
esp_err_t esp_sdr_rx_capture(esp_sdr_rate_t r, uint32_t *iq_words, size_t n);
esp_err_t esp_sdr_rx_scan(const esp_sdr_scan_cfg_t *c, esp_sdr_scan_cb_t cb, void *ctx);

/* TX */
esp_err_t esp_sdr_tx_tone(uint32_t mhz, int32_t offset_khz, bool on);
esp_err_t esp_sdr_tx_gfsk(const esp_sdr_gfsk_cfg_t *c, const uint8_t *bits, size_t nbits);

/* Protocol framers: bytes in, LSB/MSB-correct air bits out (no hardware). */
size_t esp_sdr_frame_ble_adv(const esp_sdr_ble_adv_t *p, uint8_t *bits, size_t cap);
size_t esp_sdr_frame_nrf24(const esp_sdr_nrf24_t *p, uint8_t *bits, size_t cap);
size_t esp_sdr_frame_xn297(const esp_sdr_xn297_t *p, uint8_t *bits, size_t cap);
size_t esp_sdr_frame_lt8900(const esp_sdr_lt8900_t *p, uint8_t *bits, size_t cap);
```

```cpp
esp_sdr::Radio radio;
radio.begin({.region = esp_sdr::Region::EU});
radio.tx().send(esp_sdr::Nrf24{.mhz = 2476, .addr = {0xE7,0xE7,0xE7,0xE7,0xE7}, .payload = data});
radio.rx().scan({.from = 1840, .to = 2790, .step_mhz = 4}, on_bin);
```

RX and TX are mutually exclusive on the hardware; the library switches modes
internally and reports the switch time in `esp_sdr_caps()`.

## 6. Region / legal model

- Boot state `UNSET`: every TX call returns `ESP_ERR_INVALID_STATE`; RX and scan
  are always allowed.
- First TX after a region is set logs a one-time warning that the user is
  responsible for local radio law.
- Each region maps to a band table with a power cap; TX requests outside it return
  `ESP_ERR_NOT_ALLOWED`. `REGION_USER` lets the user supply their own table.
- The effective TX range is (region bands) ∩ (frequencies verified for TX on S3).
- The legal note in `docs/` frames protocol support around the user's own
  equipment.

## 7. TX engine

- **Tuning:** LO set via the existing `s3_tune()` path to the channel centre; the
  tone field carries only the modulation (no ±10 MHz offset trick).
- **GFSK:** `gfsk_shape.c` precomputes per-sample frequency words from bits with a
  Gaussian filter (BT 0.5, 4 samples/bit) quantised to the 39 kHz fine grid
  (`0x60006040[9:0]` + `0x60006050[1:0]`); falls back to plain FSK at 156 kHz
  resolution if the fine bits do not behave.
- **Player:** IRAM function, interrupts masked, cycle-counted against
  `esp_cpu_get_cycle_count()` at 240 MHz; words in a heap buffer (static BSS
  crosses the RF ring guard). Max burst length bounded so interrupts stay masked
  ≤ 2 ms.
- **Rates:** 250 kbps, 1 Mbps, 2 Mbps.

## 8. Protocol framers

| Framer | Air format | Reference vectors |
|---|---|---|
| `ble_adv` | preamble, AA 0x8E89BED6, PDU, CRC-24 (init 0x555555), whitening | Ubertooth `btle_calc_crc`; spike-verified packet |
| `nrf24_esb` (+BK2423) | preamble, 3–5 B address, 9-bit PCF, payload, CRC-8/16, MSB first | nRF24L01+ datasheet §7.3 |
| `xn297` | nRF24 layout + XN297 scrambling and bit-reversal rules | DIY-Multiprotocol-TX-Module `XN297_EMU` |
| `lt8900` | preamble, sync words, trailer, length, payload, CRC-16 | LT8900 datasheet; Multiprotocol `LT8900_SPI` |

## 9. Spectrum scan

Steps the LO across 1840–2790 MHz (default 4 MHz step under 16 Msps so windows
overlap), runs the existing SPEC FFT per step, de-mirrors I/Q, stitches bins and
reports `(freq_hz, power_db)` through the callback. Serial firmware exposes it
as `SCAN <from> <to> <step>`.

## 10. Serial firmware

Keeps today's RX commands unchanged (browser compatibility) and adds `REGION`,
`TXTONE`, `TXGFSK`, `TXBLE`, `TXNRF`, `TXXN297`, `TXLT8900`, `SCAN`, `CAPS`
(lists TX caps). Unknown commands still return `ERR command`.

## 11. Testing

| Layer | Test | Runs |
|---|---|---|
| Framers | CRC/whitening/scrambling vs reference vectors | host gcc, CI |
| GFSK shape | Word sequence for known bits; deviation within ±1 step | host gcc, CI |
| Region | UNSET blocks TX; out-of-band rejected; USER table honoured | host gcc, CI |
| RX regression | `tests/hw/s3_smoke.py` ALL PASS | 1 board |
| GFSK on air | Stock NimBLE scanner receives `TXBLE` (≥ 20 % on one channel) | 2 boards |
| Protocols on air | Python decoders verify nRF24/XN297/LT8900 address + CRC | 2 boards |
| Scan | Finds a TX tone placed at a known frequency within one step | 2 boards |

Hardware tests stay outside `unittest discover`; CI runs host tests only.

## 12. Risks

| Risk | Mitigation |
|---|---|
| ESP-IDF / libphy update breaks undocumented calls | All such code in `src/hal/esp32s3/`; pinned IDF; hardware tests per release |
| Fine-frequency bits don't work | Plain-FSK fallback; nRF24 at ±156 kHz tested by SDR decoder |
| Frames correct on SDR but rejected by real clone chips | Documented as unverified on real chips; BLE scanner remains independent proof |
| Interrupts masked during TX starve USB | Burst cap ≤ 2 ms; command acks only after burst ends |
