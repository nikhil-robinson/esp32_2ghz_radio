# ESP-SDR S3 radio library — design

Status: draft for review (rev 2) · Branch: `s3-radio-lib` · Date: 2026-10-12

## 1. Goal and phases

Turn the ESP32-S3's built-in 2.4 GHz radio into a clean, documented, fast C/C++
library plus one firmware, so developers can receive and transmit on 2.4 GHz with a
~$2 board.

- **Phase 1 — full rewrite (this spec's main subject).** Every S3 file is rewritten
  line by line: proper module and function names, consistent standards, bugs fixed,
  stale code removed, hot paths kept or made faster. **No new features.** Every
  capability that works today must still work, measured as well or better.
- **Phase 2 — new features (summarised in §12, own spec later).** TX (GFSK,
  BLE/nRF24/XN297/LT8900 framers), region model, full-band scan.

Both phases are ESP32-S3 only. The multi-chip code stays on `main` untouched.

## 2. Phase 1 scope

**Rewritten:** the 31 files the S3 build uses today (≈ 4 200 lines, list in
Appendix A), the build system, Python tools and tests.

**Deleted on this branch:**

- All non-S3 targets and families, S31 streaming + web UI, `platform/esp32s2/`,
  per-chip `sdkconfig.defaults.*`, `partitions-stream.csv`.
- Web/browser tooling: `.github/workflows/rebuild-site.yml`,
  `tools/export_web_firmware.py`, `tools/collect_firmware.py` and their tests.
- Experimental probe builds that are never packaged: `S3_RF_PROBE`,
  `SAMPLE_RATE_PROBE`, `FILTER_REGISTER_PROBE`, `RING_PROBE`,
  `main/diagnostics/`. (Their verified findings are kept as constants/comments.)
- Chip-specific tests for deleted targets.

**Wire protocol:** clean break. The browser app is not supported.

## 3. Capability parity list (must all survive)

From `receiver.c` / `ring_capture.c` today:

| Area | Capability |
|---|---|
| Identity | info, capabilities, version record, tuning range |
| Transport | USB-Serial-JTAG; UART0 bridge with configurable pins/baud; runtime baud change; port ownership lease |
| Tuning | frequency set 1840–2790 MHz (direct PLL + 6/5 alternate LO plan), PLL fine offset |
| Front end | gain manual/auto (HW AGC), RX bandwidth / LPF code, RX limits, DC removal mode, recalibration on staleness |
| Burst capture | 256–16 380 samples at 16 / 40 / 80 Msps; formats IQ32, IQ10 packed, IQ8; repeated runs |
| Continuous ring | gapless stats run, gapless multi-unit capture, sentinel-verified continuity |
| On-chip spectrum | NFFT 256–2048, stride, units/frame, average or max-hold, optional stats, dual-core worker with assist mode |
| IQ stream | decimated continuous IQ (FIR), bits/shift, optional +fs/4 rotation |
| GPIO | GPIO command set |

## 4. Repository layout

```
components/esp_sdr/
  include/esp_sdr.h            public C API
  include/esp_sdr.hpp          header-only C++ wrapper (no exceptions, no heap)
  src/core/        sdr.c (init/state), error.c, version.c
  src/transport/   transport.c (USB + UART, lease), frame.c (wire framing, CRC)
  src/hal/         ONLY place with undocumented registers / ROM / libphy calls
      pll.c        tuning + LO plan
      frontend.c   gain, AGC, LPF/bandwidth, limits, DC
      calibration.c  recalibration + calibration state
      capture.c    burst capture engine (dump SRAM window)
      ring.c       continuous ring + bank rotation + continuity proof
      core1.c + core1_entry.S   second-core worker bring-up
      *.S          unpack, FFT rounding, FIR dot product (kept, re-commented)
  src/dsp/         spectrum.c, fir.c, stats.c, iq_pack.c
  src/app/         gpio.c
components/esp-dsp/            submodule v1.8.2
firmware/esp_sdr_fw/           main.c + command dispatcher, Kconfig, sdkconfig.defaults
host/esp_sdr/                  Python host library (protocol client) used by tools + tests
tests/host/                    pure-C unit tests (host gcc) + Python unit tests
tests/hw/                      hardware parity + performance suite
tools/                         build.py, version.py
docs/                          README, protocol.md, architecture.md
```

Rules: files and functions are named for what they do (`ring_start_run`, not
`c1_encode_step`); one module, one responsibility; no file over ~600 lines.

## 5. Coding standards

- **C:** C17, ESP-IDF style, `clang-format` config checked in; every public symbol
  prefixed `esp_sdr_`; module-private symbols `static`; `snake_case` verbs.
- **Headers:** each `.c` has one header; no `.inc`/`.h` files containing function
  bodies except `static inline` helpers that are proven hot.
- **Errors:** every fallible call returns `esp_err_t`; no silent `(void)` on
  results that can fail; `ESP_ERROR_CHECK` only in `main.c` start-up.
- **Magic numbers:** every register address, bit and ROM symbol gets a named
  constant/extern in `src/hal/*` with a one-line provenance comment.
- **Warnings:** `-Wall -Wextra -Werror` on our code; `cppcheck` clean in CI.
- **Hot paths:** IRAM placement, masked-interrupt windows and assembly kept where
  measured necessary; each documented with its timing budget.
- **ESP-IDF:** build on 5.3.x with version checks replacing the force-included
  compat header; CI builds against the pinned IDF.
- **Python:** 3.10+, type hints, `ruff` clean.

## 6. Wire protocol (new)

Binary, framed, both directions:

```
| magic 0xE5 0xD5 | type u8 | seq u8 | len u32 LE | payload | crc32 LE |
```

- Host → device: `type` = command id, payload = packed little-endian struct.
- Device → host: response frames echo `seq`; status code first; bulk data
  (captures, spectra, IQ) travels in the same frame format, so one parser
  handles everything and every byte is CRC-checked.
- Command ids and payload structs live in one header shared with the Python
  host library (generated, not hand-copied).
- `docs/protocol.md` documents every command.

## 7. Bugs and debt to fix

| # | Issue | Fix |
|---|---|---|
| 1 | `firmware-targets.json` missing → `build_firmware.py` and CI fail | replaced by `tools/build.py` with S3 profile inline |
| 2 | Code requires IDF ≥ 5.4 APIs; installed IDF is 5.3.5 | version-checked HAL helpers, no force-included header |
| 3 | SPEC worker processes only ~11–22 % of blocks at 40 Msps; advertised profiles overstate it | profile measured, root-caused, table regenerated from measurements |
| 4 | RX I/Q spectrally mirrored | corrected once in HAL; verified with a known tone |
| 5 | Advertised tuning range 100–6000 MHz is not real | range = verified 1840–2790 MHz |
| 6 | Static buffers can collide with the RF ring guard | explicit memory map in one place; guard kept |
| 7 | 3 ms fixed settle in RX prepare | measured minimum, made a named constant |
| 8 | Further defects found during the line-by-line audit | logged in `docs/refactor-log.md`, each with a test |

## 8. "Not breaking it" — verification gates

1. **Baseline first.** Before any change, the current firmware is measured with a
   recorded suite (§9) and results committed to `tests/hw/baseline.json`.
2. **Incremental.** The rewrite proceeds module by module; after every step the
   firmware builds and the hardware suite passes. No big-bang switch.
3. **Equal or better.** Each metric must match or beat the baseline (CRC-exact
   data paths, zero drops where baseline had zero, timings ≤ baseline + 2 %).
4. **Rollback point.** `main` is untouched; every green step is a commit.

## 9. Test suite

| Layer | Content | Runs |
|---|---|---|
| Host C | IQ pack/unpack, FIR, spectrum math, frame encode/decode + CRC, LO plan, gain tables | host gcc, CI |
| Host Python | protocol client encode/decode | CI |
| HW parity | every row of §3 exercised; CRC-verified data | 1 board |
| HW RF | 2-board known-tone test: frequency accuracy, mirroring, gain steps | 2 boards (one transmits a tone via spike firmware until Phase 2) |
| HW perf | capture duration per rate, ring drops at 16/40/80 Msps, SPEC processed %, IQ stream rate, USB throughput | 1 board |

## 10. Risks

| Risk | Mitigation |
|---|---|
| Rewrite changes timing-critical code and introduces drops | Baseline + perf gate after every module; asm untouched unless measured |
| Undocumented behaviour lost while renaming | Provenance comment per magic value; parity suite covers every capability |
| IDF/libphy updates break HAL | All such code in `src/hal/`; pinned IDF in CI |

## 11. Out of scope

Other chips, browser app, CC1101/sub-GHz, Arduino packaging, Phase 2 features.

## 12. Phase 2 summary (separate spec after Phase 1 lands)

Built on the Phase 1 library, using facts proven on hardware on 2026-10-12:

- TX: tone generator `0x60006040[9:0]` (156 kHz/LSB, fine bits `0x60006050[1:0]`),
  `txcal_debuge_mode`/`start_tx_tone_step` inside the RX firmware, ~4 ms TX→RX
  switch; CPU-timed GFSK at 250 k / 1 M / 2 Mbps; stock BLE scanner accepted
  hand-built adverts (+dev polarity).
- Framers: BLE adv, nRF24 ESB (+BK2423), XN297, LT8900.
- Region chosen at init with a one-time legal warning; user-defined bands allowed.
- Full-band scan 1840–2790 MHz.

## Appendix A — files rewritten

`main/targets/esp32s3/{receiver.c, s3_core1.S, s3_unpack.S, s3_fft_rnd.S,
s3_firdot.S, linker.lf, sram_guard.ld}`, `main/common/{ring_capture.c/.h,
burst_serial.c/.h, burst_gpio.c/.h, burst_version.c/.h, rx_recalibration.c/.h,
rx_calibration_state.c, rx_calibration_link.cmake, spectrum_stats.c/.h,
spectrum.h, spectrum_dc.h, ring_io.h, burst_gain.h, burst_limits.h, rx_lo.h,
rx_tuning.h, rx_bandwidth.h}`, `main/diagnostics/filter_probe.h` (deleted),
`main/CMakeLists.txt`, `main/Kconfig.projbuild`, `CMakeLists.txt`,
`sdkconfig.defaults(.esp32s3)`, `partitions.csv`, `tools/*`, `tests/*`,
`.github/workflows/firmware.yml`.
