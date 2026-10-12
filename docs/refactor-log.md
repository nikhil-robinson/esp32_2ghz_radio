# Rewrite log (phase 1, ESP32-S3)

Baseline: legacy firmware measured with `tests/hw/baseline_legacy.py` →
`tests/hw/baseline.json`. Result: `tests/hw/parity.py` ALL PASS on hardware
(ESP-IDF 5.3.5).

## Results vs baseline

| | Legacy | Rewrite |
|---|---|---|
| Burst 16 380 @ 80 / 40 / 16 Msps | 207 / 411 / 1027 µs | 207 / 411 / 1026 µs |
| Ring 2 s, all rates | gapless, late_max 37 | gapless, late_max 35 |
| Ring capture 3 units, all rates | 3/3 CRC | 3/3 CRC |
| Spectrum frames delivered (16M/256) | 264 /s | 433 /s |
| IQ stream dec 64: frames / drops | 88 / 401 | 489 / 0 |

Spectrum FFT coverage is 0.6–2.3 points lower at the higher frame rates: frames
are sent whenever the output queue drains, and a faster host receives more of
them, which costs core 1 encoding time. With a slower reader, coverage exceeds
the baseline (61.1 % vs 60.6 % at 16M/256).

## Fixed

1. **Fake spectrum parameters.** The legacy firmware accepted `stride` and
   `units_per_frame` and advertised profiles with them, but silently forced
   stride 1. Removed; coverage is reported honestly.
2. **Partition larger than flash.** `partitions.csv` declared a 4 MB app on a
   2 MB flash. Now 1 MB (image ≈ 600 KB).
3. **Unverified tuning range.** 100–6000 MHz was a software limit. The PLL
   cannot express below 960 MHz and locks ≈ 2.2–2.65 GHz directly; range is now
   the verified 1840–2790 MHz.
4. **ESP-IDF ≥ 5.4 requirement.** Two 5.4-only helpers replaced
   (register-level core-1 release, inline compare-and-set); `esp_gpio_reserve`
   (API differs per version) replaced by an explicit S3 pin mask.
5. **Inconsistent mean-power offset.** Single-core and dual-core paths used two
   different log formulas; both use the table version now.
6. **UART writes with interrupts masked** went through the UART driver; the
   stream sink now writes the FIFO via `uart_ll` (inline, IRAM-safe).
7. **Host-silence stop** fired after 2 s even when nothing was pending; it now
   requires pending output that the sink is not taking.
8. **Missing build inputs.** `firmware-targets.json` (deleted tooling) and an
   untracked, empty `components/esp-dsp` (now a pinned submodule).
9. **`M_PI` in strict C17** – not defined; explicit constant.

## Removed

Non-S3 targets, S31 streaming and web UI, the S2 USB console, browser/web-flasher
tooling, experimental probe builds (`S3_RF_PROBE`, `SAMPLE_RATE_PROBE`,
`FILTER_REGISTER_PROBE`, `RING_PROBE`), the text protocol and its tests. The
multi-chip code remains on `main`.

## Verified

* **Raw I/Q is spectrally mirrored (raw Q inverted) - fixed.** Proven without
  any assumption about a transmitter: a plain carrier at fixed 2445 MHz moves the
  wrong way as the LO steps 2441..2449 MHz. Corrected at the library boundary:
  `esp_sdr_word_q()` negates Q, spectrum bins are mirrored at encode time (free),
  the IQ stream negates Q at output, host decoders negate Q. Raw words stay raw.
  Verified by `tests/hw/iq_orientation.py` (burst, spectrum and IQ stream paths).
  An earlier tone-based test reported "correct" because the TX tone-field sign
  differs between the RF-test-mode spike and the combined TX firmware.
* **Tuning accuracy over the whole range**, `tests/hw/tuning_accuracy.py`: the
  receiver's 40 MHz crystal harmonics at 1920, 2160, 2400, 2520 and 2760 MHz
  appear within +-2 kHz of where they belong from every LO, including fractional
  LOs of the 6/5 divider band. The PLL offset sign is correct.

## Open

* `phy_param + 288` (DC-valid flag) is verified on hardware with the IDF 5.3 PHY
  library only (the symbol itself exists in 5.1 too).
