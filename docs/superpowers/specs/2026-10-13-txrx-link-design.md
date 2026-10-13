# ESP-SDR: TX+RX packet link (initiator and responder) — design

Status: implemented and verified on hardware 2026-10-13 (A and B) · Branch: `s3-radio-lib`

## Goal

Two-way packet exchange on one board: (A) send a packet and catch the reply,
(B) receive a packet and answer it within a fixed turnaround (target
130-150 µs, the nRF24 ESB / BLE T_IFS range).

## Findings that shape the design (spikes, 2026-10-13)

* TX -> RX needs no 3 ms settle; the old retune cost 128 µs.
* The dump engine **owns** the SRAM bank it writes: the CPU cannot read a
  bank while it is selected. Real-time reception therefore ping-pongs banks 0
  and 1 every P pairs and processes the bank just released; unit edges are
  found with marker words (as in the ring). P = 256 pairs: worst 16 µs from
  sample to processed, 89 % of core 0; P = 512: 32 µs, 79 %.
* Detector: samples 0 and 2 of every 4 (signal at fs/4) -> 4 Msps complex,
  sign of the cross product = FM bit, one 32-bit shift register per sampling
  phase, match on the last 32 sync bits. 98 % of 1 Mbps packets detected, no
  false matches in 1 s of noise. The loop must stay 32-bit with state in
  locals (64-bit or struct state fell behind real time).

## Design

* `ESP_SDR_MODE_TXRX`: receiver init plus the transmitter; both APIs work.
* One LO for both directions, 3.984375 MHz (102 fine tone steps) below the
  channel. TX reaches the channel through the tone generator offset; RX hears
  the channel at ~fs/4. No retune between directions.
* `esp_sdr_link_t`: channel (kHz), bit rate (250k / 1M / 2M), nRF24-style
  framing (3-5 address bytes in air order, CRC 0/1/2, fixed payload length per
  direction), attenuation.
* `esp_sdr_link_exchange()`: send, switch to RX, return the first packet with
  a valid CRC within `timeout_us`, with its arrival time after our TX end.
* `esp_sdr_link_respond()`: receive until a packet with a valid CRC arrives
  (or `timeout_ms`), then transmit the reply `turnaround_us` after that
  packet's last bit; returns the payload and the achieved turnaround.
* Firmware: `SET_MODE` 2, `LINK_EXCHANGE`, `LINK_RESPOND`; host `Device`
  methods.

## Verification

* Host unit tests for the bit collector / CRC check path (pure C).
* `tests/hw/txrx_link.py`: board A responds, board B initiates; reply rate and
  turnaround at 250k/1M/2M and CRC 1/2; then roles swapped.

## Results (two ESP32-S3 boards, IDF 5.3.5, `tests/hw/txrx_link.py`)

| | 250 kbps | 1 Mbps | 2 Mbps |
|---|---|---|---|
| replies / exchanges (each board as responder, CRC 1 and 2) | 199-200 / 200 | 200 / 200 | 179-191 / 200 |
| reply seen by the initiator after its last bit (target 130 µs) | 125.9-127.5 µs | 130.2-131.2 µs | 130.4-130.7 µs |

* The responder measures its own turnaround as 130.0 µs every time; the
  initiator's estimate of the reply start reads ~4 µs early at 250 kbps.
* Turnaround sweep at 1 Mbps (50 exchanges each): 1000, 300, 200, 150 and
  130 µs all 48-50/50, replies within +0.2..+1.9 µs of the target.
* 2 Mbps: only 2 detector samples per bit, so the matching phase is sometimes
  at the eye edge; 5-10 % of bodies fail CRC (known limitation).
* Detector load: worst bank visit 5.7k of 7.7k cycles (74 % of core 0).

### Lessons (each cost a debugging round)

* The dump engine owns the bank it writes: the CPU reads garbage from it.
* The hot loop must be 32-bit with state in registers; two variables named `k`
  (loop counter shadowing the integrator shift) silently broke it.
* AGC raises the wideband noise until a short on-chip filter cannot separate
  bits: the link needs a fixed RX gain (40 used in the tests).
* Collecting the body on a phase later than the matching one must skip that
  phase's first sample (still the last sync bit) - an off-by-one that failed
  every CRC at 250 k / 1 M.
* A retune after TX costs 0.7-0.9 ms; re-entering RX on the same LO takes
  ~27 µs and is enough.

## Out of scope (v1)

ESB packet control field and auto-ACK compatibility with real nRF24 chips,
BLE scan responses / connections: they need the other side's hardware to
verify.
